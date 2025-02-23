// ESP32 Dual-Core Uncanny Eyes with RD-03D Radar Integration
// Core 0: Radar processing
// Core 1: Eye animation and display rendering
// Eyes move left/right based on radar-detected human X position
// Eyes always on with occasional human-like blinking, optimized for stability

#define RX_PIN 16
#define TX_PIN 17
#define BAUD_RATE 256000

#include <SPI.h>
#include <TFT_eSPI.h>
TFT_eSPI tft;

#define BUFFER_SIZE 2048 // Increased to reduce SPI calls
#define BUFFERS 1
uint16_t pbuffer[BUFFERS][BUFFER_SIZE];

#include "config.h"

// Radar Variables
uint8_t RX_BUF[64] = {0};
uint8_t RX_count = 0;
uint8_t RX_temp = 0;
int16_t target1_x = 0, target1_y = 0;
int16_t target1_speed = 0;
uint16_t target1_distance_res = 0;
float target1_distance = 0;
float target1_angle = 0;
uint8_t Single_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};

// Shared variables for eye movement (protected by mutex)
int16_t eyeTargetX = 512;
SemaphoreHandle_t eyeXMutex;

TaskHandle_t radarTaskHandle = NULL;
TaskHandle_t eyeTaskHandle = NULL;

#define NOBLINK 0
#define ENBLINK 1
#define DEBLINK 2
typedef struct {
  uint8_t state;
  uint32_t duration;
  uint32_t startTime;
} eyeBlink;

struct {
  int16_t tft_cs;
  eyeBlink blink;
  int16_t xposition;
} eye[NUM_EYES];

uint32_t startTime;

void radarTask(void *pvParameters) {
  while (1) {
    while (Serial1.available()) {
      RX_temp = Serial1.read();
      RX_BUF[RX_count++] = RX_temp;

      if (RX_count >= sizeof(RX_BUF)) {
        RX_count = sizeof(RX_BUF) - 1;
      }

      if (RX_count >= 2 && RX_BUF[RX_count - 1] == 0xCC && RX_BUF[RX_count - 2] == 0x55) {
        processRadarData();
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void eyeTask(void *pvParameters) {
  while (1) {
    updateEye();
    vTaskDelay(25 / portTICK_PERIOD_MS); // ~40 FPS, more breathing room
  }
}

void processRadarData() {
  if (RX_count >= 32) {
    target1_x = (RX_BUF[4] | (RX_BUF[5] << 8)) - 0x200;
    target1_y = (RX_BUF[6] | (RX_BUF[7] << 8)) - 0x8000;
    target1_speed = (RX_BUF[8] | (RX_BUF[9] << 8)) - 0x10;
    target1_distance_res = (RX_BUF[10] | (RX_BUF[11] << 8));
    target1_distance = sqrt(pow(target1_x, 2) + pow(target1_y, 2));
    target1_angle = atan2(target1_y, target1_x) * 180.0 / PI;

    if (target1_distance < 10000 && target1_distance >= 0) {
      int16_t newEyeX = map(target1_x, -512, 511, 0, 1023);
      if (xSemaphoreTake(eyeXMutex, portMAX_DELAY) == pdTRUE) {
        eyeTargetX = newEyeX;
        xSemaphoreGive(eyeXMutex);
      }

      // Serial.print("Target 1 - Distance: "); // Reduced serial output
      // Serial.print(target1_distance / 10.0);
      // Serial.print(" cm, Angle: ");
      // Serial.print(target1_angle);
      // Serial.print(" degrees, X: ");
      // Serial.print(target1_x);
      // Serial.print(" mm, EyeX: ");
      // Serial.print(eyeTargetX);
      // Serial.println();
    }

    memset(RX_BUF, 0x00, sizeof(RX_BUF));
    RX_count = 0;
  }
}

void initEyes(void) {
  Serial.println("Initialise eye objects");
  for (uint8_t e = 0; e < NUM_EYES; e++) {
    Serial.print("Create display #"); Serial.println(e);
    eye[e].tft_cs = eyeInfo[e].select;
    eye[e].blink.state = NOBLINK;
    eye[e].xposition = eyeInfo[e].xposition;

    pinMode(eye[e].tft_cs, OUTPUT);
    digitalWrite(eye[e].tft_cs, LOW);

    if (eyeInfo[e].wink >= 0) pinMode(eyeInfo[e].wink, INPUT_PULLUP);
  }

#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
  pinMode(BLINK_PIN, INPUT_PULLUP);
#endif
}

void drawEye(uint8_t e, uint32_t iScale, uint32_t scleraX, uint32_t scleraY, uint32_t uT, uint32_t lT) {
  uint32_t screenX, screenY, scleraXsave;
  int32_t irisX, irisY;
  uint32_t p, a, d;
  uint32_t pixels = 0;

  digitalWrite(eye[e].tft_cs, LOW);
  tft.startWrite();
  tft.setAddrWindow(eye[e].xposition, 0, 240, 240);

  scleraXsave = scleraX;
  irisY = scleraY - (SCLERA_HEIGHT - IRIS_HEIGHT) / 2;

  uint16_t lidX = 0;
  uint16_t dlidX = -1;
  if (e) dlidX = 1;
  for (screenY = 0; screenY < SCREEN_HEIGHT; screenY++, scleraY++, irisY++) {
    scleraX = scleraXsave;
    irisX = scleraXsave - (SCLERA_WIDTH - IRIS_WIDTH) / 2;
    if (e) lidX = 0; else lidX = SCREEN_WIDTH - 1;
    for (screenX = 0; screenX < SCREEN_WIDTH; screenX++, scleraX++, irisX++, lidX += dlidX) {
      if ((pgm_read_byte(lower + screenY * SCREEN_WIDTH + lidX) <= lT) ||
          (pgm_read_byte(upper + screenY * SCREEN_WIDTH + lidX) <= uT)) {
        p = 0; // Eyelid closed
      } else if ((irisY < 0) || (irisY >= IRIS_HEIGHT) ||
                 (irisX < 0) || (irisX >= IRIS_WIDTH)) {
        p = pgm_read_word(sclera + scleraY * SCLERA_WIDTH + scleraX);
      } else {
        p = pgm_read_word(polar + irisY * IRIS_WIDTH + irisX);
        d = (iScale * (p & 0x7F)) / 128;
        if (d < IRIS_MAP_HEIGHT) {
          a = (IRIS_MAP_WIDTH * (p >> 7)) / 512;
          p = pgm_read_word(iris + d * IRIS_MAP_WIDTH + a);
        } else {
          p = pgm_read_word(sclera + scleraY * SCLERA_WIDTH + scleraX);
        }
      }
      pbuffer[0][pixels++] = p >> 8 | p << 8;

      if (pixels >= BUFFER_SIZE) {
        yield();
        tft.pushPixels(pbuffer[0], pixels);
        pixels = 0;
      }
    }
  }

  if (pixels) {
    tft.pushPixels(pbuffer[0], pixels);
  }
  tft.endWrite();
  digitalWrite(eye[e].tft_cs, HIGH);

  // Debug timing
  // uint32_t endTime = millis();
  // static uint32_t lastTime = 0;
  // if (endTime - lastTime > 100) { // Log delays > 100ms
  //   Serial.print("Eye "); Serial.print(e); Serial.print(" render took: ");
  //   Serial.println(endTime - lastTime);
  // }
  // lastTime = endTime;
}

void user_loop(void);

void updateEye(void) {
  static uint32_t frames = 0;
  int16_t eyeX, eyeY;
  uint32_t t = millis();

  if (!(++frames & 255)) {
    float elapsed = (millis() - startTime) / 1000.0;
    if (elapsed) {
      Serial.print("FPS=");
      Serial.println((uint16_t)(frames / elapsed));
    }
  }

  // Get eye X position from radar data
  if (xSemaphoreTake(eyeXMutex, portMAX_DELAY) == pdTRUE) {
    eyeX = eyeTargetX;
    xSemaphoreGive(eyeXMutex);
  }

  // Fixed vertical position
  eyeY = 512;

  // Human-like blinking
#ifdef AUTOBLINK
  static uint32_t timeOfLastBlink = 0L, timeToNextBlink = 0L;
  if (t - timeOfLastBlink >= timeToNextBlink) {
    timeOfLastBlink = t;
    uint32_t blinkDuration = random(100, 300); // 100-300ms
    for (uint8_t e = 0; e < NUM_EYES; e++) {
      if (eye[e].blink.state == NOBLINK) {
        eye[e].blink.state = ENBLINK;
        eye[e].blink.startTime = t;
        eye[e].blink.duration = blinkDuration;
      }
    }
    timeToNextBlink = random(2000, 6000); // Every 2-6 seconds
  }
#endif

  // Update both eyes (no iris scaling for simplicity)
  for (uint8_t eyeIndex = 0; eyeIndex < NUM_EYES; eyeIndex++) {
    if (eye[eyeIndex].blink.state) {
      if ((t - eye[eyeIndex].blink.startTime) >= eye[eyeIndex].blink.duration) {
        if (++eye[eyeIndex].blink.state > DEBLINK) {
          eye[eyeIndex].blink.state = NOBLINK;
        } else {
          eye[eyeIndex].blink.duration = eye[eyeIndex].blink.duration / 2;
          eye[eyeIndex].blink.startTime = t;
        }
      }
    }

    // Map eye position
    int16_t adjustedEyeX = map(eyeX, 0, 1023, 0, SCLERA_WIDTH - 240);
    int16_t adjustedEyeY = map(eyeY, 0, 1023, 0, SCLERA_HEIGHT - 240);
    if (NUM_EYES > 1) {
      if (eyeIndex == 1) adjustedEyeX += 4;
      else adjustedEyeX -= 4;
    }
    if (adjustedEyeX > (SCLERA_WIDTH - 240)) adjustedEyeX = (SCLERA_WIDTH - 240);

    // Eyelid thresholds
    uint8_t uThreshold = 128;
    uint8_t lThreshold = 254 - uThreshold;
    uint8_t n = uThreshold;

    if (eye[eyeIndex].blink.state) {
      uint32_t s = (t - eye[eyeIndex].blink.startTime);
      if (s >= eye[eyeIndex].blink.duration) s = 255;
      else s = 255 * s / eye[eyeIndex].blink.duration;
      s = (eye[eyeIndex].blink.state == DEBLINK) ? 1 + s : 256 - s;
      n = (uThreshold * s + 254 * (257 - s)) / 256;
      lThreshold = (lThreshold * s + 254 * (257 - s)) / 256;
    }

    drawEye(eyeIndex, (IRIS_MIN + IRIS_MAX) / 2, adjustedEyeX, adjustedEyeY, n, lThreshold);
  }

  user_loop();
}

void user_setup() {}
void user_loop() {}

void setup() {
  Serial.begin(115200);
  Serial1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial1.setRxBufferSize(64);
  Serial.println("RD-03D Radar Module Initialized");

  Serial1.write(Single_Target_Detection_CMD, sizeof(Single_Target_Detection_CMD));
  delay(200);
  Serial.println("Single-target detection mode activated.");

  RX_count = 0;
  Serial1.flush();

  user_setup();
  initEyes();

  tft.init();

  digitalWrite(eye[0].tft_cs, HIGH);
  if (NUM_EYES > 1) digitalWrite(eye[1].tft_cs, HIGH);

  for (uint8_t e = 0; e < NUM_EYES; e++) {
    digitalWrite(eye[e].tft_cs, LOW);
    tft.setRotation(eyeInfo[e].rotation);
    tft.fillScreen(TFT_BLACK);
    digitalWrite(eye[e].tft_cs, HIGH);
  }

#if defined(DISPLAY_BACKLIGHT) && (DISPLAY_BACKLIGHT >= 0)
  analogWrite(DISPLAY_BACKLIGHT, BACKLIGHT_MAX);
#endif

  startTime = millis();

  eyeXMutex = xSemaphoreCreateMutex();

  // Higher priority for eye task
  xTaskCreatePinnedToCore(radarTask, "RadarTask", 4096, NULL, 1, &radarTaskHandle, 0);
  xTaskCreatePinnedToCore(eyeTask, "EyeTask", 8192, NULL, 2, &eyeTaskHandle, 1); // Priority 2
}

void loop() {
  delay(1000);
}