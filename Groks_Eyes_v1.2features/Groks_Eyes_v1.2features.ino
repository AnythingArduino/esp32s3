// ESP32 Dual-Core Uncanny Eyes with RD-03D Radar Integration
// Core 0: Radar processing
// Core 1: Eye animation and display rendering
// Eyes move left/right with human-like features:
// - Subtle idle movement, saccadic shifts, pupil dilation, coordinated blinking,
//   eyelid tracking, emotional response (no tilt or servo)

#define RX_PIN 16
#define TX_PIN 17
#define BAUD_RATE 256000

#include <SPI.h>
#include <TFT_eSPI.h>
TFT_eSPI tft;

#define BUFFER_SIZE 2048
#define BUFFERS 1
uint16_t pbuffer[BUFFERS][BUFFER_SIZE];

#include "config.h"

// Radar Variables
uint8_t RX_BUF[64] = {0};
uint8_t RX_count = 0;
uint8_t RX_temp = 0;
int16_t target1_x = 0, target1_y = 0;
int16_t target1_speed = 0, last_target1_speed = 0;
uint16_t target1_distance_res = 0;
float target1_distance = 0;
float target1_angle = 0;
uint8_t Single_Target_Detection_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};

// Shared variables
int16_t eyeTargetX = 512;
SemaphoreHandle_t eyeXMutex;
SemaphoreHandle_t emotionMutex;
volatile bool surpriseFlag = false;

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

const uint8_t ease[] = {
  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,  3,
  3,  3,  4,  4,  4,  5,  5,  6,  6,  7,  7,  8,  9,  9, 10, 10,
  11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, 27, 28, 29, 30, 31, 33, 34, 35, 36, 37, 38, 39,
  40, 41, 42, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 56, 57, 58,
  60, 61, 62, 63, 65, 66, 67, 69, 70, 72, 73, 74, 76, 77, 78, 80,
  81, 83, 84, 85, 87, 88, 90, 91, 93, 94, 96, 97, 98, 100, 101, 103,
  104, 106, 107, 109, 110, 112, 113, 115, 116, 118, 119, 121, 122, 124, 125, 127,
  128, 130, 131, 133, 134, 136, 137, 139, 140, 142, 143, 145, 146, 148, 149, 151,
  152, 154, 155, 157, 158, 159, 161, 162, 164, 165, 167, 168, 170, 171, 172, 174,
  175, 177, 178, 179, 181, 182, 183, 185, 186, 188, 189, 190, 192, 193, 194, 195,
  197, 198, 199, 201, 202, 203, 204, 205, 207, 208, 209, 210, 211, 213, 214, 215,
  216, 217, 218, 219, 220, 221, 222, 224, 225, 226, 227, 228, 228, 229, 230, 231,
  232, 233, 234, 235, 236, 237, 237, 238, 239, 240, 240, 241, 242, 243, 243, 244,
  245, 245, 246, 246, 247, 248, 248, 249, 249, 250, 250, 251, 251, 251, 252, 252,
  252, 253, 253, 253, 254, 254, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255
};

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
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

void processRadarData() {
  if (RX_count >= 32) {
    target1_x = (RX_BUF[4] | (RX_BUF[5] << 8)) - 0x200;
    target1_y = (RX_BUF[6] | (RX_BUF[7] << 8)) - 0x8000;
    int16_t new_speed = (RX_BUF[8] | (RX_BUF[9] << 8)) - 0x10;
    target1_distance_res = (RX_BUF[10] | (RX_BUF[11] << 8));
    target1_distance = sqrt(pow(target1_x, 2) + pow(target1_y, 2));
    target1_angle = atan2(target1_y, target1_x) * 180.0 / PI;

    if (target1_distance < 10000 && target1_distance >= 0) {
      int16_t newEyeX = map(target1_x, -512, 511, 0, 1023);
      if (xSemaphoreTake(eyeXMutex, portMAX_DELAY) == pdTRUE) {
        eyeTargetX = newEyeX;
        xSemaphoreGive(eyeXMutex);
      }

      if (abs(new_speed - last_target1_speed) > 20) {
        if (xSemaphoreTake(emotionMutex, portMAX_DELAY) == pdTRUE) {
          surpriseFlag = true;
          xSemaphoreGive(emotionMutex);
        }
      }
      last_target1_speed = new_speed;
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
        p = 0;
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

  if (xSemaphoreTake(eyeXMutex, portMAX_DELAY) == pdTRUE) {
    eyeX = eyeTargetX;
    xSemaphoreGive(eyeXMutex);
  }
  eyeY = 512;

  // 1. Subtle Idle Movement
  static int16_t idleXOffset = 0, idleYOffset = 0;
  static uint32_t lastIdleUpdate = 0;
  static int16_t lastEyeX = 512;
  if (abs(eyeX - lastEyeX) < 20 && t - lastIdleUpdate > 500) {
    idleXOffset = random(-5, 6);
    idleYOffset = random(-5, 6);
    lastIdleUpdate = t;
  }
  lastEyeX = eyeX;
  eyeX += idleXOffset;
  eyeY += idleYOffset;

  // 2. Saccadic Eye Movements
  static int16_t currentEyeX = 512;
  static uint32_t saccadeStart = 0;
  static bool inSaccade = false;
  int16_t saccadeTargetX = eyeX;
  if (abs(saccadeTargetX - currentEyeX) > 50 && !inSaccade) {
    inSaccade = true;
    saccadeStart = t;
  }
  if (inSaccade) {
    uint32_t elapsed = t - saccadeStart;
    if (elapsed < 100) {
      uint8_t step = ease[255 * elapsed / 100];
      eyeX = currentEyeX + ((saccadeTargetX - currentEyeX) * step) / 255;
    } else {
      eyeX = saccadeTargetX;
      currentEyeX = eyeX;
      inSaccade = false;
    }
  } else {
    currentEyeX = eyeX;
  }

  // 3. Pupil Dilation Response
  uint32_t iScale = (IRIS_MIN + IRIS_MAX) / 2;
  if (target1_distance > 0) {
    iScale = map(target1_distance, 0, 10000, IRIS_MAX, IRIS_MIN);
    iScale = constrain(iScale, IRIS_MIN, IRIS_MAX);
  }

  // 4. Coordinated Blinking with Asymmetry
#ifdef AUTOBLINK
  static uint32_t timeOfLastBlink = 0L, timeToNextBlink = 0L;
  if (t - timeOfLastBlink >= timeToNextBlink) {
    timeOfLastBlink = t;
    uint32_t blinkDuration = random(100, 300);
    for (uint8_t e = 0; e < NUM_EYES; e++) {
      if (eye[e].blink.state == NOBLINK) {
        eye[e].blink.state = ENBLINK;
        eye[e].blink.startTime = t + (e == 1 && random(100) < 20 ? random(0, 50) : 0);
        eye[e].blink.duration = blinkDuration;
      }
    }
    timeToNextBlink = random(2000, 6000);
  }
#endif

  // Update both eyes
  for (uint8_t eyeIndex = 0; eyeIndex < NUM_EYES; eyeIndex++) {
    if (eye[eyeIndex].blink.state) {
      if ((t - eye[eyeIndex].blink.startTime) >= eye[eyeIndex].blink.duration) {
        if (++eye[eyeIndex].blink.state > DEBLINK) {
          eye[eyeIndex].blink.state = NOBLINK;
        } else {
          eye[eyeIndex].blink.duration = eye[eyeIndex].blink.duration / 2;
          eye[eyeIndex].blink.startTime = t; // Fixed: Use eyeIndex instead of e
        }
      }
    }

    int16_t adjustedEyeX = map(eyeX, 0, 1023, 0, SCLERA_WIDTH - 240);
    int16_t adjustedEyeY = map(eyeY, 0, 1023, 0, SCLERA_HEIGHT - 240);
    if (NUM_EYES > 1) {
      if (eyeIndex == 1) adjustedEyeX += 4;
      else adjustedEyeX -= 4;
    }
    if (adjustedEyeX > (SCLERA_WIDTH - 240)) adjustedEyeX = (SCLERA_WIDTH - 240);

    // 5. Eyelid Tracking with Gaze
#ifdef TRACKING
    int16_t sampleX = SCLERA_WIDTH / 2 - (adjustedEyeX / 2);
    int16_t sampleY = SCLERA_HEIGHT / 2 - (adjustedEyeY + IRIS_HEIGHT / 4);
    uint8_t n;
    if (sampleY < 0) n = 0;
    else n = (pgm_read_byte(upper + sampleY * SCREEN_WIDTH + sampleX) +
              pgm_read_byte(upper + sampleY * SCREEN_WIDTH + (SCREEN_WIDTH - 1 - sampleX))) / 2;
    static uint8_t uThreshold = 128;
    uThreshold = (uThreshold * 3 + n) / 4;
#else
    uint8_t uThreshold = 128;
#endif
    uint8_t lThreshold = 254 - uThreshold;
    uint8_t nThreshold = uThreshold;

    // 6. Emotional Response
    if (xSemaphoreTake(emotionMutex, portMAX_DELAY) == pdTRUE) {
      if (surpriseFlag) {
        uThreshold = 100;
        lThreshold = 254 - uThreshold;
        surpriseFlag = false;
      }
      xSemaphoreGive(emotionMutex);
    }

    if (eye[eyeIndex].blink.state) {
      uint32_t s = (t - eye[eyeIndex].blink.startTime);
      if (s >= eye[eyeIndex].blink.duration) s = 255;
      else s = 255 * s / eye[eyeIndex].blink.duration;
      s = (eye[eyeIndex].blink.state == DEBLINK) ? 1 + s : 256 - s;
      nThreshold = (uThreshold * s + 254 * (257 - s)) / 256;
      lThreshold = (lThreshold * s + 254 * (257 - s)) / 256;
    }

    drawEye(eyeIndex, iScale, adjustedEyeX, adjustedEyeY, nThreshold, lThreshold);
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
  emotionMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(radarTask, "RadarTask", 4096, NULL, 1, &radarTaskHandle, 0);
  xTaskCreatePinnedToCore(eyeTask, "EyeTask", 8192, NULL, 2, &eyeTaskHandle, 1);
}

void loop() {
  delay(1000);
}