// config.h
#ifndef CONFIG_H
#define CONFIG_H

// Pin selections here are based on ESP32
// Only the default_large eye runs on 240x240 px.
// For all other eyes resolutions need to be adjusted in eye_functions.

///#define SYMMETRICAL_EYELID
#include "data/default_large.h" // Using large eye for 240x240 px

#define TFT_COUNT 2
#define TFT1_CS 10
#define TFT2_CS 15
#define TFT_1_ROT 2
#define TFT_2_ROT 2
#define EYE_1_XPOSITION 0
#define EYE_2_XPOSITION 0

#define DISPLAY_BACKLIGHT -1
#define BACKLIGHT_MAX 255

#define NUM_EYES 2

#define BLINK_PIN -1
#define LH_WINK_PIN -1
#define RH_WINK_PIN -1

// Define eyeInfo_t structure
typedef struct {
  int8_t select;      // Pin numbers for each eye's screen select line
  int8_t wink;        // Wink button (or -1 if none)
  uint8_t rotation;   // Display rotation
  int16_t xposition;  // Position of eye on the screen
} eyeInfo_t;

// Define eyeInfo array (no extern)
#if (NUM_EYES == 2)
  eyeInfo_t eyeInfo[] = {
    { TFT1_CS, LH_WINK_PIN, TFT_1_ROT, EYE_1_XPOSITION },
    { TFT2_CS, RH_WINK_PIN, TFT_2_ROT, EYE_2_XPOSITION }
  };
#else
  eyeInfo_t eyeInfo[] = {
    { TFT1_CS, LH_WINK_PIN, TFT_1_ROT, EYE_1_XPOSITION }
  };
#endif

#define TRACKING
#define AUTOBLINK

#define LIGHT_CURVE 0.33
#define LIGHT_MIN 0
#define LIGHT_MAX 1023

#define IRIS_SMOOTH
#if !defined(IRIS_MIN)
  #define IRIS_MIN 90
#endif
#if !defined(IRIS_MAX)
  #define IRIS_MAX 130
#endif

#endif // CONFIG_H