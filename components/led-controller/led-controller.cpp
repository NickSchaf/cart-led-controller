#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "FastLED.h"
#include "FX.h"
#include "led-controller.hpp"
#include "PatternColor.hpp"

#include <vector>
#include <algorithm>

#define DATA_PIN_A 4 
#define DATA_PIN_B 12
#define DATA_PIN_C 13
#define DATA_PIN_D 16
#define DATA_PIN_LED_SOURCE_RELAY  32

#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define SAVE_SETTINGS_TIMEOUT      5000
#define DEFAULT_FRAMES_PER_SECOND  100

#define POWER_TESTING 0  // Set to 1 to run the power test sequence instead of normal operation
#define INCLUDE_TEST_PATTERNS 0  // 1 to include special test patterns

#if POWER_TESTING
#define POWER_TEST_BRIGHTNESS          255
#define MY_TEST_DELAY_TIME_MS          200
#define POWER_TEST_BLACK_EVERY_NTH     6
#define POWER_TEST_PRINT_LOOP_EVERY_N  100
#endif

const CRGB PALETTE_XMAS[] = { CRGB::Red, CRGB::Green, CRGB::White };
const CRGB PALETTE_PATRIOT[] = { CRGB::Red, CRGB::White, CRGB::Blue };

const char * NVS_NAME_PATTERN_INDEX = "PatternIdx";
const char * NVS_NAME_COLOR_INDEX = "ColorIdx";
const char * NVS_NAME_SPEED = "Speed";
const char * NVS_NAME_BRIGHTNESS = "Brightness";

typedef struct led_strip_t
{
    const uint16_t pixel_count;
    CRGB * pixels;
} led_strip_t;

// Roof - starts back-right, wraps around front to back-left
led_strip_t strip_A = {
    .pixel_count = 300,
    // .pixel_count = 12,
    .pixels = nullptr
};
// Underbody
led_strip_t strip_B = {
    .pixel_count = 240,
    .pixels = nullptr
};
// Sides: 52 LEDs on rocker panel daisy-chained to 57 LEDs on upright

// Right side
led_strip_t strip_C = {
    .pixel_count = 109,
    .pixels = nullptr
};
// Left side
led_strip_t strip_D = {
    .pixel_count = 109,
    .pixels = nullptr
};

// Special strips to refer to segments of the daisy-chained sides
// These are scondary access the arrays in strips C & D
// Right rocker: back to front
led_strip_t strip_rightRocker = {
    .pixel_count = 52,
    .pixels = nullptr
};
led_strip_t strip_rightUpright = {
    .pixel_count = 57,
    .pixels = nullptr
};
// Left rocker: back to front
led_strip_t strip_leftRocker = {
    .pixel_count = 52,
    .pixels = nullptr
};
led_strip_t strip_leftUpright = {
    .pixel_count = 57,
    .pixels = nullptr
};


typedef void (*PatternFunc)(void);
typedef struct led_pattern_t
{
  PatternFunc func;
  const char * name;
  uint8_t fixedSpeed;
  bool setsUnderbody;
  bool setsSides;
} led_pattern_t;

typedef struct led_color_t
{
  CRGB value;
  const char * name;
} led_color_t;


bool Equals(const CRGB& a, const CRGB& b)
{
  return memcmp(a.raw, b.raw, 3) == 0;
}

// Macro to find array length
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

// Pallettes
extern const TProgmemPalette16 redWhiteBluePalette_p;

// Forward declarations of pattern functions
void Pattern_rainbow();
// void Pattern_rainbowWithGlitter();
void Pattern_glitter();
void Pattern_confetti();
void Pattern_sinelon();
void Pattern_juggle();
void Pattern_bpm();
void Pattern_nightRider();
void Pattern_FadingMarquee();
void Pattern_Police();
void Pattern_Solid();
void Pattern_Pulse();
void Pattern_Xmas2();
void PatternRainbowStripe();
void Pattern_RedWhiteBlue();
void Pattern_Party();
void Pattern_Cloudy();
void Pattern_Pride();
#if INCLUDE_TEST_PATTERNS
// Special test patterns to check each strip
// Speed setting dictates which LED(s) on the strip light-up
void TEST_Strip(led_strip_t * strip);
void Pattern_TEST_Roof() { TEST_Strip(&strip_A); }
void Pattern_TEST_Underbody() { TEST_Strip(&strip_B); }
void Pattern_TEST_RightRocker() { TEST_Strip(&strip_rightRocker); }
void Pattern_TEST_RightUpright() { TEST_Strip(&strip_rightUpright); }
void Pattern_TEST_LeftRocker() { TEST_Strip(&strip_leftRocker); }
void Pattern_TEST_LeftUpright() { TEST_Strip(&strip_leftUpright); }
#endif
void Pattern_Special_FalconPlayer();


const led_pattern_t gPatterns[] = {
  { .func = Pattern_rainbow,              .name = "Rainbow",          .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  // { .func = Pattern_rainbowWithGlitter,   .name = "Glitter Rainbow",  .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_glitter,              .name = "Glitter",          .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_confetti,             .name = "Confetti",         .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_sinelon,              .name = "Sinelon",          .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_juggle,               .name = "Juggle",           .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_bpm,                  .name = "BPM",              .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_nightRider,           .name = "Night Rider",      .fixedSpeed = 100, .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_FadingMarquee,        .name = "Fading Marquee",   .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = true },
  { .func = Pattern_Police,               .name = "Police",           .fixedSpeed = 70,  .setsUnderbody = true,  .setsSides = true },
  { .func = Pattern_Solid,                .name = "Solid",            .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_Pulse,                .name = "Pulse",            .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_Xmas2,                .name = "Xmas Moving",      .fixedSpeed = 10,  .setsUnderbody = false, .setsSides = false },
  { .func = PatternRainbowStripe,         .name = "Rainbow Stripe",   .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_RedWhiteBlue,         .name = "Red, White, Blue", .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_Party,                .name = "Party",            .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_Cloudy,               .name = "Cloudy",           .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
  { .func = Pattern_Pride,                .name = "Pride",            .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
#if INCLUDE_TEST_PATTERNS
  { .func = Pattern_TEST_Roof,            .name = "TEST Roof",        .fixedSpeed = 0,   .setsUnderbody = true,  .setsSides = true },
  { .func = Pattern_TEST_Underbody,       .name = "TEST Underbody",   .fixedSpeed = 0,   .setsUnderbody = true,  .setsSides = true },
  { .func = Pattern_TEST_RightRocker,     .name = "TEST R Rocker",    .fixedSpeed = 0,   .setsUnderbody = true,  .setsSides = true },
  { .func = Pattern_TEST_RightUpright,    .name = "TEST R Upright",   .fixedSpeed = 0,   .setsUnderbody = true,  .setsSides = true },
  { .func = Pattern_TEST_LeftRocker,      .name = "TEST L Rocker",    .fixedSpeed = 0,   .setsUnderbody = true,  .setsSides = true },
  { .func = Pattern_TEST_LeftUpright,     .name = "TEST L Upright",   .fixedSpeed = 0,   .setsUnderbody = true,  .setsSides = true },
#endif
  { .func = Pattern_Special_FalconPlayer, .name = "[Falcon Player]",  .fixedSpeed = 0,   .setsUnderbody = false, .setsSides = false },
};

PatternColor_Rainbow * rainbowColor = new PatternColor_Rainbow();
typedef std::vector<PatternColor*>::iterator PatternColorIter;
typedef std::vector<PatternColor*>::const_iterator PatternColorConstIter;
std::vector<PatternColor*> gColors {
  new PatternColor("White", CRGB::White),
  new PatternColor("Red", CRGB::Red),
  new PatternColor("Green", CRGB::Green),
  new PatternColor("Blue", CRGB::Blue),
  new PatternColor("Violet", CRGB::Violet),
  new PatternColor_Random(),
  rainbowColor,
  new PatternColor_PaletteManual("[Xmas]", PALETTE_XMAS, ARRAY_SIZE(PALETTE_XMAS)),
  new PatternColor_PaletteManual("[Patriot]", PALETTE_PATRIOT, ARRAY_SIZE(PALETTE_PATRIOT)),

#define ColorsLength (gColors.size())
#define gHue (rainbowColor->GetCurrentHue())
};

BoundedValue bounded_speed(1, 100, 2, DEFAULT_FRAMES_PER_SECOND);
BoundedValue bounded_brightness(10, 250, 10, DEFAULT_BRIGHTNESS);


uint8_t nextPatternIndex = 6;     // Index of the pattern to be used on the next loop iteration
uint8_t nextColorIndex = 6;       // Index of the pattern to be used on the next loop iteration
uint8_t paletteStartIdx = 0;
uint8_t gCurColorIndex = 0;
SaveableSetting gPatternSaveState;
SaveableSetting gColorSaveState;
SaveableSetting gBrightnessSaveState;
SaveableSetting gSpeedSaveState;
bool genericToggle = false;       // Used by some patterns to keep track of flips, direction changes, etc.
const led_pattern_t * gPrevPattern = NULL;
const led_pattern_t * gCurPattern = NULL;
ValueChangedCb _patternChangedCb;
ValueChangedCb _colorChangedCb;
ValueChangedCb _brightnessChangedCb;
ValueChangedCb _speedChangedCb;


void init_led_strip(led_strip_t * strip)
{
    if (strip != nullptr && strip->pixel_count > 0)
    {
        strip->pixels = (CRGB*)malloc(strip->pixel_count * sizeof(CRGB));
    }
}

void LoadSavedSettings()
{
  nvs_handle_t nvs;
  esp_err_t err;
  err = nvs_open("storage", NVS_READWRITE, &nvs);
  if (err != ESP_OK)
  {
    ESP_LOGE(__func__, "nvs_open error: %s", esp_err_to_name(err));
    return;
  }

  // Load saved values
  uint8_t temp;
  err = nvs_get_u8(nvs, NVS_NAME_PATTERN_INDEX, &temp);
  if (temp < ARRAY_SIZE(gPatterns))
  {
    nextPatternIndex = temp;
    if (_patternChangedCb != NULL) _patternChangedCb(nextPatternIndex);
  }

  err = nvs_get_u8(nvs, NVS_NAME_COLOR_INDEX, &temp);
  if (temp < ColorsLength)
  {
    nextColorIndex = temp;
    if (_colorChangedCb != NULL) _colorChangedCb(nextColorIndex);
  }

  err = nvs_get_u8(nvs, NVS_NAME_SPEED, &temp);
  if (bounded_speed.Set(temp))
  {
    if (_speedChangedCb != NULL) _speedChangedCb(bounded_speed.Value());
  }

  err = nvs_get_u8(nvs, NVS_NAME_BRIGHTNESS, &temp);
  if (bounded_brightness.Set(temp))
  {
    if (_brightnessChangedCb != NULL) _brightnessChangedCb(bounded_brightness.Value());
  }

  nvs_close(nvs);
}

void SaveU8Value(const char * settingName, uint8_t value)
{
  nvs_handle_t nvs;
  esp_err_t err;

  if (settingName == NULL || strlen(settingName) == 0)
  {
    ESP_LOGE(__func__, "Null or empty settingName");
    return;
  }

  err = nvs_open("storage", NVS_READWRITE, &nvs);
  if (err != ESP_OK)
  {
    ESP_LOGE(__func__, "nvs_open error: %s", esp_err_to_name(err));
    return;
  }

  err = nvs_set_u8(nvs, settingName, value);
  if (err != ESP_OK)
  {
    ESP_LOGE(__func__, "nvs_set_u8(%s, %d) error: %s", settingName, value, esp_err_to_name(err));
    return;
  }

  nvs_close(nvs);
}

void SaveChangedSettings()
{
  if (gPatternSaveState.NeedsSaving())
  {
    uint8_t patternIdx = 0;
    for (; patternIdx < ARRAY_SIZE(gPatterns); patternIdx++)
      if (gCurPattern == &gPatterns[patternIdx]) break;

    ESP_LOGW(__func__, "Saving pattern index: %u", patternIdx);
    SaveU8Value(NVS_NAME_PATTERN_INDEX, patternIdx);
    gPatternSaveState.ClearChanged();
  }

  if (gColorSaveState.NeedsSaving())
  {
    ESP_LOGW(__func__, "Saving color index: %u", gCurColorIndex);
    SaveU8Value(NVS_NAME_COLOR_INDEX, gCurColorIndex);
    gColorSaveState.ClearChanged();
  }

  if (gSpeedSaveState.NeedsSaving())
  {
    ESP_LOGW(__func__, "Saving speed: %u", bounded_speed.Value());
    SaveU8Value(NVS_NAME_SPEED, bounded_speed.Value());
    gSpeedSaveState.ClearChanged();
  }

  if (gBrightnessSaveState.NeedsSaving())
  {
    ESP_LOGW(__func__, "Saving brightness: %u", bounded_brightness.Value());
    SaveU8Value(NVS_NAME_BRIGHTNESS, bounded_brightness.Value());
    gBrightnessSaveState.ClearChanged();
  }
}

#if POWER_TESTING
void ledTask(void *pvParameters)
{
  const CRGB * strip_end = strip_A.pixels + strip_A.pixel_count;
  uint16_t black_pixel_this_frame = 0;
  CRGB * cur_px = NULL;
  uint16_t idx = 0;
  uint32_t loop_count = 0;
  bool oddPixel = false;

  // On startup, perform a few-second light test so we can detect reboots of the controller
  FastLED.showColor(CRGB::Red, POWER_TEST_BRIGHTNESS);
  delay(1000);
  FastLED.showColor(CRGB::Green, POWER_TEST_BRIGHTNESS);
  delay(1000);
  FastLED.showColor(CRGB::Blue, POWER_TEST_BRIGHTNESS);
  delay(1000);


  while(1)
  {
    // Only calculate largest strip.  Subset of largest strips data will be pushed out other outputs
    idx = 0;
    for (cur_px = strip_A.pixels; cur_px != strip_end; idx++) {
      oddPixel = ((idx % POWER_TEST_BLACK_EVERY_NTH) == black_pixel_this_frame);
      if (idx < strip_B.pixel_count)
        strip_B.pixels[idx] = oddPixel ? CRGB::Red : CRGB::White;
      if (idx < strip_C.pixel_count)
        strip_C.pixels[idx] = oddPixel ? CRGB::Green : CRGB::White;
      if (idx < strip_C.pixel_count)
        strip_D.pixels[idx] = oddPixel ? CRGB::Blue : CRGB::White;
      // White or black, so all three channels are same value
      *cur_px++ = oddPixel ? CRGB::Black : CRGB::White;
    }

    black_pixel_this_frame = (black_pixel_this_frame + 1) % POWER_TEST_BLACK_EVERY_NTH;

    FastLED.setBrightness(POWER_TEST_BRIGHTNESS);
    FastLED.show();

    if (++loop_count % POWER_TEST_PRINT_LOOP_EVERY_N == 0)
    {
      printf("LED frame count: %u\n", loop_count);
    }

    delay(MY_TEST_DELAY_TIME_MS);
  }

}

#else

void copyRoofToSides()
{
  // Right side LED strip - copy the beginning of the main strip
  memcpy((void*)strip_C.pixels, (const void*)strip_A.pixels, sizeof(CRGB) * strip_C.pixel_count);

  // Left side LED strip - copy the end of the main strip
  memcpy((void*)strip_D.pixels, &strip_A.pixels[strip_A.pixel_count - strip_D.pixel_count], sizeof(CRGB) * strip_D.pixel_count);

  // Flip left rocker because the roof goes front-to-back on the left side while the rocker goes back-to-front
  for (int i = 0, j = strip_leftRocker.pixel_count - 1; i < j; i++, j--)
  {
    CRGB temp = strip_leftRocker.pixels[i];
    strip_leftRocker.pixels[i] = strip_leftRocker.pixels[j];
    strip_leftRocker.pixels[j] = temp;
  }
}

void copyRoofToUnderbody()
{
  // Underbody LED strip - copy the center of the main strip
  memcpy((void*)strip_B.pixels, &strip_A.pixels[(strip_A.pixel_count - strip_B.pixel_count) / 2], sizeof(CRGB) * strip_B.pixel_count);
}

void ledTask(void *pvParameters)
{
    while (1)
    {
        // Set pattern and color for this loop
        gCurPattern = &gPatterns[nextPatternIndex];
        gCurColorIndex = nextColorIndex;

        // Check whether the pattern changed to/from the special case
        if (gCurPattern != gPrevPattern)
        {
            if (gPrevPattern != NULL)
            {
              if (gPrevPattern->func == Pattern_Special_FalconPlayer)
              {
              // Switched away from FPP mode
              digitalWrite(DATA_PIN_LED_SOURCE_RELAY, LOW);
              }
              else if (gCurPattern->func == Pattern_Special_FalconPlayer)
              {
              // Switch to FPP mode
              digitalWrite(DATA_PIN_LED_SOURCE_RELAY, HIGH  );
              }
            }
        }

        // Call the current pattern function once, updating the 'strip_A.pixels' array
        gCurPattern->func();

        if (!gCurPattern->setsSides) copyRoofToSides();
        if (!gCurPattern->setsUnderbody) copyRoofToUnderbody();

        FastLED.setBrightness(bounded_brightness.Value());
        FastLED.show();
        FastLED.delay(101 - ( (gCurPattern->fixedSpeed == 0) ? bounded_speed.Value() : gCurPattern->fixedSpeed ));

        std::for_each(gColors.begin(), gColors.end(), [](PatternColor* color) { color->Loop(); });

        EVERY_N_MILLISECONDS( 100 )
        {
            SaveChangedSettings();
        }

        gPrevPattern = gCurPattern;
    }
}
#endif // POWER_TESTING

void LedController::Start()
{
  printf("Adding LEDs\n");
  // the WS2811 family uses the RMT driver
  init_led_strip(&strip_A);
  if (strip_A.pixels == NULL) ESP_LOGE(__func__, "Failed to initialize pixel strip A");
  else
    FastLED.addLeds<LED_TYPE, DATA_PIN_A, COLOR_ORDER>(strip_A.pixels, strip_A.pixel_count); // TODO: should I add .setCorrection(TypicalLEDStrip)  ??
  init_led_strip(&strip_B);
  if (strip_B.pixels == NULL) ESP_LOGE(__func__, "Failed to initialize pixel strip B");
  else
    FastLED.addLeds<LED_TYPE, DATA_PIN_B, COLOR_ORDER>(strip_B.pixels, strip_B.pixel_count);
  init_led_strip(&strip_C);
  if (strip_C.pixels == NULL) ESP_LOGE(__func__, "Failed to initialize pixel strip C");
  else
    FastLED.addLeds<LED_TYPE, DATA_PIN_C, COLOR_ORDER>(strip_C.pixels, strip_C.pixel_count);
  init_led_strip(&strip_D);
  if (strip_D.pixels == NULL) ESP_LOGE(__func__, "Failed to initialize pixel strip D");
  else
    FastLED.addLeds<LED_TYPE, DATA_PIN_D, COLOR_ORDER>(strip_D.pixels, strip_D.pixel_count);

  // Rocker panel strip is daisy-chained to upright, so it follows the 
  strip_rightRocker.pixels = strip_C.pixels;
  strip_rightUpright.pixels = &strip_C.pixels[strip_rightRocker.pixel_count];
  strip_leftRocker.pixels = strip_D.pixels;
  strip_leftUpright.pixels = &strip_D.pixels[strip_leftRocker.pixel_count];

  // Cart has a 12v->5v converter with 40A capacity
  FastLED.setMaxPowerInVoltsAndMilliamps(5,40000);

  // Setup the relay control GPIO pin
  gpio_reset_pin(gpio_num_t(DATA_PIN_LED_SOURCE_RELAY));
  gpio_set_direction(gpio_num_t(DATA_PIN_LED_SOURCE_RELAY), GPIO_MODE_OUTPUT);

  LoadSavedSettings();

  xTaskCreatePinnedToCore(&ledTask, "blinkLeds", 4000, nullptr, 6, nullptr, 1);
}

CRGB GetCurrentColor()
{
    return gColors[gCurColorIndex]->GetColor();
}

// Increment the current color, if applicable
void IncrementColor()
{
  // std::for_each(gColors.begin(), gColors.end(), [](PatternColor* color) { color->Increment(); });
  gColors[gCurColorIndex]->Increment();
}

void LedController::GetColors(char * colorNames, int32_t * bufSize)
{
    int32_t accumulatedSz = 0;
    int32_t maxSz = 0;
    if (bufSize != NULL) maxSz = *bufSize;

    PatternColorConstIter curColor = gColors.begin();
    PatternColorConstIter endColor = gColors.end();

    while (curColor != endColor)
    {
        const char* name = (*curColor)->GetName();
        int32_t len = strlen(name);
        if (colorNames != NULL && (accumulatedSz + 1 + len) < maxSz)
        {
            strcpy(colorNames + accumulatedSz, name);
            colorNames[accumulatedSz + len] = '\n';
        }
        accumulatedSz += len + 1;
        curColor++;
    }

    // Replace last newline with null terminator to denote the end of the list
    if (accumulatedSz > 0 && colorNames != NULL) colorNames[accumulatedSz - 1] = 0;

    if (bufSize != NULL) *bufSize = accumulatedSz;
}

void LedController::GetPatterns(char * patternNames, int32_t * bufSize)
{
    int32_t accumulatedSz = 0;
    int32_t maxSz = 0;
    if (bufSize != NULL) maxSz = *bufSize;

    const led_pattern_t * curPattern = &gPatterns[0];
    const led_pattern_t * endPattern = curPattern + ARRAY_SIZE(gPatterns);

    while (curPattern != endPattern)
    {
        int32_t len = strlen(curPattern->name);
        if (patternNames != NULL && (accumulatedSz + 1 + len) < maxSz)
        {
            strcpy(patternNames + accumulatedSz, curPattern->name);
            patternNames[accumulatedSz + len] = '\n';
        }
        accumulatedSz += len + 1;
        curPattern++;
    }

    // Replace last newline with null terminator to denote the end of the list
    if (accumulatedSz > 0 && patternNames != NULL) patternNames[accumulatedSz - 1] = 0;

    if (bufSize != NULL) *bufSize = accumulatedSz;
}

uint8_t LedController::GetPatternIndex()
{
  return nextPatternIndex;
}

uint8_t LedController::GetColorIndex()
{
  return nextColorIndex;
}

uint8_t LedController::GetBrightness()
{
  return bounded_brightness.Value();
}

uint8_t LedController::GetSpeed()
{
  return bounded_speed.Value();
}

void SetPatternChangedCallback(ValueChangedCb fn)
{
  _patternChangedCb = fn;

  // Call it right away to set an initial value
  if (_patternChangedCb != NULL) _patternChangedCb(nextPatternIndex);
}

void SetColorChangedCallback(ValueChangedCb fn)
{
  _colorChangedCb = fn;

  // Call it right away to set an initial value
  if (_colorChangedCb != NULL) _colorChangedCb(nextColorIndex);
}

void SetBrightnessChangedCallback(ValueChangedCb fn)
{
  _brightnessChangedCb = fn;

  // Call it right away to set an initial value
  if (_brightnessChangedCb != NULL) _brightnessChangedCb(LedController::GetBrightness());
}

void SetSpeedChangedCallback(ValueChangedCb fn)
{
  _speedChangedCb = fn;

  // Call it right away to set an initial value
  if (_speedChangedCb != NULL) _speedChangedCb(LedController::GetSpeed());
}

void GetColors(char * colorNames, int32_t * bufSize)
{
  LedController::GetColors(colorNames, bufSize);
}

void GetPatterns(char * patternNames, int32_t * bufSize)
{
  LedController::GetPatterns(patternNames, bufSize);
}


void LedController::SetColor(uint8_t colorIndex)
{
  ESP_LOGI(__func__, "Setting color index to %u", colorIndex);
  if (colorIndex < ColorsLength)
  {
    nextColorIndex = colorIndex;
    gColorSaveState.SetChanged();
    ESP_LOGI(__func__, "Color set to %s", gColors[nextColorIndex]->GetName());
    if (_colorChangedCb != NULL) _colorChangedCb(nextColorIndex);
  }
}

void LedController::SetPattern(uint8_t patternIndex)
{
  ESP_LOGI(__func__, "Setting pattern index to %u", patternIndex);
  if (patternIndex < ARRAY_SIZE(gPatterns) && patternIndex != nextPatternIndex)
  {
    nextPatternIndex = patternIndex;
    gPatternSaveState.SetChanged();
    ESP_LOGI(__func__, "Pattern set to %s", gPatterns[nextPatternIndex].name);
    if (_patternChangedCb != NULL) _patternChangedCb(nextPatternIndex);
  }
}

void LedController::SetSpeed(uint8_t speed)
{
  if (!bounded_speed.Set(speed))
  {
    ESP_LOGW(__func__, "Requested speed (%u) out of range (%u .. %u)", speed, bounded_speed.Min(), bounded_speed.Max());
    return;
  }

  ESP_LOGI(__func__, "Setting speed to %u", speed);
  gSpeedSaveState.SetChanged();
  if (_speedChangedCb != NULL) _speedChangedCb(speed);
}

void LedController::SetBrightness(uint8_t brightness)
{
  if (!bounded_brightness.Set(brightness))
  {
    ESP_LOGW(__func__, "Requested brightness (%u) out of range (%u .. %u)", brightness, bounded_brightness.Min(), bounded_brightness.Max());
    return;
  }

  ESP_LOGI(__func__, "Setting brightness to %u", brightness);
  gBrightnessSaveState.SetChanged();
  if (_brightnessChangedCb != NULL) _brightnessChangedCb(brightness);
}

void addGlitter( fract8 chanceOfGlitter) 
{
  if( random8() < chanceOfGlitter) {
    strip_A.pixels[ random16(strip_A.pixel_count) ] += CRGB::White;
  }
}

void Pattern_rainbow() 
{
  // FastLED's built-in rainbow generator
  fill_rainbow( strip_A.pixels, strip_A.pixel_count, gHue, 7);
}

// void Pattern_rainbowWithGlitter() 
// {
//   // built-in FastLED rainbow, plus some random sparkly glitter
//   Pattern_rainbow();
//   addGlitter(80);
// }

void Pattern_glitter()
{
  fill_solid( strip_A.pixels, strip_A.pixel_count, CRGB::Black);
  addGlitter(80);
}

void Pattern_confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( strip_A.pixels, strip_A.pixel_count, 10);
  int pos = random16(strip_A.pixel_count);
  strip_A.pixels[pos] += CHSV( gHue + random8(64), 200, 255);
}

void Pattern_sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( strip_A.pixels, strip_A.pixel_count, 20);
  int pos = beatsin16( 13, 0, strip_A.pixel_count-1 );
  strip_A.pixels[pos] += CHSV( gHue, 255, 192);
}

void Pattern_bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < strip_A.pixel_count; i++) { //9948
    strip_A.pixels[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void Pattern_juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( strip_A.pixels, strip_A.pixel_count, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    strip_A.pixels[beatsin16( i+7, 0, strip_A.pixel_count-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

int16_t nightRiderPos = 0;
int16_t nightRiderIncr = 1;
void Pattern_nightRider() {
  const int16_t numLeds = strip_A.pixel_count / 2;

  fadeToBlackBy(strip_A.pixels, strip_A.pixel_count, 100);
  strip_A.pixels[nightRiderPos] = GetCurrentColor();

  // Mirror on the other side
  strip_A.pixels[strip_A.pixel_count - nightRiderPos - 1] = strip_A.pixels[nightRiderPos];
    
  nightRiderPos += nightRiderIncr;
  if (nightRiderPos >= numLeds) {
    nightRiderPos = numLeds - 1;
    nightRiderIncr = -1;
    IncrementColor();
  }
  if (nightRiderPos < 0) {
    nightRiderPos = 0;
    nightRiderIncr = 1;
    IncrementColor();
  }
}

void Pattern_Solid() {
  fill_solid( strip_A.pixels, strip_A.pixel_count, GetCurrentColor());

  EVERY_N_MILLISECONDS(1000)
  {
    IncrementColor();
  }
}


#define POLICE_STROBE_COUNT 4
// #define POLICE_CORNER_SIZE 22
void Pattern_Police() {
  static int16_t policeStrobeCounter = 0;

  fill_solid( strip_B.pixels, strip_B.pixel_count, CRGB::Black);

  if (policeStrobeCounter % 2 == 0) {
    fill_solid( strip_A.pixels, strip_A.pixel_count, CRGB::Black );
    fill_solid( strip_C.pixels, strip_C.pixel_count, CRGB::Black );
    fill_solid( strip_D.pixels, strip_D.pixel_count, CRGB::Black );
  }
  else {
//    fill_solid( &strip_A.pixels[22 - POLICE_CORNER_SIZE], POLICE_CORNER_SIZE * 2, genericToggle ? CRGB::Red : CRGB::Blue );
//    fill_solid( &strip_A.pixels[117 - POLICE_CORNER_SIZE], POLICE_CORNER_SIZE * 2, genericToggle ? CRGB::Blue : CRGB::Red);
//    fill_solid( &strip_A.pixels[strip_A.pixel_count - 22 - POLICE_CORNER_SIZE], POLICE_CORNER_SIZE * 2, genericToggle ? CRGB::Blue : CRGB::Red );
//    fill_solid( &strip_A.pixels[strip_A.pixel_count - 117 - POLICE_CORNER_SIZE], POLICE_CORNER_SIZE * 2, genericToggle ? CRGB::Red : CRGB::Blue );

    // Alternate sides... roof+rocker on one side at same time as upright on other side
    if (genericToggle)
    {
      fill_solid( &strip_A.pixels[0], strip_A.pixel_count / 2, CRGB::Blue );
      fill_solid( &strip_rightRocker.pixels[0], strip_rightRocker.pixel_count, CRGB::Blue );
      fill_solid( &strip_leftUpright.pixels[0], strip_leftUpright.pixel_count, CRGB::Red );
    }
    else
    {
      fill_solid( &strip_A.pixels[strip_A.pixel_count / 2], strip_A.pixel_count / 2, CRGB::Red );
      fill_solid( &strip_leftRocker.pixels[0], strip_leftRocker.pixel_count, CRGB::Red );
      fill_solid( &strip_rightUpright.pixels[0], strip_rightUpright.pixel_count, CRGB::Blue );
    }
  }

  policeStrobeCounter++;
  if (policeStrobeCounter >= (2 * POLICE_STROBE_COUNT)) { 
    policeStrobeCounter = 0;
    genericToggle = !genericToggle;
  }
}

// // Short bars of light fade away as they move along the strip
// // Start at the center of roof and move to the back
// // Copy or recalculate for rockers
// // Uprights stay off
#define FADING_MARQUEE_BAR_SIZE 4
#define FADING_MARQUEE_FADE_STEP 5
void Pattern_FadingMarquee()
{
  const uint16_t numLedsHalfRoof = strip_A.pixel_count / 2;
  CRGB curColor = GetCurrentColor();

  // On first run, clear strips
  if (gCurPattern != gPrevPattern)
  {
    genericToggle = true;
    nightRiderPos = 0;
    fill_solid( &strip_A.pixels[0], strip_A.pixel_count, CRGB::Black );
    fill_solid( &strip_B.pixels[0], strip_A.pixel_count, CRGB::Black );
    fill_solid( &strip_C.pixels[0], strip_C.pixel_count, CRGB::Black );
    fill_solid( &strip_D.pixels[0], strip_D.pixel_count, CRGB::Black );
  }

  // Fade previous frame - only do right half; will copy to the other side below
  fadeToBlackBy(strip_A.pixels, numLedsHalfRoof, FADING_MARQUEE_FADE_STEP);
  // Rocker panel strips are short, so fade them faster
  fadeToBlackBy(strip_rightRocker.pixels, strip_rightRocker.pixel_count, FADING_MARQUEE_FADE_STEP * 2);

  // Shift all pixels toward back of cart - both sides of roof at once, right side is source
  for (uint16_t srcPx = 1; srcPx < numLedsHalfRoof; srcPx++)
  {
    strip_A.pixels[srcPx - 1] = strip_A.pixels[strip_A.pixel_count - srcPx] = strip_A.pixels[srcPx];
  }
  // Shift all pixels toward back of cart - both rockers at once
  for (uint16_t srcPx = 1; srcPx < strip_rightRocker.pixel_count; srcPx++)
  {
    strip_rightRocker.pixels[srcPx - 1] = strip_leftRocker.pixels[srcPx - 1] = strip_rightRocker.pixels[srcPx];
  }

  // Generate next pixel at start: FADING_MARQUEE_BAR_SIZE on, FADING_MARQUEE_BAR_SIZE off
  strip_A.pixels[numLedsHalfRoof - 1] = strip_A.pixels[numLedsHalfRoof] = strip_rightRocker.pixels[strip_rightRocker.pixel_count - 1] = strip_leftRocker.pixels[strip_leftRocker.pixel_count - 1] = genericToggle ? curColor : CRGB::Black;

  if (++nightRiderPos == FADING_MARQUEE_BAR_SIZE)
  {
    IncrementColor();
    genericToggle = !genericToggle;
    nightRiderPos = 0;
  }
}

#define PULSE_STEP_SIZE 15
#define PULSE_MIN_SCALE 10
void Pattern_Pulse() {
  // On the first time through the loop after chaning patterns, we need to set the direction
  if (gCurPattern != gPrevPattern)
  {
      genericToggle = false;
  }

  static uint8_t scaleVal = 255;
  strip_A.pixels[0] = GetCurrentColor();
  strip_A.pixels[0].nscale8_video(scaleVal);

  for (int i = 1; i < strip_A.pixel_count; i++)
    strip_A.pixels[i] = strip_A.pixels[0];

  if (genericToggle)
  {
    scaleVal += PULSE_STEP_SIZE;
    if (255 - scaleVal < PULSE_STEP_SIZE) genericToggle = false;  // Reverse the scaling direction for next iteration
  }
  else
  {
    scaleVal -= PULSE_STEP_SIZE;
    if (PULSE_MIN_SCALE + PULSE_STEP_SIZE > scaleVal)
    {
      genericToggle = true;  // Reverse the scaling direction for next iteration
      IncrementColor();
    }
  }
}

const CRGB Xmas2Palette[] = {
 CRGB::Red,
 CRGB::Red,
 CRGB::Red,
 CRGB::Black,
 CRGB::White,
 CRGB::Black,
 CRGB::Green,
 CRGB::Green,
 CRGB::Green,
 CRGB::Black,
 CRGB::White,
 CRGB::Black
};

void Pattern_Xmas2() {
 // Setup the pattern
 uint8_t paletteIdx = paletteStartIdx++;
 for (int i = 0; i < strip_A.pixel_count; i++)
 {
    strip_A.pixels[i] = Xmas2Palette[paletteIdx++];
    if (paletteIdx >= ARRAY_SIZE(Xmas2Palette)) paletteIdx = 0;
 }
  if (paletteStartIdx >= ARRAY_SIZE(Xmas2Palette)) paletteStartIdx = 0;
}

void PatternRainbowStripe() {
  uint8_t paletteIdx = (paletteStartIdx += 3);
  for( int i = 0; i < strip_A.pixel_count; i++) {
      strip_A.pixels[i] = ColorFromPalette( RainbowStripeColors_p, paletteIdx, bounded_brightness.Value(), LINEARBLEND);
      paletteIdx += 3;
  }
}

void Pattern_RedWhiteBlue() {
  uint8_t paletteIdx = (paletteStartIdx += 3);
  for( int i = 0; i < strip_A.pixel_count; i++) {
      strip_A.pixels[i] = ColorFromPalette( redWhiteBluePalette_p, paletteIdx, bounded_brightness.Value(), NOBLEND);
      paletteIdx += 3;
  }
}

void Pattern_Party() {
 uint8_t paletteIdx = (paletteStartIdx += 3);
 for( int i = 0; i < strip_A.pixel_count; i++) {
     strip_A.pixels[i] = ColorFromPalette( PartyColors_p, paletteIdx, bounded_brightness.Value(), NOBLEND);
     paletteIdx += 3;
 }
}

void Pattern_Cloudy() {
 uint8_t paletteIdx = (paletteStartIdx += 3);
 for( int i = 0; i < strip_A.pixel_count; i++) {
     strip_A.pixels[i] = ColorFromPalette( CloudColors_p, paletteIdx, bounded_brightness.Value(), NOBLEND);
     paletteIdx += 3;
 }
}

// This function draws rainbows with an ever-changing,
// widely-varying set of parameters.
void Pattern_Pride() 
{
  static uint16_t sPseudotime = 0;
  static uint16_t sLastMillis = 0;
  static uint16_t sHue16 = 0;
 
  uint8_t sat8 = beatsin88( 87, 220, 250);
  uint8_t brightdepth = beatsin88( 341, 96, 224);
  uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  uint8_t msmultiplier = beatsin88(147, 23, 60);

  uint16_t hue16 = sHue16;//gHue * 256;
  uint16_t hueinc16 = beatsin88(113, 1, 3000);
  
  uint16_t ms = millis();
  uint16_t deltams = ms - sLastMillis ;
  sLastMillis  = ms;
  sPseudotime += deltams * msmultiplier;
  sHue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = sPseudotime;
  
  for( uint16_t i = 0 ; i < strip_A.pixel_count; i++) {
    hue16 += hueinc16;
    uint8_t hue8 = hue16 / 256;

    brightnesstheta16  += brightnessthetainc16;
    uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

    uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
    uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
    bri8 += (255 - brightdepth);
    
    CRGB newcolor = CHSV( hue8, sat8, bri8);
    
    uint16_t pixelnumber = i;
    pixelnumber = (strip_A.pixel_count-1) - pixelnumber;
    
    nblend( strip_A.pixels[pixelnumber], newcolor, 64);
  }
}

#if INCLUDE_TEST_PATTERNS
// Use the speed setting to drive which pixel(s) light up
void TEST_Strip(led_strip_t * strip)
{
  // Start by blanking all strips
  fill_solid( strip_A.pixels, strip_A.pixel_count, CRGB::Black );
  fill_solid( strip_B.pixels, strip_B.pixel_count, CRGB::Black );
  fill_solid( strip_C.pixels, strip_C.pixel_count, CRGB::Black );
  fill_solid( strip_D.pixels, strip_D.pixel_count, CRGB::Black );
  
  uint16_t i = (uint16_t)bounded_speed.Value();
  while (i < strip->pixel_count)
  {
    strip->pixels[i] = CRGB::White;
    i += bounded_speed.Max();
  }
}
#endif

void Pattern_Special_FalconPlayer()
{
  // TODO: Use E1.31 library to get LED settings from Ethernet

  // For now, just fill black and the main loop function will switch the signal relay to the dedicated E1.31 controller
  fill_solid( strip_A.pixels, strip_A.pixel_count, CRGB::Black );
}

const TProgmemPalette16 redWhiteBluePalette_p =
{
    CRGB::Red,
    CRGB::Gray, // 'white' is too bright compared to red and blue
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Red,
    CRGB::Gray,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Blue,
    CRGB::Black,
    CRGB::Black
};

/** Arguments used by 'pattern' command */
static struct
{
    struct arg_int *pattern_idx;
    struct arg_end *end;
} pattern_args;

static int pattern_cmd_func(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &pattern_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, pattern_args.end, argv[0]);
        return 1;
    }

    if (pattern_args.pattern_idx->count == 0)
    {
      // No pattern provided, so just display the available patterns
      printf("Patterns:\r\n");
      int8_t idx = 0;
      for (; idx < ARRAY_SIZE(gPatterns); idx++)
      {
        printf("  %c %d - %s\r\n", (nextPatternIndex == idx) ? '*' : ' ', idx, gPatterns[idx].name);
      }
    }
    else
    {
      int32_t newIdx = pattern_args.pattern_idx->ival[0];

      if (newIdx >= 0 && newIdx < ARRAY_SIZE(gPatterns))
      {
        LedController::SetPattern((uint8_t)newIdx);
      }
      else
      {
        ESP_LOGW(__func__, "Invalid pattern: %d", newIdx);
      }
    }

    return 0;
}

static struct
{
    struct arg_int *color_idx;
    struct arg_end *end;
} color_args;

static int color_cmd_func(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &color_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, color_args.end, argv[0]);
        return 1;
    }

    if (color_args.color_idx->count == 0)
    {
      // No color provided, so just display the available colors
      printf("Colorss:\r\n");
      int8_t idx = 0;
      for (; idx < ColorsLength; idx++)
      {
        printf("  %c %d - %s\r\n", (nextColorIndex == idx) ? '*' : ' ', idx, gColors[idx]->GetName());
      }
    }
    else
    {
      int32_t newIdx = color_args.color_idx->ival[0];

      if (newIdx >= 0 && newIdx < ColorsLength)
      {
        LedController::SetColor((uint8_t)newIdx);
      }
      else
      {
        ESP_LOGW(__func__, "Invalid color: %d", newIdx);
      }
    }

    return 0;
}

void register_led_controller_cmds(void)
{
   // Pattern command
  pattern_args.pattern_idx = arg_int0(NULL, NULL, "<index>", "Pattern index to set" );
  pattern_args.end = arg_end(2);

  const esp_console_cmd_t pattern_cmd =
  {
    .command = "pattern",
    .help = "Display/set the pattern",
    .hint = NULL,
    .func = &pattern_cmd_func,
    .argtable = &pattern_args
  };

  ESP_ERROR_CHECK( esp_console_cmd_register(&pattern_cmd) );

   // Color command
  color_args.color_idx = arg_int0(NULL, NULL, "<index>", "Color index to set" );
  color_args.end = arg_end(2);

  const esp_console_cmd_t color_cmd =
  {
    .command = "color",
    .help = "Display/set the color",
    .hint = NULL,
    .func = &color_cmd_func,
    .argtable = &color_args
  };

  ESP_ERROR_CHECK( esp_console_cmd_register(&color_cmd) );
}

SaveableSetting::SaveableSetting()
{
  _changed = false;
  _changedTime = 0;
}

void SaveableSetting::SetChanged()
{
  _changed = true;
  _changedTime = millis();
}

void SaveableSetting::ClearChanged()
{
  _changed = false;
}

bool SaveableSetting::NeedsSaving()
{
  return (_changed && ((millis() - _changedTime) >= SAVE_SETTINGS_TIMEOUT));
}

BoundedValue::BoundedValue(uint8_t min, uint8_t max, uint8_t step, uint8_t value) :
  _min(min), _max(max), _step(step)
{
  _value = MAX(_min, value);
}

bool BoundedValue::Set(uint8_t value)
{
  if (value < _min || value > _max) return false;

  _value = value;
  return true;
}

uint8_t get_current_pattern_index() { return LedController::GetPatternIndex(); }
uint8_t get_current_color_index() { return LedController::GetColorIndex(); }
uint8_t get_brightness() { return LedController::GetBrightness(); }
uint8_t get_speed() { return LedController::GetSpeed(); }

void set_pattern_index(uint8_t index) { LedController::SetPattern(index); }
void set_color_index(uint8_t index) { LedController::SetColor(index); }
void set_brightness(uint8_t value) { LedController::SetBrightness(value); }
void set_speed(uint8_t value) { LedController::SetSpeed(value); }
