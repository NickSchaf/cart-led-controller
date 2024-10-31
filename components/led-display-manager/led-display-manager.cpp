#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
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
#include "led-strip.h"
#include "led-display-manager.hpp"
#include "BoundedValue.hpp"
#include "SaveableSetting.hpp"

#include "LedSpecialPatternFPP.hpp"

#include <algorithm>

#define DEFAULT_FRAMES_PER_SECOND  100

const char * NVS_NAME_PATTERN_INDEX = "PatternIdx";
const char * NVS_NAME_COLOR_INDEX = "ColorIdx";
const char * NVS_NAME_SPEED = "Speed";
const char * NVS_NAME_BRIGHTNESS = "Brightness";

LedSpecialPatternFPP * fppPattern = nullptr;

// Internal rainbow color
PatternColor_Rainbow internal_rainbowColor;
PatternColor_Rainbow * rainbowColor = &internal_rainbowColor;

BoundedValue bounded_speed(1, 100, 2, DEFAULT_FRAMES_PER_SECOND);
BoundedValue bounded_brightness(10, 250, 10, DEFAULT_BRIGHTNESS);


uint8_t nextPatternIndex = 0;     // Index of the pattern to be used on the next loop iteration
uint8_t nextColorIndex = 6;       // Index of the pattern to be used on the next loop iteration
uint8_t gCurColorIndex = 0;
SaveableSetting gPatternSaveState;
SaveableSetting gColorSaveState;
SaveableSetting gBrightnessSaveState;
SaveableSetting gSpeedSaveState;
LedPattern * gPrevPattern = NULL;
LedPattern * gCurPattern = NULL;
ValueChangedCb _patternChangedCb;
ValueChangedCb _colorChangedCb;
ValueChangedCb _brightnessChangedCb;
ValueChangedCb _speedChangedCb;

// Using vectors because we use random access a lot, but only add items at startup
std::vector<PatternColor*> _colors;
std::vector<LedPattern*> _patterns;
led_strip_list _allLedStrips;

void LedDisplayManager::AddPattern(LedPattern* pattern)
{
  if (pattern != nullptr)
  {
    _patterns.push_back(pattern);

    // If the caller specifies an FPP pattern, we need to make note of it because there is special handling for it
    LedSpecialPatternFPP test({nullptr});
    // All instances of LedSpecialPatternFPP return the same static string for the name
    if (test.GetName() != pattern->GetName())
    {
      fppPattern = (LedSpecialPatternFPP*)pattern;
    }
  }
}

void LedDisplayManager::AddColor(PatternColor* color)
{
  if (color != nullptr)
  {
    _colors.push_back(color);

    // All instances of PatternColor_Rainbow return the same static string for the name
    if (internal_rainbowColor.GetName() == color->GetName())
    {
      rainbowColor = (PatternColor_Rainbow*)color;
    }
  }
}

/// @brief Add an LED strip
/// @param strip Strip to add
/// Used simply to clear the strips between patterns
void LedDisplayManager::AddLedStrip(led_strip_t * strip)
{
  if (strip != nullptr)
  {
    _allLedStrips.push_back(strip);
  }
}

void LedDisplayManager::LoadSavedSettings()
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
  if (temp < _patterns.size())
  {
    nextPatternIndex = temp;
    if (_patternChangedCb != NULL) _patternChangedCb(nextPatternIndex);
  }

  err = nvs_get_u8(nvs, NVS_NAME_COLOR_INDEX, &temp);
  if (temp < _colors.size())
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

void LedDisplayManager::SaveU8Value(const char * settingName, uint8_t value)
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

void LedDisplayManager::SaveChangedSettings()
{
  if (gPatternSaveState.NeedsSaving())
  {
    uint8_t patternIdx = 0;
    for (; patternIdx < _patterns.size(); patternIdx++)
      if (gCurPattern == _patterns[patternIdx]) break;

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

void LedDisplayManager::ledTask(void *pvParameters)
{
  while (1)
  {
    // Set pattern and color for this loop
    gCurPattern = _patterns[nextPatternIndex];
    gCurColorIndex = nextColorIndex;

    // If pattern is changing, call the previous pattern's PatternStop() and new
    // pattern's PatternStart().
    if (gPrevPattern != gCurPattern)
    {
      if (gPrevPattern != NULL) gPrevPattern->PatternStop();

      // Clear LEDs to ready for new pattern
      led_strip_list::iterator iter = _allLedStrips.begin();
      while (iter != _allLedStrips.end())
      {
        fill_solid( (*iter)->pixels, (*iter)->pixel_count, CRGB::Black);
        iter++;
      }

      if (gCurPattern != NULL) gCurPattern->PatternStart();
    }

    // Call the current pattern function once, updating the 'strip_A.pixels' array
    if (gCurPattern != NULL) gCurPattern->DrawFrame();

    if (gCurPattern != fppPattern && fppPattern != nullptr)
    {
      FastLED.setBrightness(bounded_brightness.Value());
      FastLED.show();
      FastLED.delay(101 - ( (gCurPattern->GetConstSpeed() == 0) ? bounded_speed.Value() : gCurPattern->GetConstSpeed() ));

      // Call the loop function on all colors so they can incrmenet themselves if desired
      std::for_each(_colors.begin(), _colors.end(), [](PatternColor* color) { color->Loop(); });
    }

    EVERY_N_MILLISECONDS( 100 )
    {
      SaveChangedSettings();
    }

    gPrevPattern = gCurPattern;
  }
}

CRGB LedDisplayManager::GetCurrentColor()
{
    return _colors[gCurColorIndex]->GetColor();
}

// Increment the current color, if applicable
void LedDisplayManager::IncrementColor()
{
  // std::for_each(_colors.begin(), _colors.end(), [](PatternColor* color) { color->Increment(); });
  _colors[gCurColorIndex]->Increment();
}

uint8_t LedDisplayManager::GetCurrentHue()
{
  return rainbowColor->GetCurrentHue();
}

void LedDisplayManager::Start()
{
  LedPattern::GetCurrentColorHandler = GetCurrentColor;
  LedPattern::GetCurrentHueHandler = GetCurrentHue;
  LedPattern::IncrementColorHandler = IncrementColor;

  // Make sure a rainbow color is added to the colors vector
  // If the rainbowColor pointer is still pointing at the internal instance, then none was added by the caller
  if (rainbowColor == &internal_rainbowColor)
  {
    AddColor(rainbowColor);
  }

  LoadSavedSettings();

  xTaskCreatePinnedToCore(&ledTask, "ledController", 4000, nullptr, 6, nullptr, 1);
}

void LedDisplayManager::GetColors(char * colorNames, int32_t * bufSize)
{
    int32_t accumulatedSz = 0;
    int32_t maxSz = 0;
    if (bufSize != NULL) maxSz = *bufSize;

    PatternColorConstIter curColor = _colors.begin();
    PatternColorConstIter endColor = _colors.end();

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

void LedDisplayManager::GetPatterns(char * patternNames, int32_t * bufSize)
{
    int32_t accumulatedSz = 0;
    int32_t maxSz = 0;
    if (bufSize != NULL) maxSz = *bufSize;

    LedPattern * curPattern = _patterns[0];
    LedPattern * endPattern = curPattern + _patterns.size();

    while (curPattern != endPattern)
    {
        int32_t len = strlen(curPattern->GetName());
        if (patternNames != NULL && (accumulatedSz + 1 + len) < maxSz)
        {
            strcpy(patternNames + accumulatedSz, curPattern->GetName());
            patternNames[accumulatedSz + len] = '\n';
        }
        accumulatedSz += len + 1;
        curPattern++;
    }

    // Replace last newline with null terminator to denote the end of the list
    if (accumulatedSz > 0 && patternNames != NULL) patternNames[accumulatedSz - 1] = 0;

    if (bufSize != NULL) *bufSize = accumulatedSz;
}

uint8_t LedDisplayManager::GetPatternIndex()
{
  return nextPatternIndex;
}

uint8_t LedDisplayManager::GetColorIndex()
{
  return nextColorIndex;
}

uint8_t LedDisplayManager::GetBrightness()
{
  return bounded_brightness.Value();
}

uint8_t LedDisplayManager::GetSpeed()
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
  if (_brightnessChangedCb != NULL) _brightnessChangedCb(LedDisplayManager::GetBrightness());
}

void SetSpeedChangedCallback(ValueChangedCb fn)
{
  _speedChangedCb = fn;

  // Call it right away to set an initial value
  if (_speedChangedCb != NULL) _speedChangedCb(LedDisplayManager::GetSpeed());
}

void GetColors(char * colorNames, int32_t * bufSize)
{
  LedDisplayManager::GetColors(colorNames, bufSize);
}

void GetPatterns(char * patternNames, int32_t * bufSize)
{
  LedDisplayManager::GetPatterns(patternNames, bufSize);
}


void LedDisplayManager::SetColor(uint8_t colorIndex)
{
  ESP_LOGI(__func__, "Setting color index to %u", colorIndex);
  if (colorIndex < _colors.size())
  {
    nextColorIndex = colorIndex;
    gColorSaveState.SetChanged();
    ESP_LOGI(__func__, "Color set to %s", _colors[nextColorIndex]->GetName());
    if (_colorChangedCb != NULL) _colorChangedCb(nextColorIndex);
  }
}

void LedDisplayManager::SetPattern(uint8_t patternIndex)
{
  ESP_LOGI(__func__, "Setting pattern index to %u", patternIndex);
  if (patternIndex < _patterns.size() && patternIndex != nextPatternIndex)
  {
    nextPatternIndex = patternIndex;
    gPatternSaveState.SetChanged();
    ESP_LOGI(__func__, "Pattern set to %s", _patterns[nextPatternIndex]->GetName());
    if (_patternChangedCb != NULL) _patternChangedCb(nextPatternIndex);
  }
}

void LedDisplayManager::SetSpeed(uint8_t speed)
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

void LedDisplayManager::SetBrightness(uint8_t brightness)
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
      for (; idx < _patterns.size(); idx++)
      {
        printf("  %c %d - %s\r\n", (nextPatternIndex == idx) ? '*' : ' ', idx, _patterns[idx]->GetName());
      }
    }
    else
    {
      int32_t newIdx = pattern_args.pattern_idx->ival[0];

      if (newIdx >= 0 && newIdx < _patterns.size())
      {
        LedDisplayManager::SetPattern((uint8_t)newIdx);
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
      for (; idx < _colors.size(); idx++)
      {
        printf("  %c %d - %s\r\n", (nextColorIndex == idx) ? '*' : ' ', idx, _colors[idx]->GetName());
      }
    }
    else
    {
      int32_t newIdx = color_args.color_idx->ival[0];

      if (newIdx >= 0 && newIdx < _colors.size())
      {
        LedDisplayManager::SetColor((uint8_t)newIdx);
      }
      else
      {
        ESP_LOGW(__func__, "Invalid color: %d", newIdx);
      }
    }

    return 0;
}

static struct
{
    struct arg_int *speed_val;
    struct arg_end *end;
} speed_args;

static int speed_cmd_func(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &speed_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, speed_args.end, argv[0]);
        return 1;
    }

    if (speed_args.speed_val->count == 0)
    {
      // No speed provided, so just display the current speed
      printf("\r\n");
      printf("Speed: %u\r\n", bounded_speed.Value());
      printf("Range: %u .. %u\r\n", bounded_speed.Min(), bounded_speed.Max());
    }
    else
    {
      int32_t newVal = speed_args.speed_val->ival[0];

      if (newVal >= bounded_speed.Min() && newVal <= bounded_speed.Max())
      {
        bounded_speed.Set((uint8_t)newVal);
      }
      else
      {
        ESP_LOGW(__func__, "Invalid speed value %d (range: %u..%u)", newVal, bounded_speed.Min(), bounded_speed.Max());
      }
    }

    return 0;
}

static struct
{
    struct arg_int *brightness_val;
    struct arg_end *end;
} brightness_args;

static int brightness_cmd_func(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &brightness_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, brightness_args.end, argv[0]);
        return 1;
    }

    if (brightness_args.brightness_val->count == 0)
    {
      // No brightness provided, so just display the current brightness
      printf("\r\n");
      printf("Brightness: %u\r\n", bounded_brightness.Value());
      printf("Range: %u .. %u\r\n", bounded_brightness.Min(), bounded_brightness.Max());
    }
    else
    {
      int32_t newVal = brightness_args.brightness_val->ival[0];

      if (newVal >= bounded_brightness.Min() && newVal <= bounded_brightness.Max())
      {
        bounded_brightness.Set((uint8_t)newVal);
      }
      else
      {
        ESP_LOGW(__func__, "Invalid brightness value %d (range: %u..%u)", newVal, bounded_brightness.Min(), bounded_brightness.Max());
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

  // Speed command
  speed_args.speed_val = arg_int0(NULL, NULL, "<speed>", "Speed value to set" );
  speed_args.end = arg_end(2);

  const esp_console_cmd_t speed_cmd =
  {
    .command = "speed",
    .help = "Display/set the speed",
    .hint = NULL,
    .func = &speed_cmd_func,
    .argtable = &speed_args
  };

  ESP_ERROR_CHECK( esp_console_cmd_register(&speed_cmd) );

  // Brightness command
  brightness_args.brightness_val = arg_int0(NULL, NULL, "<brightness>", "Brightness value to set" );
  brightness_args.end = arg_end(2);

  const esp_console_cmd_t brightness_cmd =
  {
    .command = "brightness",
    .help = "Display/set the brightness",
    .hint = NULL,
    .func = &brightness_cmd_func,
    .argtable = &brightness_args
  };

  ESP_ERROR_CHECK( esp_console_cmd_register(&brightness_cmd) );
}

uint8_t get_current_pattern_index() { return LedDisplayManager::GetPatternIndex(); }
uint8_t get_current_color_index() { return LedDisplayManager::GetColorIndex(); }
uint8_t get_brightness() { return LedDisplayManager::GetBrightness(); }
uint8_t get_speed() { return LedDisplayManager::GetSpeed(); }

void set_pattern_index(uint8_t index) { LedDisplayManager::SetPattern(index); }
void set_color_index(uint8_t index) { LedDisplayManager::SetColor(index); }
void set_brightness(uint8_t value) { LedDisplayManager::SetBrightness(value); }
void set_speed(uint8_t value) { LedDisplayManager::SetSpeed(value); }
