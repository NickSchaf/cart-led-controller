#pragma once

#include "FastLED.h"
#include "led-strip.h"
#include <list>

typedef CRGB (*GetCurrentColorFunc)();
typedef uint8_t (*GetCurrentHueFunc)();
typedef void (*IncrementColorFunc)();

typedef std::list<led_strip_t*> led_strip_list;

/// @brief Base class for an 
class LedPattern
{
  protected:
  led_strip_list _led_strips;
  LedPattern() {}
  LedPattern(led_strip_list strips);
  void SetStrips(led_strip_list strips);

  static CRGB GetCurrentColor();
  static uint8_t GetCurrentHue();
  static void IncrementColor();

  public:
  /// @brief Destructor
  virtual ~LedPattern() {}

  // Patterns will need to be able to obtain the currently-selected color and hue from the
  // controller.  This would be the same for all instances of the pattern, so these are static.
  static GetCurrentColorFunc GetCurrentColorHandler;
  static GetCurrentHueFunc GetCurrentHueHandler;
  static IncrementColorFunc IncrementColorHandler;

  /// @brief Get the pattern name
  virtual const char * GetName();

  /// @brief Get the constant speed for this pattern (0 for variable speed)
  virtual uint8_t GetConstSpeed();

  /// @brief Function to be called when switching to this pattern
  virtual void PatternStart() {}

  /// @brief Function to be called when switching away from this pattern
  virtual void PatternStop() {}

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() {}
};
