#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternPolice : public LedPattern
{
  protected:
  led_strip_list _strips_red_B;
  led_strip_list _strips_blue_A;
  led_strip_list _strips_blue_B;
  int16_t _counter;
  bool _toggle;
  LedPatternPolice() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternPolice() {}

  LedPatternPolice(led_strip_list strips_red_A, led_strip_list strips_red_B, led_strip_list strips_blue_A, led_strip_list strips_blue_B);
  virtual const char * GetName() override;

  virtual uint8_t GetConstSpeed() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
