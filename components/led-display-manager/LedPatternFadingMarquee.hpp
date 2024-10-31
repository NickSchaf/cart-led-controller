#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternFadingMarquee : public LedPattern
{
  protected:
  led_strip_list _led_strips_reverse;
  uint8_t _barSize;
  uint8_t _gapSize;
  int16_t _position;
  int16_t _increment;
  bool _toggle;
  LedPatternFadingMarquee() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternFadingMarquee() {}

  LedPatternFadingMarquee(led_strip_list stripsForward, led_strip_list stripsReverse, uint8_t barSize = 4, uint8_t gapSize = 3);
  virtual const char * GetName() override;

  void PatternStart() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
