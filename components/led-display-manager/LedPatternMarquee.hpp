#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternMarquee : public LedPattern
{
  protected:
  uint8_t _barSize;
  uint8_t _gapSize;
  int16_t _position;
  int16_t _increment;
  bool _toggle;
  LedPatternMarquee() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternMarquee() {}

  LedPatternMarquee(led_strip_list strips, uint8_t barSize = 4, uint8_t gapSize = 3);
  virtual const char * GetName() override;

  void PatternStart() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
