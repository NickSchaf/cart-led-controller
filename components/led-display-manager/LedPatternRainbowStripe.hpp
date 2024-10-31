#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternRainbowStripe : public LedPattern
{
  protected:
  uint8_t _paletteStartIdx;
  LedPatternRainbowStripe() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternRainbowStripe() {}

  LedPatternRainbowStripe(led_strip_list strips);
  virtual const char * GetName() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
