#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternPride : public LedPattern
{
  protected:
  uint16_t _pseudotime = 0;
  uint16_t _lastMillis = 0;
  uint16_t _hue16 = 0;
  LedPatternPride() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternPride() {}

  LedPatternPride(led_strip_list strips);
  virtual const char * GetName() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
