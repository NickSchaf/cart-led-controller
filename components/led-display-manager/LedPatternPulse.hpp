#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternPulse : public LedPattern
{
  protected:
  bool _increasing;
  uint8_t _scaleVal;
  LedPatternPulse() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternPulse() {}

  void PatternStart() override;

  LedPatternPulse(led_strip_list strips);
  virtual const char * GetName() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
