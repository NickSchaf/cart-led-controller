#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternGlitter : public LedPattern
{
  protected:
  uint16_t _chanceOfGlitter;
  LedPatternGlitter() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternGlitter() {}

  LedPatternGlitter(led_strip_list strips, uint16_t chanceOfGlitter = 80);
  virtual const char * GetName() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
