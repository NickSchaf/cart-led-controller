#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternSolid : public LedPattern
{
  protected:
  uint16_t _colorIncrTimeMs = 1000;
  const char * _name = nullptr;
  LedPatternSolid() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternSolid() {}

  LedPatternSolid(led_strip_list strips, uint16_t colorIncrementTimeMs = 1000, const char * nameOverride = nullptr);
  virtual const char * GetName() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
