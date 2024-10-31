#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternJuggle : public LedPattern
{
  protected:
  LedPatternJuggle() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternJuggle() {}

  LedPatternJuggle(led_strip_list strips);
  virtual const char * GetName() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
