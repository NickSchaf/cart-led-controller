#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternConfetti : public LedPattern
{
  protected:
  LedPatternConfetti() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternConfetti() {}

  LedPatternConfetti(led_strip_list strips);
  virtual const char * GetName() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
