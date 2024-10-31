#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternSinelon : public LedPattern
{
  protected:
  LedPatternSinelon() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternSinelon() {}

  LedPatternSinelon(led_strip_list strips);
  virtual const char * GetName() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
