#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternStrobe : public LedPattern
{
  protected:
  bool _lastState;
  LedPatternStrobe() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternStrobe() {}

  LedPatternStrobe(led_strip_list strips);
  virtual const char * GetName() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
