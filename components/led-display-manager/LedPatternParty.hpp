#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternParty : public LedPattern
{
  protected:
  uint8_t _paletteStartIdx;
  LedPatternParty() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternParty() {}

  LedPatternParty(led_strip_list strips);
  virtual const char * GetName() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
