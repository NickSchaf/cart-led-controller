#pragma once

#include "LedPattern.hpp"
#include "fire.h"

/// @brief Base class for an 
class LedPatternFire : public LedPattern
{
  typedef std::list<ClassicFireEffect*> heat_list;

  protected:
  led_strip_list _led_strips_vertical;
  heat_list _heat_list_horiz;
  heat_list _heat_list_vertical;
  LedPatternFire() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternFire() {}

  LedPatternFire(led_strip_list stripsHoriz, led_strip_list stripsVertical);
  virtual const char * GetName() override;

  void PatternStart() override;
  void PatternStop() override;

  virtual uint8_t GetConstSpeed() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
