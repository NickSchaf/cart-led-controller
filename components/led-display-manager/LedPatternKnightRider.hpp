#pragma once

#include "LedPattern.hpp"

/// @brief Base class for an 
class LedPatternKnightRider : public LedPattern
{
  class MovementData
  {
    public:
    int16_t Position;
    int16_t Increment;
  };

  protected:
  std::list<MovementData> _movementDataList;
  LedPatternKnightRider() { }

  public:
  /// @brief Destructor
  virtual ~LedPatternKnightRider() {}

  LedPatternKnightRider(led_strip_list strips);
  virtual const char * GetName() override;

  /// @brief Get the constant speed for this pattern (0 for variable speed)
  virtual uint8_t GetConstSpeed() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;
};
