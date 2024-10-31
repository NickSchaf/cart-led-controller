#pragma once

#include "LedPattern.hpp"
#include "ESPAsyncE131.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"

/// @brief Base class for an 
class LedSpecialPatternFPP : public LedPattern
{
  protected:
  bool _isActive;
  EventGroupHandle_t _eventGroup;
  LedSpecialPatternFPP() { }

  public:
  /// @brief Destructor
  virtual ~LedSpecialPatternFPP() {}

  LedSpecialPatternFPP(led_strip_t * strip);
  virtual const char * GetName() override;

  void PatternStart() override;
  void PatternStop() override;

  /// @brief Function to be called to set the pixels
  virtual void DrawFrame() override;

  bool E131Callback(e131_packet_t* ReceivedData, void* UserInfo);
};
