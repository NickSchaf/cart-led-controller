#include "LedSpecialPatternFPP.hpp"
#include <algorithm>

LedSpecialPatternFPP::LedSpecialPatternFPP(led_strip_t * strip)
: LedPattern({strip}), _isActive(false)
{
  _eventGroup = xEventGroupCreate();
  if (_eventGroup == NULL)
  {
    ESP_LOGE(__func__, "LedSpecialPatternFPP: Failed to create event group.");
  }
}

const char * LedSpecialPatternFPP::GetName() { return "[Falcon Player]"; }

void LedSpecialPatternFPP::PatternStart()
{
  // Not going to process E131 packets if we don't have a strip to put the data in
  if (_led_strips.size() > 0)
  {
    _isActive = true;
  }
}

void LedSpecialPatternFPP::PatternStop()
{
  _isActive = false;
}

#define EVENT_BIT_E131_FRAME_READY ( 1 << 0)

/// @brief Function to be called to set the pixels
void LedSpecialPatternFPP::DrawFrame()
{
  // Doesn't actually set the pixels; that work is done by the callback function
  // This waits for the callback to be finished filling in the pixels and outputs the frame
  const TickType_t maxWaitTicks = 20 / portTICK_PERIOD_MS;
  EventBits_t triggeredEvents = xEventGroupWaitBits(_eventGroup, EVENT_BIT_E131_FRAME_READY, pdTRUE, pdFALSE, maxWaitTicks);

  if ((triggeredEvents & EVENT_BIT_E131_FRAME_READY) != 0)
  {
    FastLED.show();
    // printf("%lu\n",millis());
  }
}

#define DMX_CHANNELS_PER_UNIVERSE 510
#define DMX_PIXELS_PER_UNIVERSE (DMX_CHANNELS_PER_UNIVERSE/3)
bool LedSpecialPatternFPP::E131Callback(e131_packet_t* ReceivedData, void* UserInfo)
{
  if (!_isActive)
  {
    // When not running the e131 mode, we'll drop all incoming packets
    // returning true so the caller won't add the packet to the queue
    return true;
  }
  else
  {
    if (ReceivedData == NULL) return false;
    led_strip_list::iterator iter = _led_strips.begin();

    uint16_t universe = ntohs(ReceivedData->universe);
    uint16_t prop_val_count = ntohs(ReceivedData->property_value_count);

    // printf("seq %u; univ %u, count %u\n", e131Packet.sequence_number, universe, prop_val_count);

    const uint16_t firstLedIdx = (universe - 1) * DMX_PIXELS_PER_UNIVERSE;
    CRGB * curLed = &(*iter)->pixels[firstLedIdx];
    const uint8_t * curChan = &ReceivedData->property_values[1];

#if 0
    static const CRGB * endLed = &(*iter)->pixels[(*iter)->pixel_count];
    const uint8_t * endChan = &ReceivedData->property_values[prop_val_count];

    // while (chanIdx < e131Packet.property_value_count && ledxIdx < ALL_LEDS_COUNT)
    while (curChan < endChan && curLed < endLed)
    {
      (curLed)->r = *(curChan++);
      (curLed)->g = *(curChan++);
      (curLed++)->b = *(curChan++);
    }
#else
    // Luckily CRGB is just 3 bytes in RGB order, so we can do a memcpy on top of the array
    memcpy((void *)curLed, curChan, std::min((unsigned int)(prop_val_count - 1), sizeof(CRGB) * ((*iter)->pixel_count - firstLedIdx)));

    // printf("%u, %u, %u\n", universe, prop_val_count, firstLedIdx);
#endif

    if (universe == 5)
    {
      // Notify the other thread that the buffer is ready
      xEventGroupSetBits(_eventGroup, EVENT_BIT_E131_FRAME_READY);
    }
    return true;
  }
}
