#include "LedPatternStrobe.hpp"

LedPatternStrobe::LedPatternStrobe(led_strip_list strips)
: LedPattern(strips), _lastState(false)
{
}

const char * LedPatternStrobe::GetName() { return "Strobe"; }

/// @brief Function to be called to set the pixels
void LedPatternStrobe::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    fill_solid( (*iter)->pixels, (*iter)->pixel_count, _lastState ? GetCurrentColor() : CRGB::Black );
    iter++;
  }

  IncrementColor();
  _lastState = !_lastState;
}
