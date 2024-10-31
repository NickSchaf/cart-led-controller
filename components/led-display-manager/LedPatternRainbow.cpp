#include "LedPatternRainbow.hpp"

LedPatternRainbow::LedPatternRainbow(led_strip_list strips)
: LedPattern(strips)
{
}

const char * LedPatternRainbow::GetName() { return "Rainbow"; }

/// @brief Function to be called to set the pixels
void LedPatternRainbow::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    fill_rainbow( (*iter)->pixels, (*iter)->pixel_count, GetCurrentHue(), 7);
    iter++;
  }
}
