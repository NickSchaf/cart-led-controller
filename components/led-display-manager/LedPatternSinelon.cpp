#include "LedPatternSinelon.hpp"

LedPatternSinelon::LedPatternSinelon(led_strip_list strips)
: LedPattern(strips)
{
}

const char * LedPatternSinelon::GetName() { return "Sinelon"; }

/// @brief Function to be called to set the pixels
void LedPatternSinelon::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    // a colored dot sweeping back and forth, with fading trails
    fadeToBlackBy( (*iter)->pixels, (*iter)->pixel_count, 20);
    int pos = beatsin16( 13, 0, (*iter)->pixel_count-1 );
    (*iter)->pixels[pos] += CHSV( GetCurrentHue(), 255, 192);

    iter++;
  }
}
