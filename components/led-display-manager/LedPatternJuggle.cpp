#include "LedPatternJuggle.hpp"

LedPatternJuggle::LedPatternJuggle(led_strip_list strips)
: LedPattern(strips)
{
}

const char * LedPatternJuggle::GetName() { return "Juggle"; }

/// @brief Function to be called to set the pixels
void LedPatternJuggle::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    // eight colored dots, weaving in and out of sync with each other
    fadeToBlackBy( (*iter)->pixels, (*iter)->pixel_count, 20);
    uint8_t dothue = 0;
    for( int i = 0; i < 8; i++) {
      (*iter)->pixels[beatsin16( i+7, 0, (*iter)->pixel_count-1 )] |= CHSV(dothue, 200, 255);
      dothue += 32;
    }

    iter++;
  }
}
