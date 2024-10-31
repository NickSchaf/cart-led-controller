#include "LedPatternConfetti.hpp"

LedPatternConfetti::LedPatternConfetti(led_strip_list strips)
: LedPattern(strips)
{
}

const char * LedPatternConfetti::GetName() { return "Confetti"; }

/// @brief Function to be called to set the pixels
void LedPatternConfetti::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    // random colored speckles that blink in and fade smoothly
    fadeToBlackBy( (*iter)->pixels, (*iter)->pixel_count, 10);
    int pos = random16((*iter)->pixel_count);
    (*iter)->pixels[pos] += CHSV( GetCurrentHue() + random8(64), 200, 255);

    iter++;
  }
}
