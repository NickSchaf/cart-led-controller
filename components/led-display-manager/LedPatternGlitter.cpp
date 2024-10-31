#include "LedPatternGlitter.hpp"

LedPatternGlitter::LedPatternGlitter(led_strip_list strips, uint16_t chanceOfGlitter)
: LedPattern(strips)
{
  _chanceOfGlitter = chanceOfGlitter;
}

const char * LedPatternGlitter::GetName() { return "Glitter"; }

/// @brief Function to be called to set the pixels
void LedPatternGlitter::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    fill_solid( (*iter)->pixels, (*iter)->pixel_count, CRGB::Black);
    if( random8() < _chanceOfGlitter) {
      (*iter)->pixels[ random16((*iter)->pixel_count  ) ] += GetCurrentColor();
    }
    iter++;
  }

  IncrementColor();
}
