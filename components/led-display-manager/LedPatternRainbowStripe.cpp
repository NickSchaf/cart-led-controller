#include "LedPatternRainbowStripe.hpp"

LedPatternRainbowStripe::LedPatternRainbowStripe(led_strip_list strips)
: LedPattern(strips), _paletteStartIdx(0)
{
}

const char * LedPatternRainbowStripe::GetName() { return "Rainbow Stripe"; }

/// @brief Function to be called to set the pixels
void LedPatternRainbowStripe::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    uint8_t paletteIdx = (_paletteStartIdx += 3);
    for( int i = 0; i < (*iter)->pixel_count; i++) {
        (*iter)->pixels[i] = ColorFromPalette( RainbowStripeColors_p, paletteIdx, 255, LINEARBLEND);   // brightness here was using the configurable brightness before
        paletteIdx += 3;
    }

    iter++;
  }
}
