#include "LedPatternBPM.hpp"

LedPatternBPM::LedPatternBPM(led_strip_list strips)
: LedPattern(strips)
{
}

const char * LedPatternBPM::GetName() { return "BPM"; }

/// @brief Function to be called to set the pixels
void LedPatternBPM::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
    uint8_t BeatsPerMinute = 62;
    CRGBPalette16 palette = PartyColors_p;
    uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
    for( int i = 0; i < (*iter)->pixel_count; i++) { //9948
      (*iter)->pixels[i] = ColorFromPalette(palette, GetCurrentHue()+(i*2), beat-GetCurrentHue()+(i*10));
    }
   iter++;
  }
}
