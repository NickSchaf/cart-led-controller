#include "LedPatternPride.hpp"

LedPatternPride::LedPatternPride(led_strip_list strips)
: LedPattern(strips), _pseudotime(0), _lastMillis(0), _hue16(0)
{
}

const char * LedPatternPride::GetName() { return "Pride"; }

/// @brief Function to be called to set the pixels
void LedPatternPride::DrawFrame()
{
  const uint8_t sat8 = beatsin88( 87, 220, 250);
  const uint8_t brightdepth = beatsin88( 341, 96, 224);
  const uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
  const uint8_t msmultiplier = beatsin88(147, 23, 60);
  const uint16_t hueinc16 = beatsin88(113, 1, 3000);

  uint16_t hue16 = _hue16;//gHue * 256;
  
  uint16_t ms = millis();
  uint16_t deltams = ms - _lastMillis ;
  _lastMillis  = ms;
  _pseudotime += deltams * msmultiplier;
  _hue16 += deltams * beatsin88( 400, 5,9);
  uint16_t brightnesstheta16 = _pseudotime;

  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    for( uint16_t i = 0 ; i < (*iter)->pixel_count; i++) {
      hue16 += hueinc16;

      brightnesstheta16  += brightnessthetainc16;
      uint16_t b16 = sin16( brightnesstheta16  ) + 32768;

      uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
      uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
      bri8 += (255 - brightdepth);
      
      CRGB newcolor = CHSV( hue16 / 256, sat8, bri8);
      
      uint16_t pixelnumber = i;
      pixelnumber = ((*iter)->pixel_count-1) - pixelnumber;
      
      nblend( (*iter)->pixels[pixelnumber], newcolor, 64);
    }

    iter++;
  }
}
