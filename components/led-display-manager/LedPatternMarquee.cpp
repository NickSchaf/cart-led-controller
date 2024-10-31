#include "LedPatternMarquee.hpp"

LedPatternMarquee::LedPatternMarquee(led_strip_list strips, uint8_t barSize, uint8_t gapSize)
: LedPattern(strips), _barSize(barSize), _gapSize(gapSize)
{
}

const char * LedPatternMarquee::GetName() { return "Marquee"; }

void LedPatternMarquee::PatternStart()
{
  _position = 0;
  _increment = 1;
  _toggle = false;

  // TODO: Pre-fill the strip(s); otherwise we need to wait until the whole pattern shifts through
  //       Simple method may be to just call DrawFrame() once for every pixel in the largest strip
}

/// @brief Function to be called to set the pixels
void LedPatternMarquee::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();

  CRGB curColor = GetCurrentColor();
  while (iter != _led_strips.end())
  {
    // Shift all pixels
    CRGB* dest = (*iter)->pixels + (*iter)->pixel_count - 1;
    CRGB* src = dest - 1;
    while (dest != (*iter)->pixels)
    {
      *dest-- = *src--;
    }

    (*iter)->pixels[0] = _toggle ? curColor : CRGB::Black;

    iter++;
  }

  if (++_position == (_toggle ? _barSize : _gapSize) )
  {
    if (_toggle)          // Make sure we only increment color when switching off a bar, not both off and on
    {
      IncrementColor();   // This won't work well with multiple strips
    }
    _toggle = !_toggle;
    _position = 0;
  }

}
