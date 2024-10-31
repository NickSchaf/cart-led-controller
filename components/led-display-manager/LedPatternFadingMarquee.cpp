#include "LedPatternFadingMarquee.hpp"

LedPatternFadingMarquee::LedPatternFadingMarquee(led_strip_list stripsForward, led_strip_list stripsReverse, uint8_t barSize, uint8_t gapSize)
: LedPattern(stripsForward), _led_strips_reverse(stripsReverse), _barSize(barSize), _gapSize(gapSize)
{
}

const char * LedPatternFadingMarquee::GetName() { return "Fading Marquee"; }

void LedPatternFadingMarquee::PatternStart()
{
  _position = 0;
  _increment = 1;
  _toggle = false;

  // TODO: Pre-fill the strip(s); otherwise we need to wait until the whole pattern shifts through
  //       Simple method may be to just call DrawFrame() once for every pixel in the largest strip
}

/// @brief Function to be called to set the pixels
void LedPatternFadingMarquee::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();

  CRGB curColor = GetCurrentColor();
  while (iter != _led_strips.end())
  {
    // Fade previous frame - only do right half; will copy to the other side below
    // Experimentation yielded a fade step eq: step = 13 - led_count/16
    fadeToBlackBy((*iter)->pixels, (*iter)->pixel_count, 13 - (*iter)->pixel_count / 16);

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

  iter = _led_strips_reverse.begin();
  while (iter != _led_strips_reverse.end())
  {
    // Fade previous frame - only do right half; will copy to the other side below
    // Experimentation yielded a fade step eq: step = 13 - led_count/16
    fadeToBlackBy((*iter)->pixels, (*iter)->pixel_count, 13 - (*iter)->pixel_count / 16);

    // Shift all pixels the other direction
    const CRGB* end = (*iter)->pixels + (*iter)->pixel_count;
    CRGB* dest = (*iter)->pixels;
    CRGB* src = dest + 1;
    while (src != end)
    {
      *dest++ = *src++;
    }

    (*iter)->pixels[(*iter)->pixel_count - 1] = _toggle ? curColor : CRGB::Black;

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
