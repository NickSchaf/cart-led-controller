#include "LedPatternPolice.hpp"

LedPatternPolice::LedPatternPolice(led_strip_list strips_red_A, led_strip_list strips_red_B, led_strip_list strips_blue_A, led_strip_list strips_blue_B)
: LedPattern(strips_red_A), _strips_red_B(strips_red_B), _strips_blue_A(strips_blue_A), _strips_blue_B(strips_blue_B), _counter(0), _toggle(false)
{
}

const char * LedPatternPolice::GetName() { return "Police"; }

uint8_t LedPatternPolice::GetConstSpeed() { return 70; };

#define POLICE_STROBE_COUNT 4
/// @brief Function to be called to set the pixels
void LedPatternPolice::DrawFrame()
{
  // Groups A are toggle==true
  // Groups B are toggle==false
  bool darkFrame = _counter % 2 == 0;

  // First the red side in A group
  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    if (_toggle && !darkFrame)
      fill_solid((*iter)->pixels, (*iter)->pixel_count, CRGB::Red);
    else
      fill_solid((*iter)->pixels, (*iter)->pixel_count, CRGB::Black);
    iter++;
  }

  // Red side in B group
  iter = _strips_red_B.begin();
  while (iter != _strips_red_B.end())
  {
    if (!_toggle && !darkFrame)
      fill_solid((*iter)->pixels, (*iter)->pixel_count, CRGB::Red);
    else
      fill_solid((*iter)->pixels, (*iter)->pixel_count, CRGB::Black);
    iter++;
  }

  // Blue side in A group
  iter = _strips_blue_A.begin();
  while (iter != _strips_blue_A.end())
  {
    if (!_toggle && !darkFrame)
      fill_solid((*iter)->pixels, (*iter)->pixel_count, CRGB::Blue);
    else
      fill_solid((*iter)->pixels, (*iter)->pixel_count, CRGB::Black);
    iter++;
  }

  // Blue side in B group
  iter = _strips_blue_B.begin();
  while (iter != _strips_blue_B.end())
  {
    if (_toggle && !darkFrame)
      fill_solid((*iter)->pixels, (*iter)->pixel_count, CRGB::Blue);
    else
      fill_solid((*iter)->pixels, (*iter)->pixel_count, CRGB::Black);
    iter++;
  }

  _counter++;
  if (_counter >= (2 * POLICE_STROBE_COUNT)) { 
    _counter = 0;
    _toggle = !_toggle;
  }

}
