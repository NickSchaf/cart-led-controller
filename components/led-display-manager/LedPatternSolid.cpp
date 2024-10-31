#include "LedPatternSolid.hpp"

LedPatternSolid::LedPatternSolid(led_strip_list strips, uint16_t colorIncrementTimeMs, const char * nameOverride)
: LedPattern(strips)
{
  _colorIncrTimeMs = colorIncrementTimeMs;
  _name = nameOverride;
}

const char * LedPatternSolid::GetName()
{
  if (_name != nullptr) return _name;
  else return "Solid";
}

/// @brief Function to be called to set the pixels
void LedPatternSolid::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    fill_solid( (*iter)->pixels, (*iter)->pixel_count, GetCurrentColor());
    iter++;
  }

  EVERY_N_MILLISECONDS(_colorIncrTimeMs)
  {
    IncrementColor();
  }
}
