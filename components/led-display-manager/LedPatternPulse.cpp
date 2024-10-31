#include "LedPatternPulse.hpp"

LedPatternPulse::LedPatternPulse(led_strip_list strips)
: LedPattern(strips)
{
}

const char * LedPatternPulse::GetName() { return "Pulse"; }

void LedPatternPulse::PatternStart()
{
  // On the first time through the loop after chaning patterns, we need to set the direction
  _increasing = false;
  _scaleVal = 255;
}

#define PULSE_STEP_SIZE 15
#define PULSE_MIN_SCALE 10

/// @brief Function to be called to set the pixels
void LedPatternPulse::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    (*iter)->pixels[0] = GetCurrentColor();
    (*iter)->pixels[0].nscale8_video(_scaleVal);

    for (int i = 1; i < (*iter)->pixel_count; i++)
      (*iter)->pixels[i] = (*iter)->pixels[0];

    iter++;
  }

  if (_increasing)
  {
    _scaleVal += PULSE_STEP_SIZE;
    if (255 - _scaleVal < PULSE_STEP_SIZE) _increasing = false;  // Reverse the scaling direction for next iteration
  }
  else
  {
    _scaleVal -= PULSE_STEP_SIZE;
    if (PULSE_MIN_SCALE + PULSE_STEP_SIZE > _scaleVal)
    {
      _increasing = true;  // Reverse the scaling direction for next iteration
      IncrementColor();
    }
  }
}
