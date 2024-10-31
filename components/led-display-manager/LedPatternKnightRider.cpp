#include "LedPatternKnightRider.hpp"

LedPatternKnightRider::LedPatternKnightRider(led_strip_list strips)
: LedPattern(strips), _movementDataList(strips.size(), { .Position = 0, .Increment = 1})
{
}

const char * LedPatternKnightRider::GetName() { return "Knight Rider"; }

uint8_t LedPatternKnightRider::GetConstSpeed() { return 100; };

/// @brief Function to be called to set the pixels
void LedPatternKnightRider::DrawFrame()
{
  led_strip_list::iterator iter = _led_strips.begin();
  std::list<MovementData>::iterator md_iter = _movementDataList.begin();
  while (iter != _led_strips.end())
  {
    const int16_t numLeds = (*iter)->pixel_count / 2;

    fadeToBlackBy((*iter)->pixels, (*iter)->pixel_count, 100);
    (*iter)->pixels[md_iter->Position] = GetCurrentColor();

    // Mirror on the other side - TODO: This is based on sending only one strip
    (*iter)->pixels[(*iter)->pixel_count - md_iter->Position - 1] = (*iter)->pixels[md_iter->Position];

    md_iter->Position += md_iter->Increment;
    if (md_iter->Position >= numLeds) {
      md_iter->Position = numLeds - 1;
      md_iter->Increment = -1;
      IncrementColor();             // This won't work well with multiple strips
    }
    if (md_iter->Position < 0) {
      md_iter->Position = 0;
      md_iter->Increment = 1;
      IncrementColor();             // This won't work well with multiple strips
    }

    iter++;
    md_iter++;
  }
}
