#include "LedPatternFire.hpp"

LedPatternFire::LedPatternFire(led_strip_list stripsHoriz, led_strip_list stripsVertical)
: LedPattern(stripsHoriz), _led_strips_vertical(stripsVertical)
{
}

const char * LedPatternFire::GetName() { return "Fire"; }

uint8_t LedPatternFire::GetConstSpeed() { return 90; };

void LedPatternFire::PatternStart()
{
  led_strip_list::iterator iter = _led_strips.begin();
  while (iter != _led_strips.end())
  {
    // Create a heat array for each led strip
    _heat_list_horiz.push_back(new ClassicFireEffect(*iter,                     // led_strip
                                                     &HorizontalBlend,          // blend_type
                                                     30,                        // cooling
                                                     70,                        // sparking
                                                     (*iter)->pixel_count / 20, // sparks
                                                     (*iter)->pixel_count,      // sparkHeight
                                                     true,                      // breversed
                                                     false)                     // bmirrored
                              );
    iter++;
  }

  iter = _led_strips_vertical.begin();
  while (iter != _led_strips_vertical.end())
  {
    // Create a heat array for each led strip
    _heat_list_vertical.push_back(new ClassicFireEffect(*iter,                     // led_strip
                                                        &ConvectionBlend,          // blend_type
                                                        20,                        // cooling
                                                        70,                        // sparking
                                                        (*iter)->pixel_count / 15, // sparks
                                                        (*iter)->pixel_count / 10, // sparkHeight
                                                        true,                      // breversed
                                                        false)                     // bmirrored
                                 );
    iter++;
  }
}

void LedPatternFire::PatternStop()
{
  // Clean up the heat arrays
  heat_list::iterator heat_iter = _heat_list_horiz.begin();
  while (heat_iter != _heat_list_horiz.end())
  {
    delete *heat_iter++;
  }

  heat_iter = _heat_list_vertical.begin();
  while (heat_iter != _heat_list_vertical.end())
  {
    delete *heat_iter++;
  }
}

/// @brief Function to be called to set the pixels
void LedPatternFire::DrawFrame()
{
  heat_list::iterator heat_iter = _heat_list_horiz.begin();
  while (heat_iter != _heat_list_horiz.end())
  {
    (*heat_iter)->DrawFire();
    heat_iter++;
  }

  heat_iter = _heat_list_vertical.begin();
  while (heat_iter != _heat_list_vertical.end())
  {
    (*heat_iter)->DrawFire();
    heat_iter++;
  }
}
