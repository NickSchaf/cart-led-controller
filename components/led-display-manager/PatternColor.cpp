#include <stdint.h>

#include "PatternColor.hpp"

PatternColor::PatternColor(const char * name, CRGB initialColor) :
  _name(name), _curColor(initialColor), _incrementType(None)
{ }

PatternColor_Random::PatternColor_Random() :
  PatternColor("[Random]", CRGB::Black)
{
  _incrementType = External;

  // Call increment to set the first random color
  Increment();
}

void PatternColor_Random::Increment()
{
  _curColor = CHSV(random8(), 255, 255);  
}

PatternColor_Rainbow::PatternColor_Rainbow() :
  PatternColor("[Rainbow]", CRGB::Black), _curHue(0)
{ }

void PatternColor_Rainbow::Loop()
{
  EVERY_N_MILLISECONDS( 20 )
  {
    _curHue++;   // slowly cycle the "base color" through the rainbow
    _curColor = CHSV( _curHue, 255, 255);
  }
}

PatternColor_Palette::PatternColor_Palette(const char * name, const CRGB * palette, uint8_t paletteLen) :
  PatternColor(name, CRGB::Black), _palette(palette), _count(paletteLen), _index(0)
{
  if (_palette != NULL && _count > 0)
  {
    _curColor = _palette[_index];
  }
}

void PatternColor_Palette::BumpIndex()
{
  if (_palette != NULL && _count > 0)
  {
    if (++_index == _count)
    {
      _index = 0;
    }
    _curColor = _palette[_index];
  }
}

PatternColor_PaletteAuto::PatternColor_PaletteAuto(const char * name, const CRGB * palette, uint8_t paletteLen) :
  PatternColor_Palette(name, palette, paletteLen)
{
  _incrementType = Automatic;
}

void PatternColor_PaletteAuto::Loop()
{
  EVERY_N_MILLISECONDS( 1500 )
  {
    BumpIndex();
  }
}


PatternColor_PaletteManual::PatternColor_PaletteManual(const char * name, const CRGB * palette, uint8_t paletteLen) :
  PatternColor_Palette(name, palette, paletteLen)
{
  _incrementType = External;
}

void PatternColor_PaletteManual::Increment()
{
  BumpIndex();
}
