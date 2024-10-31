#ifndef PATTERNCOLOR_HPP
#define PATTERNCOLOR_HPP

#include "FastLED.h"
#include <vector>

enum ColorIncrementMode
{
	None = 0,   // Doesn't increment (static color)
	Automatic,  // Increments itself
	External    // Incremented externally (usually controlled by the pattern)
};

class PatternColor
{
	protected:
	const char * _name;
	CRGB _curColor;
	ColorIncrementMode _incrementType;

	public:
	PatternColor(const char * name, CRGB initialColor);
	virtual CRGB GetColor() const { return _curColor; }
	virtual const char * GetName() const { return _name; }
	virtual void Loop() {}
	virtual ColorIncrementMode GetIncrementMode() const { return _incrementType; }
	virtual void Increment() {}
};

typedef std::vector<PatternColor*>::iterator PatternColorIter;
typedef std::vector<PatternColor*>::const_iterator PatternColorConstIter;

class PatternColor_Random : public PatternColor
{
	public:
	PatternColor_Random();
	void Increment() override;
};

class PatternColor_Rainbow : public PatternColor
{
	protected:
	uint8_t _curHue;

	public:
	PatternColor_Rainbow();
	void Loop() override;
	uint8_t GetCurrentHue() const { return _curHue; }
};

class PatternColor_Palette : public PatternColor
{
	protected:
	const CRGB * _palette;
	uint8_t _count;
	uint8_t _index;

	PatternColor_Palette(const char * name, const CRGB * palette, uint8_t paletteLen);
	void BumpIndex();
};

class PatternColor_PaletteAuto : public PatternColor_Palette
{
	public:
	PatternColor_PaletteAuto(const char * name, const CRGB * palette, uint8_t paletteLen);
	void Loop() override;
};

class PatternColor_PaletteManual : public PatternColor_Palette
{
	public:
	PatternColor_PaletteManual(const char * name, const CRGB * palette, uint8_t paletteLen);
	void Increment() override;
};

#endif
