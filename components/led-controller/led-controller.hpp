#ifndef LED_CONTROLLER_HPP
#define LED_CONTROLLER_HPP

#include "led-controller.h"
#include "FastLED.h"

#define USE_NEW_COLOR_CLASSES 1

class LedController
{
	public:
	static void Start();

	static void GetColors(char * colorNames, int32_t * bufSize);
	static void GetPatterns(char * patternNames, int32_t * bufSize);

	static uint8_t GetPatternIndex();
	static uint8_t GetColorIndex();
	static uint8_t GetBrightness();
	static uint8_t GetSpeed();

	static void SetColor(uint8_t colorIndex);
	static void SetPattern(uint8_t patternIndex);
	static void SetSpeed(uint8_t speed);
	static void SetBrightness(uint8_t brightness);

	// TODO: Functions to get/set which segments (roof, sides, underbody) are enabled

	private:
};

class SaveableSetting
{
	private:
	bool _changed;
	uint32_t _changedTime;

	public:
	SaveableSetting();
	void SetChanged();
	void ClearChanged();
	bool NeedsSaving();
};

class BoundedValue
{
	private:
	uint8_t _value;
	uint8_t _min;
	uint8_t _max;
	uint8_t _step;

	public:
	BoundedValue(uint8_t min, uint8_t max, uint8_t step = 1, uint8_t value = 0);
	bool Set(uint8_t value);
	uint8_t Max() const { return _max; }
	uint8_t Min() const { return _min; }
	uint8_t Value() const { return _value; }
};

#endif
