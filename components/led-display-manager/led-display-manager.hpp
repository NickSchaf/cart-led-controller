#ifndef LED_CONTROLLER_HPP
#define LED_CONTROLLER_HPP

#include "led-display-manager.h"
#include "FastLED.h"
#include "PatternColor.hpp"
#include "LedPattern.hpp"
#include <vector>

class LedDisplayManager
{
	public:
	static void Start();

	static void AddPattern(LedPattern* pattern);
	static void AddColor(PatternColor* color);
	static void AddLedStrip(led_strip_t * strip);
	// Could also add functions to remove items or clear the vectors, will do that when they're needed

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

	private:
	static void ledTask(void *pvParameters);
	static void LoadSavedSettings();
	static void SaveU8Value(const char * settingName, uint8_t value);
	static void SaveChangedSettings();
	static CRGB GetCurrentColor();
	static void IncrementColor();
	static uint8_t GetCurrentHue();
};

#endif
