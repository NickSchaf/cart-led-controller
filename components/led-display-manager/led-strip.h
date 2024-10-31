#pragma once

#include <FastLED.h>

typedef struct led_strip_t
{
    uint16_t pixel_count;
    CRGB * pixels;
} led_strip_t;
