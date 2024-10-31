//+--------------------------------------------------------------------------
//
// NightDriver - (c) 2020 Dave Plummer.  All Rights Reserved.
//
// File:        fire.h
// Orig Loc:    https://github.com/davepl/DavesGarageLEDSeries/blob/master/LED%20Episode%2010/include/fire.h
//
// Description:
//
//              LED Flame Effect
//
// History:     Sep-28-2020     davepl      Created
//              Jul-31-2024     NickSchaf   Ported for ESP-IDF
//                                          Multiple LED array support
//                                          Variable blending options
//
//---------------------------------------------------------------------------

#pragma once

// #include <Arduino.h>
#define FASTLED_INTERNAL
#include <FastLED.h>

// #include "ledgfx.h"
#include "led-strip.h"

// Compatibility
// #ifndef UnixTime
// float UnixTime()
// {
//     return (float)esp_timer_get_time() / 1000000.0f;
// }
// #endif

typedef uint8_t byte;

class MathHelp
{
    public: 
    static float max(float a,float b) { return ((a)>(b)?(a):(b)); }
    static float min(float a,float b) { return ((a)<(b)?(a):(b)); }

    static int32_t random(int32_t low, int32_t high)
    {
        return low + (int32_t)((uint64_t)(high - low) * (uint64_t)esp_random() / (uint64_t)(UINT32_MAX));
    }
    static int32_t random(int32_t high) { return MathHelp::random(0, high); }
};

typedef struct blend_type_t
{
    uint8_t vector_size;               // Size of the vector
    int8_t vector_offset;              // Offset of pixel to start the blend, relative to the current pixel
    const int8_t  blend_vector[];      // The blending weights of self and neighbors
};

// When diffusing the fire upwards, these control how much to blend in from the cells below (ie: downward neighbors)
// You can tune these coefficients to control how quickly and smoothly the fire spreads.  
const blend_type_t ConvectionBlend = {
  .vector_size = 4,
  .vector_offset = 0,              // blend vector starts with the pixel being calculated (self)
  .blend_vector = { 2, 4, 3, 1 }   // blend factors for self and three pixels below
};

// Blend kernel appropriate for horizontal LED strips where the fire sparks along the entire strip and spreads
// across the strip
const blend_type_t HorizontalBlend = {
  .vector_size = 5,
  .vector_offset = -2,              // blend vector starts with two pixels above self
  .blend_vector = { 2, 4, 3, 4, 2 } // blend factors two pixels above self, self, & two pixels below self
};

class ClassicFireEffect
{
protected:
    led_strip_t* LedStrip;
    const blend_type_t * BlendType;
    int     Size;
    int     Cooling;
    int     Sparks;
    int     SparkHeight;
    int     Sparking;
    bool    bReversed;
    bool    bMirrored;

    // Two buffers for the heat because we toggle between them when updating frame-to-frame
    byte * heatA;
    byte * heatB;
    byte * heatCur;
    byte * heatNext;


public:
    
    // Lower sparking -> more flicker.  Higher sparking -> more consistent flame

    ClassicFireEffect(led_strip_t* led_strip, const blend_type_t * blend_type, int cooling = 80, int sparking = 50, int sparks = 3, int sparkHeight = 4, bool breversed = true, bool bmirrored = true) 
        : LedStrip(led_strip),
          BlendType(blend_type),
          Size(led_strip->pixel_count),
          Cooling(cooling),
          Sparks(sparks),
          SparkHeight(sparkHeight),
          Sparking(sparking),
          bReversed(breversed),
          bMirrored(bmirrored)
    {
        if (bMirrored)
            Size = Size / 2;

        heatA = new byte[Size] { 0 };
        heatB = new byte[Size] { 0 };
        heatCur = heatA;
        heatNext = heatB;
    }

    virtual ~ClassicFireEffect()
    {
        delete [] heatA;
        delete [] heatB;
    }

    virtual void DrawFire()
    {
        // First cool each cell by a little bit
        for (int i = 0; i < Size; i++)
            heatCur[i] = MathHelp::max(0L, heatCur[i] - MathHelp::random(0, ((Cooling * 10) / Size) + 2));

        // Next drift heat up and diffuse it a little but
        for (int i = 0; i < Size; i++)
        {
            int16_t blend_total = 0;
            int16_t temp = 0;
            for (int bvi = 0; bvi < BlendType->vector_size; bvi++)
            {
                int j = i + BlendType->vector_offset + bvi;
                if (j < 0) j = Size + j;
                if (j >= Size) j = j - Size;
                if (j >= 0  && j < Size)
                {
                    blend_total += (int16_t)BlendType->blend_vector[bvi];
                    temp += (int16_t)heatCur[j] * (int16_t)BlendType->blend_vector[bvi];
                }
            }

            heatNext[i] = (byte)MathHelp::min(255, MathHelp::max(0, (temp / blend_total)));
        }

        // Randomly ignite new sparks down in the flame kernel
        for (int i = 0; i < Sparks; i++)
        {
            if (MathHelp::random(255) < Sparking)
            {
                int y = Size - 1 - MathHelp::random(SparkHeight);
                // heatNext[y] = heatNext[y] + MathHelp::random(160, 200); // This randomly rolls over sometimes of course, and that's essential to the effect
                heatNext[y] = MathHelp::random(160, 220); // This randomly rolls over sometimes of course, and that's essential to the effect
            }
        }

        // Finally convert heat to a color
        for (int i = 0; i < Size; i++)
        {
            CRGB color = HeatColor(heatNext[i]);
            int j = bReversed ? (Size - 1 - i) : i;
            LedStrip->pixels[j] = color;
            if (bMirrored)
            {
                int j2 = !bReversed ? (2 * Size - 1 - i) : Size + i;
                LedStrip->pixels[j2] = color;
            }
        }

        // Swap the heat buffers for the next run
        heatCur = heatNext;
        heatNext = (heatNext == heatA) ? heatB : heatA;
    }
};
