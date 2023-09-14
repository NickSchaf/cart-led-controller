/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#ifndef GATTS_TABLE_CREAT_DEMO_H
#define GATTS_TABLE_CREAT_DEMO_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#ifdef __cplusplus
extern "C" {
#endif

/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_PATTERN,
    IDX_CHAR_VAL_PATTERN,
    // IDX_CHAR_CFG_A,

    IDX_CHAR_COLOR,
    IDX_CHAR_VAL_COLOR,

    IDX_CHAR_BRIGHTNESS,
    IDX_CHAR_VAL_BRIGHTNESS,

    IDX_CHAR_SPEED,
    IDX_CHAR_VAL_SPEED,

    IDX_CHAR_PATTERN_LIST,
    IDX_CHAR_VAL_PATTERN_LIST,

    IDX_CHAR_COLOR_LIST,
    IDX_CHAR_VAL_COLOR_LIST,

    IDX_END,
};

void setup_GATTS(void);

// typedef void (*ValueChangedCb)(uint8_t value);
void PatternChangedCallback(uint8_t value);
void ColorChangedCallback(uint8_t value);
void BrightnessChangedCallback(uint8_t value);
void SpeedChangedCallback(uint8_t value);
void SetPatternList(const char * list, int32_t list_len);
void SetColorList(const char * list, int32_t list_len);


#ifdef __cplusplus
}
#endif

#endif // GATTS_TABLE_CREAT_DEMO_H