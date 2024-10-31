#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H


#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ValueChangedCb)(uint8_t value);

void register_led_controller_cmds(void);

uint8_t get_current_pattern_index();
uint8_t get_current_color_index();
uint8_t get_brightness();
uint8_t get_speed();

void set_pattern_index(uint8_t index);
void set_color_index(uint8_t index);
void set_brightness(uint8_t value);
void set_speed(uint8_t value);

void SetPatternChangedCallback(ValueChangedCb fn);
void SetColorChangedCallback(ValueChangedCb fn);
void SetBrightnessChangedCallback(ValueChangedCb fn);
void SetSpeedChangedCallback(ValueChangedCb fn);

void GetColors(char * colorNames, int32_t * bufSize);
void GetPatterns(char * patternNames, int32_t * bufSize);


#ifdef __cplusplus
}
#endif


#endif
