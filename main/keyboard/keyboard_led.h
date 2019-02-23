#ifndef __LED_H__
#define __LED_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

void led_notice(uint8_t num, uint8_t type);
void led_change_handler(uint8_t val, uint8_t all);
void led_powersave_mode(bool powersave);
void led_init(void);
void set_blue_led(uint8_t val);
void set_battery_led(uint8_t val);

#endif
