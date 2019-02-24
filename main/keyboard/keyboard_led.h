#ifndef __LED_H__
#define __LED_H__

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

void led_notice(uint8_t num, uint8_t type);
void led_change_handler(uint8_t val, uint8_t all);
void led_powersave_mode(bool powersave);
void led_init(void);

#endif
