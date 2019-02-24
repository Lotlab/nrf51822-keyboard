#ifndef __HIDS_H__
#define __HIDS_H__

#include "ble.h"
#include <stdbool.h>
#include <stdint.h>

void hids_init(void);
void hids_on_ble_evt(ble_evt_t* p_ble_evt);

void hids_keys_send(uint8_t key_pattern_len, uint8_t* p_key_pattern);
void hids_system_key_send(uint8_t key_pattern_len, uint8_t* p_key_pattern);
void hids_consumer_key_send(uint8_t key_pattern_len, uint8_t* p_key_pattern);

extern uint8_t led_val;

#endif
