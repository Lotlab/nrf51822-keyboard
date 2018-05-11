#ifndef _keyboard_conf_h_
#define _keyboard_conf_h_

#include <stdint.h>
#include "nrf_adc.h"
#include "config.h"

/** 电量测量引脚 */
#define KEYBOARD_ADC NRF_ADC_CONFIG_INPUT_2

/** 数字锁定灯IO口 */
#define LED_NUM 11
/** 大小写锁定灯IO口 */
#define LED_CAPS 12
/** 滚动锁定IO口 */
#define LED_SCLK 13


/** 行IO */
static const uint8_t row_pin_array[MATRIX_ROWS] = {21,22,23,24,25,26,27,29};
/** 列IO */
static const uint8_t column_pin_array[MATRIX_COLS] = {3,4,5,6,7,15,14,10,9,8,2,0,30,28};

#endif
