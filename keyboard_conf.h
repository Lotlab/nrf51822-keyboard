#ifndef _keyboard_conf_h_
#define _keyboard_conf_h_

#include "nrf_adc.h"

/** 电量测量引脚 */
#define KEYBOARD_ADC NRF_ADC_CONFIG_INPUT_2

/** 数字锁定灯IO口 */
#define LED_NUM 11
/** 大小写锁定灯IO口 */
#define LED_CAPS 12
/** 滚动锁定IO口 */
#define LED_SCLK 13

#endif
