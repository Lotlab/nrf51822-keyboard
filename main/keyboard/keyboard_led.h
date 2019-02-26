#ifndef __LED_H__
#define __LED_H__

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum led_bit_usage {
    LED_BIT_NUM = 0,
    LED_BIT_CAPS,
    LED_BIT_SCLK,
    LED_BIT_COMPOSE,
    LED_BIT_KANA,

    // 自定义灯
    LED_BIT_BLE = 0x08,
    LED_BIT_USB,
    LED_BIT_CHARGING,
    LED_BIT_FULL,
    LED_BIT_LOW_POWER,

    LED_BIT_USR1,
    LED_BIT_USR2,
    LED_BIT_USR3
};

/**
 * @brief 闪烁所有LED
 * 
 */
void led_flash_all(void);
/**
 * @brief 使用Timer闪烁所有LED
 * 
 */
void led_flash_all_timer(void);
/**@brief 设置LED状态
 *
 * @param[in]   num   led val.
 */
void led_notice(uint16_t num);
/**
 * @brief 更新并暂存LED状态
 * 
 * @param val 全部的LED状态值
 */
void led_change_handler(uint16_t val);
/**
 * @brief 设定指定范围内的LED状态
 * 
 * @param mask 范围 mask
 * @param val LED状态
 */
void led_set_mask(uint16_t mask, uint16_t val);
/**
 * @brief 设置指定位置的LED状态
 * 
 * @param bit 位置
 * @param state 状态
 */
void led_set_bit(enum led_bit_usage bit, bool state);
/**
 * @brief 设置省电模式状态
 * 
 * @param powersave 
 */
void led_powersave_mode(bool powersave);
/**
 * @brief 初始化LED
 * 
 */
void led_init(void);
/**
 * @brief 关闭所有LED
 * 
 */
void led_off(void);
/**
 * @brief 开启所有LED
 * 
 */
void led_on(void);

#endif
