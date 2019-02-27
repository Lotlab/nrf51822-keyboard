/**
 * @brief 键盘LED
 * 
 * @file keyboard_led.c
 * @author Jim Jiang
 * @date 2018-05-13
 */
#include "keyboard_led.h"
#include "app_timer_appsh.h"
#include "keyboard_conf.h"
#include "led.h"
#include "main.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

uint16_t led_state = 0x0000;

/**
 * @brief 底层设置LED状态
 * 
 * @param num 
 */
void led_set_state_ll(uint16_t num);

/**
 * @brief 闪烁所有LED
 * 
 */
void led_flash_all(void)
{
    led_set_state_ll(0xFFFF);
    nrf_delay_ms(100);
    led_set_state_ll(0x0000);
    nrf_delay_ms(100);
}

void led_flash_all_off(void* p)
{
    led_off();
}

/**
 * @brief 使用Timer闪烁所有LED
 * 
 */
void led_flash_all_timer(void)
{
    led_set_state_ll(0xFFFF);
    APP_TIMER_DEF(single_timer);
    app_timer_create(&single_timer, APP_TIMER_MODE_SINGLE_SHOT, led_flash_all_off);
    app_timer_start(single_timer, APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER), NULL);
}

/**
 * @brief 设置指定位置的LED状态
 * 
 * @param bit 位置
 * @param state 状态
 */
void led_set_bit(enum led_bit_usage bit, bool state)
{
    uint16_t val = led_state;
    if (state) {
        val |= 1 << bit;
    } else {
        val &= ~(1 << bit);
    }
    led_change_handler(val);
}

/**
 * @brief 设定指定范围内的LED状态
 * 
 * @param val LED状态
 * @param mask 范围 mask
 */
void led_set_mask(uint16_t mask, uint16_t val)
{
    uint16_t state = (led_state & ~mask) + (val & mask);
    led_change_handler(state);
}

/**
 * @brief HID下传数值设置LED状态
 * 
 * @param usb_led 三个LED位的值
 */
void led_set(uint8_t usb_led)
{
    led_set_mask(0x001F, usb_led);
}

/**
 * @brief 更新并暂存LED状态
 * 
 * @param val 全部的LED状态值
 */
void led_change_handler(uint16_t val)
{
    led_notice(val);
    led_state = val;
}

/**
 * @brief 关闭LED
 * 
 */
void led_off(void)
{
    led_set_state_ll(0x0000);
}

/**
 * @brief 开启LED
 * 
 */
void led_on(void)
{
    led_set_state_ll(led_state);
}

void led_timer_init(void);

/**
 * @brief 初始化LED
 * 
 */
void led_init(void)
{
#ifdef LED_NUM
    nrf_gpio_cfg_output(LED_NUM);
#endif
#ifdef LED_CAPS
    nrf_gpio_cfg_output(LED_CAPS);
#endif
#ifdef LED_SCLK
    nrf_gpio_cfg_output(LED_SCLK);
#endif
#ifdef LED_COMPOSE
    nrf_gpio_cfg_output(LED_COMPOSE);
#endif
#ifdef LED_KANA
    nrf_gpio_cfg_output(LED_KANA);
#endif
#ifdef LED_BLE
    nrf_gpio_cfg_output(LED_BLE);
#endif
#ifdef LED_USB
    nrf_gpio_cfg_output(LED_USB);
#endif
#ifdef LED_CHARGING
    nrf_gpio_cfg_output(LED_CHARGING);
#endif
#ifdef LED_FULL
    nrf_gpio_cfg_output(LED_FULL);
#endif
#ifdef LED_LOW_POWER
    nrf_gpio_cfg_output(LED_LOW_POWER);
#endif
#ifdef LED_USR1
    nrf_gpio_cfg_output(LED_USR1);
#endif
#ifdef LED_USR2
    nrf_gpio_cfg_output(LED_USR2);
#endif
#ifdef LED_USR3
    nrf_gpio_cfg_output(LED_USR3);
#endif
    led_timer_init();
}

/**
 * @brief 底层设置LED状态
 * 
 * @param num 
 */
void led_set_state_ll(uint16_t num)
{
#ifdef LED_NUM
    LED_WRITE(LED_NUM, num & (1 << LED_BIT_NUM));
#endif
#ifdef LED_CAPS
    LED_WRITE(LED_CAPS, num & (1 << LED_BIT_CAPS));
#endif
#ifdef LED_SCLK
    LED_WRITE(LED_SCLK, num & (1 << LED_BIT_SCLK));
#endif
#ifdef LED_COMPOSE
    LED_WRITE(LED_COMPOSE, num & (1 << LED_BIT_COMPOSE));
#endif
#ifdef LED_KANA
    LED_WRITE(LED_KANA, num & (1 << LED_BIT_KANA));
#endif
#ifdef LED_BLE
    LED_WRITE(LED_BLE, num & (1 << LED_BIT_BLE));
#endif
#ifdef LED_USB
    LED_WRITE(LED_USB, num & (1 << LED_BIT_USB));
#endif
#ifdef LED_CHARGING
    LED_WRITE(LED_CHARGING, num & (1 << LED_BIT_CHARGING));
#endif
#ifdef LED_FULL
    LED_WRITE(LED_FULL, num & (1 << LED_BIT_FULL));
#endif
#ifdef LED_LOW_POWER
    LED_WRITE(LED_LOW_POWER, num & (1 << LED_BIT_LOW_POWER));
#endif
#ifdef LED_USR1
    LED_WRITE(LED_USR1, num & (1 << LED_BIT_USR1));
#endif
#ifdef LED_USR2
    LED_WRITE(LED_USR2, num & (1 << LED_BIT_USR2));
#endif
#ifdef LED_USR3
    LED_WRITE(LED_USR3, num & (1 << LED_BIT_USR3));
#endif
}

#if LED_AUTOOFF_TIME > 0

bool counting;
bool led_autooff = true;
APP_TIMER_DEF(led_off_timer);

/**
 * @brief LED自动关闭的handler
 * 
 * @param context 
 */
void led_off_timer_handler(void* context)
{
    led_off();
    counting = false;
}

/**@brief 设置LED状态
 *
 * @param[in]   num   led val.
 */
void led_notice(uint16_t num)
{
    led_set_state_ll(num);
    if (led_autooff) {
        if (counting)
            app_timer_stop(led_off_timer);
        app_timer_start(led_off_timer, APP_TIMER_TICKS(LED_AUTOOFF_TIME, APP_TIMER_PRESCALER), NULL);
        counting = true;
    }
}
void led_timer_init(void)
{
    app_timer_create(&led_off_timer, APP_TIMER_MODE_SINGLE_SHOT, led_off_timer_handler);
}

/**
 * @brief 设置省电模式状态
 * 
 * @param powersave 
 */
void led_powersave_mode(bool powersave)
{
    if (counting)
        app_timer_stop(led_off_timer);

    led_autooff = powersave;
    led_notice(led_state);
}

#else

void led_timer_init()
{
}
void led_notice(uint16_t num)
{
    led_set_state_ll(num);
}
void led_powersave_mode(bool powersave)
{
}

#endif
