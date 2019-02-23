/**
 * @brief 键盘LED
 * 
 * @file keyboard_led.c
 * @author Jim Jiang
 * @date 2018-05-13
 */
#include "main.h"
#include "keyboard_led.h"
#include "keyboard_conf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_timer_appsh.h"
#include "led.h"

bool m_led_state[3] = {false};                    /**< LED State. */
bool counting;
#ifdef POWER_SAVE_MODE
bool led_autooff = true;
#else
bool led_autooff = false;
#endif
APP_TIMER_DEF(led_off);


/**
 * @brief 底层设置LED状态
 * 
 * @param num 
 */
void set_led_num(uint8_t num)
{
#ifdef LED_NUM
    nrf_gpio_pin_write(LED_NUM, num & 0x01);
#endif
#ifdef LED_CAPS
    nrf_gpio_pin_write(LED_CAPS, num & 0x02);
#endif
#ifdef LED_SCLK
    nrf_gpio_pin_write(LED_SCLK, num & 0x04);
#endif
}

/**
 * @brief 底层设置蓝牙LED指示灯状态
 * 
 * @param val 
 */
void set_blue_led(uint8_t val)
{
#ifdef LED_BLE
    nrf_gpio_pin_write(LED_BLE, val);
#endif
}

/**
 * @brief 底层设置电池LED指示灯状态
 * 
 * @param val 
 */
void set_battery_led(uint8_t val)
{
#ifdef LED_BATT
    nrf_gpio_pin_write(LED_BATT, val);
#endif
}

/**@brief Notice by Led
 *
 * @param[in]   num   led val.
 * @param[in]   type  flash type;
 */
void led_notice(uint8_t num, uint8_t type)
{
    switch(type) 
    {
        case 0:
            set_led_num(num);
            if(led_autooff)
            {
                if(counting) app_timer_stop(led_off);
                app_timer_start(led_off, APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER), NULL);
                counting = true;
            }
        break;
        case 1:
            set_led_num(0x07);
            nrf_delay_ms(100);
            set_led_num(num);
            nrf_delay_ms(100);
        break; 
    }
}
/**
 * @brief 设置三个键盘LED状态
 * 
 * @param usb_led 三个键盘LED位的值
 */
void led_set(uint8_t usb_led)
{
	led_change_handler(usb_led, true);
}

/**
 * @brief 更新LED状态
 * 
 * @param val 若全局更新，则指示全部的LED状态值。若非全局更新，则指示翻转对应位上的LED状态
 * @param all 是否全局更新
 */
void led_change_handler(uint8_t val, uint8_t all)
{
    if(!all)
    {
        uint8_t newBit = 0x00;
        for(uint8_t i=0; i<3; i++)
            newBit |= m_led_state[i] << i;
        
        val = newBit ^ val; 
    }
    
    led_notice(val, 0x00); // 处理LED的显示
    for(uint8_t i=0; i<3; i++) // 更新暂存状态
        m_led_state[i] = val & 1 << i;
}
/**
 * @brief 关闭LED
 * 
 * @param p_context 
 */
void led_turnoff(void * p_context)
{
    set_led_num(0x00);
    counting = false;
}

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
#ifdef LED_BLE
    nrf_gpio_cfg_output(LED_BLE);
#endif
#ifdef LED_BATT
    nrf_gpio_cfg_output(LED_BATT);
#endif
    app_timer_create(&led_off, APP_TIMER_MODE_SINGLE_SHOT, led_turnoff);
}

void led_powersave_mode(bool powersave)
{
    led_autooff = powersave;
    if(!powersave)
    {
        led_change_handler(0x00,false);
    }
}

