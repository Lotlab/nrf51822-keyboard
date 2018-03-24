#include "keyboard_led.h"
#include "keyboard_conf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_scheduler.h"
#include "led.h"

bool m_led_state[3] = {false};                    /**< LED State. */

/**@brief Notice by Led
 *
 * @param[in]   num   led val.
 * @param[in]   type  flash type;
 */
void set_led_num(uint8_t num)
{
    nrf_gpio_pin_write(LED_NUM, num & 0x01);
    nrf_gpio_pin_write(LED_CAPS, num & 0x02);
    nrf_gpio_pin_write(LED_SCLK, num & 0x04);
}

void led_appsh_evt_handler(void *p_event_data, uint16_t event_size)
{
    uint8_t num = ((uint8_t *)p_event_data)[0];
    uint8_t type = ((uint8_t *)p_event_data)[1];
    set_led_num(num);
    
    
    switch(type) 
    {
        case 0:
            set_led_num(num);
            nrf_delay_ms(50);
            set_led_num(0x00);
        break;
        case 1:
            set_led_num(0x07);
            nrf_delay_ms(100);
            set_led_num(num);
            nrf_delay_ms(100);
        break; 
    }
    
}

/**@brief Notice by Led
 *
 * @param[in]   num   led val.
 * @param[in]   type  flash type;
 */
void led_notice(uint8_t num, uint8_t type)
{
    // uint8_t evt_data[2] = {num, type};
    // app_sched_event_put(&evt_data, sizeof(evt_data), led_appsh_evt_handler);
    set_led_num(num);
}

void led_set(uint8_t usb_led)
{
		led_change_handler(usb_led, true);
}

// val: 灯光值
// all: 是否全局更新。若非全局更新则只更新指定的PIN
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

void led_init(void)
{
    nrf_gpio_cfg_output(LED_NUM);
    nrf_gpio_cfg_output(LED_CAPS);
    nrf_gpio_cfg_output(LED_SCLK);
}
