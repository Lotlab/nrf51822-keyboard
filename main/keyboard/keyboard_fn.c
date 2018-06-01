/**
 * @brief 键盘内置功能
 * 
 * @file keyboard_fn.c
 * @author Jim Jiang
 * @date 2018-05-13
 */
#include <stdint.h>
#include "keyboard_fn.h"

#include "main.h"
#include "uart_driver.h"

void action_function(keyrecord_t *record, uint8_t id, uint8_t opt)
{
    switch(id)
    {
        case POWER_SLEEP:
            sleep_mode_enter(true);
        break;
        case SWITCH_DEVICE:
            #ifdef UART_SUPPORT
            if(kbd_mode == KEYBOARD_MODE_BLE)
                kbd_mode = KEYBOARD_MODE_USB;
            else 
                kbd_mode = KEYBOARD_MODE_BLE;
            #endif
        break;
        default:
            break;
    }
}
