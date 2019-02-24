/**
 * @brief 键盘内置功能
 * 
 * @file keyboard_fn.c
 * @author Jim Jiang
 * @date 2018-05-13
 */
#include "keyboard_fn.h"
#include <stdint.h>

#include "main.h"
#include "uart_driver.h"

void action_function(keyrecord_t* record, uint8_t id, uint8_t opt)
{
    if (record->event.pressed) {
        switch (id) {
        case POWER_SLEEP:
            sleep_mode_enter(true);
            break;
        case SWITCH_DEVICE:
#ifdef UART_SUPPORT
            uart_switch_mode();
#endif
            break;
        default:
            break;
        }
    }
}
