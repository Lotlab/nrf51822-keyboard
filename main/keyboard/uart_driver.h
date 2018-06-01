#ifndef __UART_DRIVER__
#define __UART_DRIVER__

#include <stdint.h>
#include <stdbool.h>

#ifdef UART_SUPPORT

/**
 * @brief
 * @note 包格式为 Len TYPE DATA[Len-1]
 *       Len: 除Len字节外的包长度
 *       Type: 包类型
 *       Data[]: 实际数据，长度为len-1
 * 
 */

typedef enum {
    // uart rx
    PACKET_PING = 0x00,
    PACKET_LED,
    PACKET_CHARGING,
    PACKET_KEYMAP,
    PACKET_USB_STATUS,
    
    // uart tx
    PACKET_KEYBOARD = 0x80,
    PACKET_SYSTEM,
    PACKET_COMSUMER,
    PACKET_GET_STATE,
    
    // uart other
    PACKET_FAIL = 0xc0,
    PACKET_ACK,
} packet_type;

typedef enum {
    KEYBOARD_MODE_BLE,
    KEYBOARD_MODE_USB
} keyboard_mode;

#define UART_BAUDRATE NRF_UART_BAUDRATE_57600

void uart_init(void);
void uart_sleep_prepare(void);
void uart_send_packet(packet_type type, uint8_t * data, uint8_t len);
void uart_set_evt_handler(void (*evt)(bool));
extern bool uart_enable;
extern bool usb_enable;
extern keyboard_mode kbd_mode;

#define USE_USB (uart_enable && usb_enable && kbd_mode == KEYBOARD_MODE_USB)

#endif

#endif
