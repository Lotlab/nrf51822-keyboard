#ifndef __UART_DRIVER__
#define __UART_DRIVER__

#include <stdbool.h>
#include <stdint.h>

#ifdef UART_SUPPORT

#define UART_BAUDRATE NRF_UART_BAUDRATE_57600

/**
 * @brief
 * @note Packet format: Len TYPE DATA[Len-1]
 *       Len: the length of data and type, 
 *       Type: packet type.
 *       Data[]: data to transmit, last byte is the checksum. 
 *               if the length of data is 0, checksum can be omitted.
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
    UART_MODE_IDLE, // USB 未连接
    UART_MODE_CHARGING, // USB 已连接但尚未连接到主机
    UART_MODE_USB, // USB 主机已连接
    UART_MODE_BLE_OVERRIDE // USB 主机已连接但强制使用BLE模式通讯
} uart_mode;
extern uart_mode uart_current_mode;

void uart_init(void);
void uart_sleep_prepare(void);
void uart_send_packet(packet_type type, uint8_t* data, uint8_t len);
void uart_set_evt_handler(void (*evt)(bool));
bool uart_is_using_usb(void);
void uart_switch_mode(void);

#endif

#endif
