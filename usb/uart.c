#include "uart.h"
#include "CH554_SDCC.h"
#include "usb_comm.h"
#include "system.h"
#include <stdbool.h>

#define CHARGING UCC1
#define STANDBY  UCC2

uart_state uart_rx_state;
static uint8_t len,pos;
static uint8_t __xdata recv_buff[64];
static packet_type send_type;

static bool uart_check_flag;

void uart_send(packet_type type, uint8_t * data, uint8_t len);

void uart_tx(uint8_t c)
{
    SBUF1 = c;
	while(U1TI == 0);
	U1TI = 0;
}

uint8_t uart_rx()
{
    while(U1RI == 0);
    U1RI = 0;
    return SBUF1;
}

void uart_ack()
{
    uart_send(PACKET_ACK, 0, 0);
}
void uart_fail()
{
    uart_send(PACKET_FAIL, 0, 0);
}

uint8_t checksum()
{
    uint8_t sum = 0x00;

    for(int i=1;i < len - 1;i++)
        sum += recv_buff[i];
    return sum == recv_buff[len - 1];
}

void uart_init()
{
    U1SM0 = 0;    // 8Bit
    U1SMOD = 1;   // fast mode
    U1REN = 1;                                                                   //串口0接收使能
    SBAUD1 = 256 - FREQ_SYS / 16 / 57600;
    IE_UART1 = 1;                                                               //启用串口中断
}

void uart_check()
{
    if(uart_check_flag && uart_rx_state == STATE_DATA)
    {
        // 超时强制退出
        uart_rx_state = STATE_IDLE;
    }
    uart_check_flag = true;
}

void uart_data_parser(void)
{
    switch((packet_type)recv_buff[0])
    {
    case PACKET_ACK:
    case PACKET_FAIL:
        if(send_type == PACKET_KEYMAP)
        {
            ResponseConfigurePacket(recv_buff,1);
        }
        break;
    case PACKET_KEYBOARD:
        if(checksum())
        {
            KeyboardGenericUpload(&recv_buff[1], len - 2);
            uart_ack();
        }
        else
        {
            uart_fail();
        }

        break;
    case PACKET_SYSTEM:
        // 使用type位置暂存一下ID
        recv_buff[0] = 2;
        KeyboardExtraUpload(recv_buff, 3);
        uart_ack();
        break;
    case PACKET_COMSUMER:
        recv_buff[0] = 3;
        KeyboardExtraUpload(recv_buff, 3);
        uart_ack();
        break;
    case PACKET_GET_STATE:
        recv_buff[0] = CHARGING;
        uart_send(PACKET_CHARGING, recv_buff, 1);
        break;

    }
}

void uart_recv(void)
{
    switch(uart_rx_state)
    {
        case STATE_IDLE:
            len = uart_rx();
            pos = 0;
            if(len > 0) uart_rx_state = STATE_DATA;
            else uart_data_parser();
        case STATE_DATA:
            recv_buff[pos++] = uart_rx();
            uart_check_flag = false;
            if(pos >= len)
            {
                uart_rx_state = STATE_IDLE;
                uart_data_parser();
            }
            break;
    }
}

void uart_send(packet_type type, uint8_t * data, uint8_t len)
{
    send_type = type;
    uart_tx(len + 1);
    uart_tx(type);
    while(len--){
        uart_tx(*(data++));
    }
}


