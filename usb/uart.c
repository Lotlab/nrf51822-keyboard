#include "uart.h"
#include "CH554_SDCC.h"
#include "usb_comm.h"
#include "system.h"

#define CHARGING UCC1
#define STANDBY  UCC2

typedef enum {
    STATE_IDLE, // 接收完毕
    STATE_DATA, // 正在接收数据
} state;


static state current;
static uint8_t len,pos;
static uint8_t __xdata recv_buff[64];
static packet_type send_type;

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
    uart_tx(0);
    uart_tx(PACKET_ACK);
}

void uart_init()
{
    U1SM0 = 0;    // 8Bit
    U1SMOD = 1;   // fast mode
    U1REN = 1;                                                                   //串口0接收使能
    SBAUD1 = 256 - FREQ_SYS / 16 / 57600;
    IE_UART1 = 1;                                                               //启用串口中断
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
        KeyboardGenericUpload(&recv_buff[1], len - 1);
        uart_ack();
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
    switch(current)
    {
        case STATE_IDLE:
            len = uart_rx();
            pos = 0;
            if(len > 0) current = STATE_DATA;
            else uart_data_parser();
        case STATE_DATA:
            recv_buff[pos++] = uart_rx();
            if(pos >= len)
            {
                current = STATE_IDLE;
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


