#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "CH554_SDCC.h"
#include "compiler.h"
#include "system.h"
#include "endpoints.h"
#include "interrupt.h"
#include "usb_comm.h"
#include "uart.h"
#include "app_timer.h"

bool usb_evt = false;

/** \brief CH554软复位
 *
 */
void CH554SoftReset()
{
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;
    GLOBAL_CFG	|=bSW_RESET;
}

/** \brief CH554设备模式唤醒主机，发送K信号
 *
 */
void CH554USBDevWakeup( )
{
    UDEV_CTRL |= bUD_LOW_SPEED;
    DelayMs(2);
    UDEV_CTRL &= ~bUD_LOW_SPEED;
}

/** \brief CH559USB中断处理函数
 */
void DeviceInterrupt( void ) __interrupt INT_NO_USB __using 1                   //USB中断服务程序,使用寄存器组1
{
    UsbIsr();
    usb_evt = true;
}

void KeyboardGenericUpload(uint8_t * packet, uint8_t len)
{
    if(len != 8) return;

    memcpy(&Ep1Buffer[64], packet, len);
    UEP1_T_LEN = len;
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}

void KeyboardExtraUpload(uint8_t * packet, uint8_t len)
{
    if(len != 3) return;

    memcpy(Ep2Buffer, packet, len);
    UEP2_T_LEN = len;
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}

void ResponseConfigurePacket(uint8_t * packet, uint8_t len)
{
    if(len > 64) return;
    Ep3Buffer[64] = 0x3f;
    memcpy(&Ep3Buffer[65], packet, len);
    UEP3_T_LEN = 5;
    UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}

void UARTInterrupt(void) __interrupt INT_NO_UART1
{
    if(U1RI)
    {
        uart_recv();
        // U1RI = 0;
    }
/*
    if(U1TI)
    {
        U1TI = 0;
    }
*/
}


void EP3_OUT()
{
    uint8_t checksum = 0x00;
    for(int i=1;i<62;i++)
    {
        checksum += Ep3Buffer[i];
    }
    Ep3Buffer[62] = checksum;
    uart_send(PACKET_KEYMAP, &Ep3Buffer[1], 62);
}

// LED状态下传
void EP1_OUT()
{
    uint8_t datalen = USB_RX_LEN;
    uart_send(PACKET_LED, Ep1Buffer, 1);
}

void EnableWatchDog()
{
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xaa;                                                             //进入安全模式
    GLOBAL_CFG |= bWDOG_EN;                                                    //启动看门狗复位
    SAFE_MOD = 0x00;                                                             //退出安全模式
    WDOG_COUNT = 0;                                                              //看门狗赋初值
}

void TimerInterrupt( void ) __interrupt INT_NO_TKEY
{
    TKEY_CTRL = 0;
    timer_tick();
}

void ping_packet()
{
    if(uart_rx_state == STATE_IDLE)
    {
        if(usb_evt)
            uart_send(PACKET_USB_STATE, NULL, 0);
        else
            uart_send(PACKET_PING, NULL, 0);
    }
}


void timer_init()
{
    timer_create(&ping_packet, true, 500);
    timer_create(&uart_check, true, 2);
    IE_TKEY = 1;
}

void main()
{
    CfgSysClock();
    DelayMs(5);                                                          //修改主频等待内部晶振稳定,必加
    // InitUART();                                                        //串口0初始化
    uart_init();
    DelayMs(5);
    // printf_tiny("Build %s %s\n", __TIME__, __DATE__);
    timer_init();
    USBDeviceInit();                                                      //USB设备模式初始化
    EA = 1;                                                               //允许单片机中断
    UEP1_T_LEN = 0;                                                       //预使用发送长度一定要清空
    UEP2_T_LEN = 0;                                                       //预使用发送长度一定要清空

    while(1)
    {
        timer_task_exec();
    }
}
