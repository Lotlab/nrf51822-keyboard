#include "CH554_SDCC.h"
#include "compiler.h"
#include "system.h"
#include "endpoints.h"
#include "interrupt.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#define LED RXD1
#define Ep2InKey P1_5;


void printHEX(uint8_t * string, uint8_t len)
{
    for(uint8_t i=0;i<len;i++)
    {
        printf("%02x ", string[i]);
    }
}

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
}


void main()
{
    CfgSysClock();
    DelayMs(5);                                                          //修改主频等待内部晶振稳定,必加
    InitUART();                                                        //串口0初始化
    DelayMs(5);
    printf_tiny("Build %s %s\n", __TIME__, __DATE__);

    USBDeviceInit();                                                      //USB设备模式初始化
    EA = 1;                                                               //允许单片机中断
    UEP1_T_LEN = 0;                                                       //预使用发送长度一定要清空
    UEP2_T_LEN = 0;                                                       //预使用发送长度一定要清空
    // FLAG = 0;
    // Ready = 0;

    while(1)
    {
        /*
        if(Ready)
        {
            // HIDValueHandle();
        }

        if(Ready && Ep2InKey == 0)
        {
            CH554USBDevWakeup();
            mDelaymS(10);
            Enp1IntIn();
        }
        */
        LED = !LED;
        DelayMs(500);
    }
}
