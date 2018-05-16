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

    memcpy(Ep3Buffer, packet, len);
    UEP2_T_LEN = len;
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}

void ResponseConfigurePacket(uint8_t * packet, uint8_t len)
{
    if(len>64) return;
    memcpy(&Ep3Buffer[64], packet, len);
    UEP3_T_LEN = len;
    UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
}

void EP3_OUT()
{
    PrintHex(Ep3Buffer, USB_RX_LEN);
}

void EP1_OUT()
{
    // 既可以从EP0端点的SET_CONFIGURATION获取下传的报告值
    // 也可以从EPn的OUT输出获取下传的报告值
    uint8_t datalen = USB_RX_LEN;
    TXD1 = !(Ep1Buffer[0] & 0x02);
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

    P1_DIR_PU &= 0x0C;

    while(1)
    {
        DelayMs(50);
    }
}
