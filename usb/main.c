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


uint8_t static __xdata HIDKey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t static __xdata Upload[] = {0x00, 0x00};

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

uint16_t GetTouchData(uint8_t ch)
{
    TKEY_CTRL = ((TKEY_CTRL & 0xF8) | ch) + 1;
    while((TKEY_CTRL&bTKC_IF) == 0);                                          //bTKC_IF变为1时，本周期采样完成
    return TKEY_DAT & 0x3FFF;
}


/** \brief USB设备模式端点1的中断上传
 *
 */
void Enp1IntIn()
{
    memcpy(&Ep1Buffer[64], HIDKey, sizeof(HIDKey));                              //加载上传数据
    UEP1_T_LEN = sizeof(HIDKey);                                             //上传数据长度
    UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                //有数据时上传数据并应答ACK
}

/** \brief USB设备模式端点2的中断上传
*/

void Enp2IntIn()
{
    memcpy( &Ep2Buffer[64], Upload, sizeof(Upload));                           //加载上传数据
    UEP2_T_LEN = 8;                                                           //上传数据长度
    UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                 //有数据时上传数据并应答ACK
}

void main()
{
    uint16_t touchData;
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
    TKEY_CTRL = TKEY_CTRL & 0xF8 | 0x03;

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

        touchData = GetTouchData(2);
        if(touchData < 0x400 )
        {
            LED = !LED;
            SendFinish = 0;
            HIDKey[2] = 0x04;
            Enp1IntIn();
            while(!SendFinish);
            HIDKey[2] = 0x00;
            Enp1IntIn();
            while(!SendFinish);
            memcpy(&Upload[0], &touchData, sizeof(touchData));
            Enp2IntIn();

            while(GetTouchData(2)<0x400)DelayMs(2);
        }

        DelayMs(20);
    }
}
