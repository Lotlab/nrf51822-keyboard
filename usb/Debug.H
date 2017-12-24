/* 调试 */
/* 提供printf子程序和延时函数 */
/* CH554主频修改、延时函数定义
   串口0和串口1初始化
   串口0和串口1的收发子函数
   看门狗初始化
*/

#include <stdint.h>

#ifndef	__DEBUG_H__
#define __DEBUG_H__

//定义函数返回值
#ifndef  SUCCESS
#define  SUCCESS  0
#endif
#ifndef  FAIL
#define  FAIL    0xFF
#endif

//定义定时器起始
#ifndef  START
#define  START  1
#endif
#ifndef  STOP
#define  STOP    0
#endif

#ifndef  DE_PRINTF
#define  DE_PRINTF     0
#endif
#define	 FREQ_SYS	     12000000	         //系统主频12MHz
#ifndef  UART0_BUAD
#define  UART0_BUAD    57600
#define  UART1_BUAD    57600
#endif

#define _putchar CH554UART0SendByte

#define DELAY_MS_HW 1

void	CfgFsys( );                        //CH554时钟选择和配置
void	mDelayuS( uint16_t n );              // 以uS为单位延时
void	mDelaymS( uint16_t n );              // 以mS为单位延时
void  CH554UART0Alter();                 //CH554串口0引脚映射到P0.2/P0.3
void	mInitSTDIO( );                      //T1作为波特率发生器
uint8_t CH554UART0RcvByte( );              //CH554 UART0查询方式接收一个字节
void  CH554UART0SendByte(uint8_t SendDat); //CH554UART0发送一个字节

void	UART1Setup( );                     //
uint8_t CH554UART1RcvByte( );              //CH554 UART1查询方式接收一个字节
void  CH554UART1SendByte(uint8_t SendDat); //CH554UART1发送一个字节

void CH554WDTModeSelect(uint8_t mode);     //CH554看门狗模式设置
void CH554WDTFeed(uint8_t tim);            //CH554看门狗喂狗
#endif
