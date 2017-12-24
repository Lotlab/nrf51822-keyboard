#include "CH554_SDCC.h"
#include "compiler.h"

#define FREQ_SYS 12000000
#define UART0_BUAD 57600


/** \brief 配置系统时钟
 *
 * \param void
 * \return void
 *
 */
void CfgSysClock()
{
    SAFE_MOD = 0x55;    //Enter unsafe mode
    SAFE_MOD = 0xAA;
    CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x04;  // 12MHz
    SAFE_MOD = 0x00;
}

/** \brief 初始化UART
 *
 */
void InitUART()
{
    uint32_t x;
    // uint32_t x2;

    SM0 = 0;    // 8Bit
    SM1 = 1;    // Variable
    SM2 = 0;    // Single device

    RCLK = 0;   // using timer1 as send/recv clock
    TCLK = 0;   //

    PCON |= SMOD;   //SMOD = 0; uart slow mode

    /*
    x = 8 * FREQ_SYS / UART0_BUAD / 16;                                       //如果更改主频，注意x的值不要溢出
    x2 = x % 8;
    x /= 10;
    if ( x2 >= 4 ) x ++;                                                       //四舍五入
    */

    x = FREQ_SYS / UART0_BUAD / 16;

    TMOD = TMOD & ~ bT1_GATE & ~ bT1_CT & ~ MASK_T1_MOD | bT1_M1;              //0X20，Timer1作为8位自动重载定时器
    T2MOD = T2MOD | bTMR_CLK | bT1_CLK;                                        //Timer1时钟选择
    TH1 = 0-x;                                                                 //12MHz晶振,buad/12为实际需设置波特率
    TR1 = 1;                                                                   //启动定时器1
    TI = 1;
    REN = 1;                                                                   //串口0接收使能
}

/** \brief 延时x毫秒
 *
 * \param 毫秒数
 * \return
 *
 */
void DelayMs(uint16_t n)                                                  // 以mS为单位延时
{
    // md为何会用这么dirty的方法做延时啊
	while ( n-- ) {
		while ( ( TKEY_CTRL & bTKC_IF ) == 0 );
		while ( TKEY_CTRL & bTKC_IF );
	}
}

int putchar(int c)
{
    //查询发送，中断方式可不用下面2条语句,但发送前需TI=0
	while(TI == 0);
	TI = 0;
    SBUF = c;
    return c;
}

int getchar()
{
    while(RI == 0);                                                            //查询接收，中断方式可不用
    RI = 0;
    return SBUF;
}
