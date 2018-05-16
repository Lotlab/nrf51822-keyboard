#ifndef __SYSTEM_H__
#define __SYSTEM_H__

extern void InitUART();
extern void CfgSysClock();
extern void DelayMs(uint16_t n);
extern void PrintHex(uint8_t * data, uint8_t len);

#endif // __SYSTEM_H__
