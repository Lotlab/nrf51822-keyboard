/**
 * @brief 键盘计时器兼容层
 * 
 * @file keyboard_timer.c
 * @author Jim Jiang
 * @date 2018-05-13
 */
#include "main.h"
#include "nordic_common.h"
#include "nrf.h"
#include "timer.h"

#include "app_timer.h"

void timer_init()
{
    // do nothing
}

void timer_clear()
{
    // can't do anything
}

inline uint16_t timer_read()
{
    return (uint16_t)(timer_read32() % 0xFFFF);
}

inline uint32_t timer_read32()
{
    uint32_t time;
    // 24bit, 32768 tick per sec.
    // so only 19 bits are valid.

    // but, tmk does not use 32bit timer
    app_timer_cnt_get(&time);
    return time / 32;
}

inline uint16_t timer_elapsed(uint16_t last)
{
    return TIMER_DIFF_16(timer_read(), last);
}

inline uint32_t timer_elapsed32(uint32_t last)
{
    uint32_t time, elapsed;
    app_timer_cnt_get(&time);
    app_timer_cnt_diff_compute(time, last * 32, &elapsed);
    return elapsed / 32;
}
