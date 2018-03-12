#include "nrf.h"
#include "main.h"
#include "timer.h"

#include "app_scheduler.h"
#include "app_timer_appsh.h"

volatile uint32_t timer_count;

APP_TIMER_DEF(inner_timer_id);

void timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    timer_count++;
}

void timer_init()
{
    uint32_t err_code = app_timer_create(&inner_timer_id, APP_TIMER_MODE_REPEATED, timeout_handler);
    APP_ERROR_CHECK(err_code);
}

void timer_clear()
{
    timer_count = 0;
}

inline
uint16_t timer_read()
{
    return (uint16_t)(timer_count % 0xFFFF);
}

inline
uint32_t timer_read32()
{
    return timer_count;
}

inline
uint16_t timer_elapsed(uint16_t last)
{
    return TIMER_DIFF_16(timer_read(), last);
}

inline
uint32_t timer_elapsed32(uint32_t last)
{
    return TIMER_DIFF_32(timer_count, last);
}



