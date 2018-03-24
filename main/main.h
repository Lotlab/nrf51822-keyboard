#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdint.h>
#include <stdbool.h>

#define APP_TIMER_PRESCALER 0     /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 8 /**< Size of timer operation queues. */

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

void service_error_handler(uint32_t nrf_error);
void sleep_mode_enter(bool notice);

#endif
