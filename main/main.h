#ifndef __MAIN_H__
#define __MAIN_H__

#include <stdint.h>
#include "device_manager.h"

#define APP_TIMER_PRESCALER 0     /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 4 /**< Size of timer operation queues. */

extern dm_handle_t m_bonded_peer_handle;

void service_error_handler(uint32_t nrf_error);

#endif
