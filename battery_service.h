#ifndef __BATTERY_SERVICE__
#define __BATTERY_SERVICE__

#include "ble_bas.h"

#define BATTERY_LEVEL_MEAS_INTERVAL_SLOW APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define BATTERY_LEVEL_MEAS_INTERVAL_FAST APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< Battert level report interval (ticks). */
#define ADC_RESULT_QUEUE_SIZE 7
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS        1200                                     /**< Reference voltage (in  milli volts) used by ADC while doing conversion. */

void battery_timer_start(void);
void battery_service_init(void);
void battery_service_ble_evt(ble_evt_t *p_ble_evt);

#endif

