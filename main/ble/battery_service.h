#ifndef __BATTERY_SERVICE__
#define __BATTERY_SERVICE__

#include "ble_bas.h"

/** 电量慢速测量计时器 (30秒) */
#define BATTERY_LEVEL_MEAS_INTERVAL_SLOW APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
/** 电量快速测量计时器 (1秒) */
#define BATTERY_LEVEL_MEAS_INTERVAL_FAST APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
/** 电量测量结果队列长度 */
#define ADC_RESULT_QUEUE_SIZE 7
/** ADC参考电源 (mV) */
#define ADC_REF_VOLTAGE_IN_MILLIVOLTS 1200

/** 启动电量计时器 */
void battery_timer_start(void);
/** 初始化电量服务 */
void battery_service_init(void);
/** 蓝牙电量服务事件回调 */
void battery_service_ble_evt(ble_evt_t* p_ble_evt);

#endif
