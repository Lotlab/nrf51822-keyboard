/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_hids_keyboard_main main.c
 * @{
 * @ingroup ble_sdk_app_hids_keyboard
 * @brief HID Keyboard Sample Application main file.
 *
 * This file contains is the source code for a sample application using the HID, Battery and Device
 * Information Services for implementing a simple keyboard functionality.
 * Pressing Button 0 will send text 'hello' to the connected peer. On receiving output report,
 * it toggles the state of LED 2 on the mother board based on whether or not Caps Lock is on.
 * This application uses the @ref app_scheduler.
 *
 * Also it would accept pairing requests from any peer device.
 */

#include "main.h"
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "ble.h"
#include "ble_advertising.h"
#include "softdevice_handler_appsh.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "nrf_delay.h"
#include "app_trace.h"
#include "pstorage.h"

#include "report.h"
#include "keyboard_matrix.h"
#include "keycode.h"
#include "keymap.h"
#include "battery_service.h"
#include "keyboard_led.h"
#include "ble_hid_service.h"
#include "keyboard.h"
#include "hook.h"
#include "custom_hook.h"
#include "ble_services.h"

#define SLEEP_DEVIDE 10
#define SLEEP_SLOW_TIMEOUT 60 // 60秒后转入慢速扫描模式
#define SLEEP_OFF_TIMEOUT 600 // 10分钟之后自动关机

#define KEYBOARD_SCAN_INTERVAL APP_TIMER_TICKS(25, APP_TIMER_PRESCALER)                  /**< Keyboard scan interval (ticks). */
#define KEYBOARD_SCAN_INTERVAL_SLOW APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)            /**< Keyboard slow scan interval (ticks). */
#define KEYBOARD_FREE_INTERVAL APP_TIMER_TICKS(1000 * SLEEP_DEVIDE, APP_TIMER_PRESCALER) /**< Keyboard sleep interval. */

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE MAX(APP_TIMER_SCHED_EVT_SIZE, \
                                      BLE_STACK_HANDLER_SCHED_EVT_SIZE) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 10                                             /**< Maximum number of events in the scheduler queue. */

APP_TIMER_DEF(m_keyboard_scan_timer_id);
APP_TIMER_DEF(m_keyboard_sleep_timer_id);

void sleep_mode_enter(void);

static uint8_t passkey_enter_index = 0xFF;
static uint8_t passkey_entered[6];

static uint16_t sleep_counter = 0;

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void keyboard_scan_timeout_handler(void *p_context);
static void keyboard_sleep_timeout_handler(void *p_context);
/**@brief 计时器初始化函数
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

    err_code = app_timer_create(&m_keyboard_scan_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                keyboard_scan_timeout_handler);

    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_keyboard_sleep_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                keyboard_sleep_timeout_handler);

    APP_ERROR_CHECK(err_code);
}



/**@brief 初始化程序所需的服务
 */
static void services_init(void)
{
    hids_init();
}

bool keyboard_conn_pass_enter_handler(const uint8_t *key_packet, uint8_t key_packet_size)
{
    if (passkey_enter_index < 6 && auth_key_reqired())
    {
        for (uint_fast8_t i = 0; i < key_packet_size; i++)
        {
            if (key_packet[i] >= KC_1)
            {
                if (key_packet[i] <= KC_0)
                {
                    passkey_entered[passkey_enter_index++] = (key_packet[i] + 1 - KC_1) % 10 + '0';
                }
                else if (key_packet[i] >= KC_KP_1 && key_packet[i] <= KC_KP_0)
                {
                    passkey_entered[passkey_enter_index++] = (key_packet[i] + 1 - KC_KP_1) % 10 + '0';
                }
                else if (key_packet[i] == KC_BSPACE) passkey_enter_index--;
            }
        }
        if (passkey_enter_index == 6)
        {
            auth_key_reply(passkey_entered);
            passkey_enter_index = 0;
        }
        return true;
    }
    else
    {
        return false;
    }
}

static void keyboard_switch_scan_mode(bool slow)
{
    uint32_t err_code;
    err_code = app_timer_stop(m_keyboard_scan_timer_id);
    if (slow)
        err_code = app_timer_start(m_keyboard_scan_timer_id, KEYBOARD_SCAN_INTERVAL_SLOW, NULL);
    else
        err_code = app_timer_start(m_keyboard_scan_timer_id, KEYBOARD_SCAN_INTERVAL, NULL);

    APP_ERROR_CHECK(err_code);
}

static void keyboard_sleep_timeout_handler(void *p_context)
{
    sleep_counter++;
    if (sleep_counter == SLEEP_SLOW_TIMEOUT / SLEEP_DEVIDE)
    {
        keyboard_switch_scan_mode(true);
    }
    else if (sleep_counter == SLEEP_OFF_TIMEOUT / SLEEP_DEVIDE)
    {
        sleep_mode_enter();
    }
}

static void keyboard_sleep_counter_reset(void)
{
    if (sleep_counter >= SLEEP_SLOW_TIMEOUT / SLEEP_DEVIDE)
    {
        keyboard_switch_scan_mode(false);
    }
    sleep_counter = 0;
}

/**@brief Function for handling the keyboard scan timer timeout.
 *
 * @details This function will be called each time the keyboard scan timer expires.
 *
 */
static void keyboard_scan_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
	keyboard_task();
}

void hook_matrix_change(keyevent_t event)
{
    keyboard_sleep_counter_reset();
}

void hook_send_keyboard(report_keyboard_t * report)
{
    keyboard_conn_pass_enter_handler(report->keys, sizeof(report->keys));
}


/**@brief 计时器启动函数
 */
static void timers_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_keyboard_scan_timer_id, KEYBOARD_SCAN_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_keyboard_sleep_timer_id, KEYBOARD_FREE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    battery_timer_start();
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void sleep_mode_enter(void)
{
    uint32_t err_code;

    matrix_sleep_prepare();
    led_notice(0x00, 0x01);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief   Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t *p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);

    ble_services_evt_dispatch(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);

    hids_on_ble_evt(p_ble_evt);
    battery_service_ble_evt(p_ble_evt);
}

/**@brief   Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_APPSH_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_250MS_CALIBRATION, true);

    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(S132))
    ble_enable_params.gatts_enable_params.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(void)
{
    keyboard_setup();
    led_init();
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    _Bool erase_bond = false;

    // Initialize.
    app_trace_init();
    timers_init();
    buttons_leds_init();
    ble_stack_init();
    scheduler_init();

    // erase_bond = cherry8x16_getch(KC_2);

    ble_services_init(erase_bond);
    services_init();
    battery_service_init();

    led_notice(0x00, 0x01);
	keyboard_init();
	
    // Start execution.
    timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    
    led_change_handler(0x01, true);

    // Enter main loop.
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }
}
