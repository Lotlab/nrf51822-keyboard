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
#include "ble_advertising.h"
#include "softdevice_handler_appsh.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "pstorage.h"
#include "nrf_drv_wdt.h"

#include "ble_services.h"
#include "battery_service.h"
#include "ble_hid_service.h"

#include "keyboard.h"
#include "keyboard_led.h"
#include "keyboard_matrix.h"
#include "keyboard_host_driver.h"
#include "keymap_storage.h"

#include "report.h"
#include "hook.h"
#include "custom_hook.h"
#include "bootmagic.h"
#include "eeconfig.h"
#include "uart_driver.h"

#define KEYBOARD_SCAN_INTERVAL APP_TIMER_TICKS(KEYBOARD_FAST_SCAN_INTERVAL, APP_TIMER_PRESCALER)                  /**< Keyboard scan interval (ticks). */
#define KEYBOARD_SCAN_INTERVAL_SLOW APP_TIMER_TICKS(KEYBOARD_SLOW_SCAN_INTERVAL, APP_TIMER_PRESCALER)            /**< Keyboard slow scan interval (ticks). */
#define KEYBOARD_FREE_INTERVAL APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)                /**< 键盘Tick计时器 */
#define KEYBOARD_WDT_INTERVAL APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE MAX(APP_TIMER_SCHED_EVT_SIZE, \
                                      BLE_STACK_HANDLER_SCHED_EVT_SIZE) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 0x10                                             /**< Maximum number of events in the scheduler queue. */

APP_TIMER_DEF(m_keyboard_scan_timer_id);
APP_TIMER_DEF(m_keyboard_sleep_timer_id);
APP_TIMER_DEF(m_keyboard_wdt_timer_id);

static uint8_t passkey_enter_index = 0;
static uint8_t passkey_entered[6];

static uint16_t sleep_timer_counter = 0;
static nrf_drv_wdt_channel_id m_channel_id;

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
    printf("Line %d: %s", line_num, p_file_name);
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
static void keyboard_wdt_timeout_handler(void *p_context);

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
    
    err_code = app_timer_create(&m_keyboard_wdt_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                keyboard_wdt_timeout_handler);

    APP_ERROR_CHECK(err_code);
}

/**@brief 初始化程序所需的服务
 */
static void services_init(void)
{
    hids_init();
    battery_service_init();
}

/**
 * @brief 处理配对码输入
 * 
 * @param key_packet 按键报文
 * @param key_packet_size 报文长度
 */
void keyboard_conn_pass_enter_handler(uint8_t *key_packet, uint8_t key_packet_size)
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
    }
}

/**
 * @brief 切换扫描模式
 * 
 * @param slow 切换至慢速扫描
 */
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

/**
 * @brief 键盘睡眠定时器
 * 
 * @param p_context 
 */
static void keyboard_sleep_timeout_handler(void *p_context)
{
    sleep_timer_counter++;
    if (sleep_timer_counter == SLEEP_SLOW_TIMEOUT)
    {
        keyboard_switch_scan_mode(true);
    }
    else if (sleep_timer_counter == SLEEP_OFF_TIMEOUT)
    {
        sleep_mode_enter(true);
    }
}
/**
 * @brief 重置键盘睡眠定时器
 * 
 */
static void keyboard_sleep_counter_reset(void)
{
    if (sleep_timer_counter >= SLEEP_SLOW_TIMEOUT)
    {
        keyboard_switch_scan_mode(false);
    }
    sleep_timer_counter = 0;
}

/**@brief Function for handling the keyboard scan timer timeout.
 *
 * @details This function will be called each time the keyboard scan timer expires.
 *
 */
static void keyboard_scan_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    // well, as fast as possible, it's impossible.
	keyboard_task();
}
/**
 * @brief 键盘扫描结果改变的Hook
 * 
 * @param event 
 */
void hook_matrix_change(keyevent_t event)
{
    keyboard_sleep_counter_reset();
}
/**
 * @brief 键盘按键按下的Hook
 */
void hook_key_change()
{
#ifdef SLOW_MODE_EARLY_EXIT
    keyboard_sleep_counter_reset();
#endif
}
/**
 * @brief 发送按键报文的Hook
 * 
 * @param report 按键报文
 */
void hook_send_keyboard(report_keyboard_t * report)
{
    keyboard_conn_pass_enter_handler(report->keys, sizeof(report->keys));
}

/**
 * @brief BootMagic的Hook
 * 
 */
void hook_bootmagic()
{
    bool erase_bond = false;
    if(!bootmagic_scan_key(BOOTMAGIC_KEY_BOOT))
    {
        // Yes, 如果没有按下Space+U，那就不开机。
        #ifdef UART_SUPPORT
        if(uart_current_mode == UART_MODE_IDLE) // 插入了USB，则直接开机
        #endif
        {
        #ifndef KEYBOARD_DEBUG
            sleep_mode_enter(false);
        #endif
        }
    }
    if(bootmagic_scan_key(BOOTMAGIC_KEY_ERASE_BOND))
    {
        // 按下Space+E，删除所有的绑定信息
        erase_bond = true;
    }
    ble_services_init(erase_bond);
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
    
    err_code = app_timer_start(m_keyboard_wdt_timer_id, KEYBOARD_WDT_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    battery_timer_start();
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 * 
 * @param[in]   notice   Flash led to notice or not.
 */
void sleep_mode_enter(bool notice)
{
    uint32_t err_code;

    if(notice)led_notice(0x00, true);
    else led_notice(0x00, false);
    matrix_sleep_prepare();
#ifdef UART_SUPPORT
    uart_sleep_prepare();
#endif

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

static void wdt_evt(void){}

static void wdt_init(void)
{
    uint32_t err_code;
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_evt);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}

static void keyboard_wdt_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    nrf_drv_wdt_channel_feed(m_channel_id);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

#ifdef UART_SUPPORT

void uart_state_change(bool state)
{
    uint32_t err_code;
    if(state)
    {
        err_code = app_timer_stop(m_keyboard_sleep_timer_id);
        APP_ERROR_CHECK(err_code);
        led_powersave_mode(false);
        keyboard_switch_scan_mode(false);
    }
    else
    {
        err_code = app_timer_start(m_keyboard_sleep_timer_id, KEYBOARD_FREE_INTERVAL, NULL);
        APP_ERROR_CHECK(err_code);
        led_powersave_mode(true);
    }
}
#endif

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    
    // Initialize.
    // app_trace_init();
    timers_init();
    buttons_leds_init();
    ble_stack_init();
    scheduler_init();
    
    sd_power_dcdc_mode_set(true);
    
    // Initialize persistent storage module before the keyboard.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);
    
    keymap_init();
#ifdef UART_SUPPORT
    uart_set_evt_handler(&uart_state_change);
    uart_init();
#endif
	keyboard_init();
    services_init();

    // set driver after all module inited.
    host_set_driver(&driver);
    // Start execution.
    timers_start();
    wdt_init();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    
    led_change_handler(0x01, true);
    led_notice(0x07, 0x00);
    
#ifdef UART_SUPPORT
    uart_state_change(uart_current_mode != UART_MODE_IDLE);
#endif
    
    // Enter main loop.
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }
}
