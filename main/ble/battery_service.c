/**
 * @brief 电量测量服务
 * 
 * @file battery_service.c
 * @author Jim Jiang
 * @date 2018-05-13
 */
#include "battery_service.h"
#include "keyboard_conf.h"
#include "main.h"
#include <string.h>

#ifdef KEYBOARD_ADC

#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"
#include "keyboard_led.h"
#include "nrf_adc.h"
#include "softdevice_handler_appsh.h"

APP_TIMER_DEF(m_battery_timer_id);

static ble_bas_t m_bas; /**< Structure used to identify the battery service. */

uint16_t adc_result_queue[ADC_RESULT_QUEUE_SIZE]; /**中值滤波**/
uint8_t adc_result_queue_index;
uint32_t currVot; /**< Current Vottage of battery. */
bool slowMode = false;

/**
 * @brief 初始化电量服务(BAS)
 * 
 */
static void bas_init(void)
{
    uint32_t err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref = NULL;
    bas_init_obj.initial_batt_level = 100;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_report_read_perm);

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief 初始化电量测量ADC
 * 
 */
static void battery_sensor_init(void)
{
    nrf_adc_config_t nrf_adc_config = { NRF_ADC_CONFIG_RES_10BIT, // 10Bit 精度
        NRF_ADC_CONFIG_SCALING_INPUT_FULL_SCALE, // 完整输入
        NRF_ADC_CONFIG_REF_VBG }; // 内置 1.2V 基准

    // Initialize and configure ADC
    nrf_adc_configure((nrf_adc_config_t*)&nrf_adc_config);
    nrf_adc_input_select(KEYBOARD_ADC);
    nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
    NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
    NVIC_EnableIRQ(ADC_IRQn);
}

/**@brief 电量测量计时器溢出处理函数
 *
 * @details 当电量状态需要测量时调用此函数
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void* p_context)
{
    UNUSED_PARAMETER(p_context);
    nrf_adc_start();
}

/**
 * @brief 初始化电量计时器
 * 
 */
static void battery_timer_init(void)
{
    uint32_t err_code;

    // Create battery timer.
    err_code = app_timer_create(&m_battery_timer_id,
        APP_TIMER_MODE_REPEATED,
        battery_level_meas_timeout_handler);

    APP_ERROR_CHECK(err_code);
}

/**
 * @brief 初始化电量服务
 * 
 */
void battery_service_init(void)
{
    battery_sensor_init();
    bas_init();
    battery_timer_init();
}

/**
 * @brief 电压转换到电量
 * 
 * @param voltage 电压
 * @return uint8_t 电量百分比
 */
static uint8_t bas_vot2lvl(uint16_t voltage)
{
    if (voltage > 4200)
        return 100;
    else if (voltage > 4000)
        return 90 + (voltage - 4000) / 20;
    else if (voltage > 3600)
        return 10 + (voltage - 3600) / 5;
    else if (voltage > 3200)
        return (voltage - 3200) / 40;
    else
        return 0;
}

/**
 * @brief 上传电量数据
 * 
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t battery_level;

    battery_level = bas_vot2lvl(currVot);
    // 设置低电量指示灯
    led_set_bit(LED_BIT_LOW_POWER, battery_level <= 10);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE) && (err_code != BLE_ERROR_NO_TX_BUFFERS) && (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
        APP_ERROR_HANDLER(err_code);
    }
}

/**
 * @brief 启动电量计时器
 * 
 */
void battery_timer_start(void)
{
    uint32_t err_code;

    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL_FAST, NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief ADC结果转换到实际电压
 * 
 * @param adcResult adc结果
 * @return uint16_t 电压(mv)
 */
static uint16_t adc2vottage(int32_t adcResult)
{
    // Vmes = Vreal * 2.2M / (10M + 2.2M)
    // result = Vmes / Vref * BATTERY_ADC_DIV
    //
    // Vreal = Vmes * 12.2 / 2.2
    //       = result / BATTERY_ADC_DIV * Vref * 12.2 / 2.2
    //       = result * Vref / 2^10 * 122 / 22
    //      ~= result * Vref * 11 / 2^11
    // 22/122约等于2/11
    return (uint32_t)(adcResult * ADC_REF_VOLTAGE_IN_MILLIVOLTS * 11 >> 11);
}

/**
 * @brief 对获取到的电压数值进行预处理
 * 
 * @return uint16_t 
 */
static uint16_t adc_result_calc()
{
    // 中值滤波
    uint16_t min = 0xFFFF, max = 0x0000, curr;
    uint32_t total = 0;
    for (int i = 0; i < ADC_RESULT_QUEUE_SIZE; i++) {
        curr = adc_result_queue[i];
        total += curr;
        if (curr > max)
            max = curr;
        if (curr < min)
            min = curr;
    }
    total -= max;
    total -= min;

    return total / (ADC_RESULT_QUEUE_SIZE - 2);
}

/**
 * @brief 电量测量切换至慢速测量模式
 * 
 */
static void ADC_switch_to_slow_mode()
{
    uint32_t err_code = app_timer_stop(m_battery_timer_id);
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL_SLOW, NULL);
    APP_ERROR_CHECK(err_code);

    slowMode = true;
}

/**
 * @brief ADC电量测量回调
 * 
 * @param p_event_data 
 * @param event_size 
 */
static void ADC_appsh_mes_evt_handler(void* p_event_data, uint16_t event_size)
{
    UNUSED_PARAMETER(p_event_data);
    UNUSED_PARAMETER(event_size);

    if (adc_result_queue_index >= ADC_RESULT_QUEUE_SIZE) {
        adc_result_queue_index = 0;
    }
    adc_result_queue[adc_result_queue_index++] = nrf_adc_result_get();

    if (slowMode) {
        // 数据已经稳定，可以直接计算上传
        currVot = adc2vottage(adc_result_calc());
        battery_level_update();
    } else {
        // 数据尚未稳定，需要稳定后才延长测量间隔并上传
        if (currVot == adc2vottage(adc_result_calc()) && currVot > 0) {
            ADC_switch_to_slow_mode();
            battery_level_update();
        }
    }
}

/**
 * @brief ADC测量完毕
 * 
 */
void ADC_IRQHandler()
{
    nrf_adc_conversion_event_clean();
    nrf_adc_stop();
    app_sched_event_put(NULL, 0, ADC_appsh_mes_evt_handler);
}

/**
 * @brief BLE电量服务事件处理
 * 
 * @param p_ble_evt 
 */
void battery_service_ble_evt(ble_evt_t* p_ble_evt)
{
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
}

#else
void battery_service_ble_evt(ble_evt_t* p_ble_evt) {}
void battery_timer_start(void) {}
void battery_service_init(void) {}
#endif
