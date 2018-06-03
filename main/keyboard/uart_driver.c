/**
 * @brief UART串口通信协议
 * 
 * @file uart_driver.c
 * @author Jim Jiang
 * @date 2018-05-18
 */
#include <string.h>
#include <config.h>

#ifdef UART_SUPPORT
#include "uart_driver.h"
#include "app_error.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "nrf_drv_uart.h"
#include "nrf_gpio.h"
#include "main.h"
#include "app_timer_appsh.h"
#include "app_scheduler.h"

#include "ble_hid_service.h"
#include "keyboard_conf.h"
#include "keyboard_led.h"
#include "keymap_storage.h"

#define UART_CHECK_INTERVAL APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
APP_TIMER_DEF(uart_check_timer);

uint8_t rx_buf[64];
uint8_t tx_buf[128];

uint8_t tx_resend_buf[64];

/**
 * @brief UART当前状态。
 * 
 */
uart_mode uart_current_mode;

/**
 * @brief Ping包状态。定时ping包来判断USB芯片是否工作正常。
 * 
 */
bool ping_state;

typedef enum
{
    STATE_IDLE, // 接收完毕
    STATE_DATA, // 正在接收数据
} state;
state current;

struct
{
    uint8_t len;
    uint8_t pos;
    uint8_t data_len;
    union {
        uint8_t raw[64];
        struct
        {
            uint8_t command;
            uint8_t data[63];
        };
    };
} recv;

void (*state_change_evt_handler)(bool);

void uart_set_evt_handler(void (*evt)(bool))
{
    state_change_evt_handler = evt;
}

void uart_state_change_invoke()
{
    (*state_change_evt_handler)((bool)uart_current_mode != UART_MODE_IDLE);
}

/**
 * @brief UART应答
 * 
 * @param success 响应是否成功
 */
void uart_ack(bool success)
{
    uart_send_packet(success ? PACKET_ACK : PACKET_FAIL, NULL, 0);
}

uint8_t checksum(uint8_t * data, uint8_t len)
{
    uint8_t checksum = 0x00;
    for (int i = 0; i < len; i++)
    {
        checksum += data[i];
    }
    return checksum;
}

/**
 * @brief 处理Keymap下发信息
 * 
 */
void uart_keymap()
{
    if (checksum(recv.data, 61) != recv.data[61])
    {
        uart_ack(false);
    }
    else
    {
        uint16_t id = recv.data[0];
        memcpy(&keymap_data[id * 60], &recv.data[1], 60);
        if (id >= 16)
        {
            keymap_write();
        }
        uart_ack(true);
    }
}

/**
 * @brief UART包处理
 * 
 */
void uart_data_handler()
{
    switch ((packet_type)recv.command)
    {
    case PACKET_USB_STATUS:
        if (uart_current_mode != UART_MODE_BLE_OVERRIDE)
            uart_current_mode = UART_MODE_USB;
    case PACKET_PING:
        // uart_ack(true);
    case PACKET_ACK:
        ping_state = true;
        break;

    case PACKET_KEYMAP:
        uart_keymap();
        break;
    case PACKET_LED:
        led_val = recv.data[0];
        led_change_handler(led_val, true);
        break;
    case PACKET_CHARGING:
        if (recv.data[0] == 0x00)
        { // full
        }
        else
        { // charging
        }
        break;
    case PACKET_FAIL:    
        uart_send_packet((packet_type)tx_resend_buf[0], &tx_resend_buf[2], tx_resend_buf[1]);
        break;
    default:
        break;
    }
}

/**
 * @brief UART包长度校验
 * 
 * @param type 包类型
 * @param len 包长度
 * @return true 长度校验通过
 * @return false 长度校验失败
 */
bool uart_packet_len_validator(packet_type type, uint8_t len)
{
    switch (type)
    {
    case PACKET_PING:
    case PACKET_USB_STATUS:
    case PACKET_FAIL:
    case PACKET_ACK:
        return len == 0;
    case PACKET_LED:
    case PACKET_CHARGING:
        return len == 1;
    case PACKET_KEYMAP:
        return len == 62;
    default:
        return false;
    }
}

/**
 * @brief UART接收事件
 * 
 */
void uart_on_recv()
{
    switch (current)
    {
    case STATE_IDLE:
        app_uart_get(&recv.len);
        recv.pos = 0;
        recv.data_len = recv.len - 1;
        current = STATE_DATA;
        break;
    case STATE_DATA:
    {
        while (app_uart_get(&recv.raw[recv.pos]) == NRF_SUCCESS)
            recv.pos++;

        if (recv.pos >= recv.len)
        {
            current = STATE_IDLE;

            if (uart_packet_len_validator((packet_type)recv.command, recv.data_len))
                uart_data_handler();
            else
                uart_ack(false);
        }
    }
    break;
    }
}

/**
 * @brief UART事件回调
 * 
 * @param p_event 
 */
void uart_evt_handler(app_uart_evt_t *p_event)
{
    switch (p_event->evt_type)
    {
    case APP_UART_DATA_READY:
        uart_on_recv();
        break;

    case APP_UART_TX_EMPTY:
        break;

    case APP_UART_FIFO_ERROR:
        app_uart_flush();
        break;
    default:
        break;
    }
}

/**
 * @brief UART休眠
 * 
 */
void uart_to_idle()
{
    app_uart_close();
    nrf_gpio_cfg_input(UART_RXD, NRF_GPIO_PIN_PULLDOWN);
    uart_current_mode = UART_MODE_IDLE;
}
/**
 * @brief 初始化串口
 * 
 */
void uart_init_hardware()
{
    uint32_t err_code;
    app_uart_buffers_t buffers;

    buffers.rx_buf = rx_buf;
    buffers.rx_buf_size = sizeof(rx_buf);
    buffers.tx_buf = tx_buf;
    buffers.tx_buf_size = sizeof(tx_buf);

    const app_uart_comm_params_t config = {
        .baud_rate = UART_BAUDRATE,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .rx_pin_no = UART_RXD,
        .tx_pin_no = UART_TXD,
        .use_parity = false};

    err_code = app_uart_init(&config, &buffers, uart_evt_handler, APP_IRQ_PRIORITY_LOW);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief UART定时任务。用于检测PING包是否正常上传，以获得USB芯片工作状态
 * 
 * @param p_context 
 */
void uart_task(void *p_context)
{
    (void)p_context;
    if (uart_current_mode != UART_MODE_IDLE)
    {
        if (!ping_state) // 没有收到ping包
        {
            uart_to_idle();
            uart_state_change_invoke();
        }
        ping_state = false;
    }
    else
    {
        if (nrf_gpio_pin_read(UART_RXD)) // 状态改变了
        {
            uart_init_hardware();
            uart_current_mode = UART_MODE_CHARGING;
            uart_state_change_invoke();
        }
    }
}

/**
 * @brief 初始化UART
 * 
 */
void uart_init()
{
    uint32_t err_code;

    // Create battery timer.
    err_code = app_timer_create(&uart_check_timer,
                                APP_TIMER_MODE_REPEATED,
                                uart_task);

    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(uart_check_timer, UART_CHECK_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    uart_to_idle();
    //uart_task(NULL);
    if (nrf_gpio_pin_read(UART_RXD)) // 状态改变了
    {
        uart_init_hardware();
        uart_current_mode = UART_MODE_CHARGING;
    }
}

/**
 * @brief 发送数据
 * 
 * @param data 
 * @param len 
 */
void uart_send(uint8_t *data, uint8_t len)
{
    while (len--)
    {
        app_uart_put(*(data++));
    }
}

/**
 * @brief 发送数据包
 * 
 * @param type 包类型
 * @param data 数据
 * @param len 长度
 */
void uart_send_packet(packet_type type, uint8_t *data, uint8_t len)
{
    if (uart_current_mode != UART_MODE_IDLE)
    {
        if(len == 0)
        {
            app_uart_put(1);
            app_uart_put(type);
        }
        else
        {
            app_uart_put(len + 2);
            app_uart_put(type);
            uart_send(data, len);
            app_uart_put(checksum(data, len));
        }
        
        tx_resend_buf[0] = type;
        tx_resend_buf[1] = len;
        memcpy(data, &tx_resend_buf[2], len);
    }
}

/**
 * @brief 取消串口使能
 * 
 */
void uart_sleep_prepare()
{
    uart_to_idle();
    nrf_gpio_cfg_sense_input(UART_RXD, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
}

/**
 * @brief 是否使用USB进行通讯
 * 
 * @return true 
 * @return false 
 */
bool uart_is_using_usb()
{
    return uart_current_mode == UART_MODE_USB;
}

/**
 * @brief 在蓝牙模式和USB模式中进行切换
 * 
 */
void uart_switch_mode()
{
    switch(uart_current_mode)
    {
        case UART_MODE_USB:
            uart_current_mode = UART_MODE_BLE_OVERRIDE;
            break;
        case UART_MODE_BLE_OVERRIDE:
            uart_current_mode = UART_MODE_USB;
            break;
        default:
            break;
    }
}

#endif
