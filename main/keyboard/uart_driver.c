/**
 * @brief UART串口通信协议
 * 
 * @file uart_driver.c
 * @author Jim Jiang
 * @date 2018-05-18
 */
#include <string.h>
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

uint8_t     rx_buf[64];
uint8_t     tx_buf[64];

typedef enum {
    STATE_IDLE, // 接收完毕
    STATE_DATA, // 正在接收数据
} state;

state current;
uint8_t len,pos;
uint8_t recv_buff[64];
//串口状态
state uart_state = STATE_DATA;
bool volatile ping_state;

APP_TIMER_DEF(uart_check_timer);
void uart_task(void *p_context);

#define UART_CHECK_INTERVAL APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

/**
 * @brief UART应答
 * 
 * @param success 响应是否成功
 */
void uart_ack(bool success)
{
    uart_send_packet(success ? PACKET_ACK : PACKET_FAIL, NULL, 0);
}

/**
 * @brief UART包处理
 * 
 */
void uart_data_handler()
{
    switch((packet_type)recv_buff[0])
    {
        case PACKET_PING:
            if(len - 1 != 0)
            {
                uart_ack(false);
                return;
            }
            uart_ack(true);
            ping_state = true;
            break;
        case PACKET_KEYMAP:
        {
            uint8_t checksum = 0x00;
            if(len - 1 != 62)
            {
                uart_ack(false);
                return;
            }
            
            for(int i=2;i<63;i++)
            {
                checksum += recv_buff[i];
            }
            if(checksum != recv_buff[63])
            {
                uart_ack(false);
            }
            else
            {
                uint16_t id = recv_buff[2];
                memcpy(&keymap_data[id * 60], &recv_buff[3], 60);
                if(id >= 16)
                {
                    keymap_write();
                }
                uart_ack(true);
            }
        }
        break;
        case PACKET_LED:
            if(len - 1 != 1)return;
            led_val = recv_buff[1];
            led_change_handler(led_val, true);
            break;
        case PACKET_CHARGING:
            if(len - 1 != 1)return;
            if(recv_buff[1] == 0x00)
            {
                // full
            }
            else
            {
                // charging
            }
            break;
        default:
            break;
    }
}
/**
 * @brief UART接收事件
 * 
 */
void uart_on_recv()
{
    switch(current)
    {
        case STATE_IDLE:
            app_uart_get(&len);
            pos = 0;
            if(len != 0)
                current = STATE_DATA;
            else
                uart_data_handler();
        break;
        case STATE_DATA:
        {
            while(app_uart_get(&recv_buff[pos]) == NRF_SUCCESS)
            {
                pos++;
            }
            if(pos >= len)
            {
                current = STATE_IDLE;
                app_uart_flush();
                uart_data_handler();
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
void uart_evt_handler(app_uart_evt_t * p_event)
{
    switch(p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            uart_on_recv();
        break;
        
        case APP_UART_TX_EMPTY:
        break;
        default:
            break;
    }
}
/**
 * @brief 初始化串口
 * 
 */
void uart_init_hardware()
{
    uint32_t err_code;
    app_uart_buffers_t buffers;                                                                
    
    buffers.rx_buf      = rx_buf;
    buffers.rx_buf_size = sizeof (rx_buf);
    buffers.tx_buf      = tx_buf;
    buffers.tx_buf_size = sizeof (tx_buf);
    
    const app_uart_comm_params_t config = {
    .baud_rate = UART_BAUDRATE,
    .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
    .rx_pin_no = UART_RXD,
    .tx_pin_no = UART_TXD,
    .use_parity = false
    };
    
    err_code = app_uart_init(&config, &buffers, uart_evt_handler, APP_IRQ_PRIORITY_LOW);   
    APP_ERROR_CHECK(err_code);
}

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
    
    uart_init_hardware();
}


/**
 * @brief 发送数据
 * 
 * @param data 
 * @param len 
 */
void uart_send(uint8_t * data, uint8_t len)
{
    while(len--){
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
void uart_send_packet(packet_type type, uint8_t * data, uint8_t len)
{
    if(uart_state == STATE_DATA)
    {
        app_uart_put(len + 1);
        app_uart_put(type);
        uart_send(data, len);
    }
}

/**
 * @brief 取消串口使能
 * 
 */
void uart_deinit()
{
    app_uart_close();
}

void uart_to_idle()
{
    uart_deinit();
    nrf_gpio_cfg_input(UART_RXD, NRF_GPIO_PIN_PULLDOWN);
}

void uart_task(void *p_context)
{
    (void)p_context;
    switch(uart_state)
    {
        case STATE_IDLE:
            if(nrf_gpio_pin_read(UART_RXD)) // 状态改变了
            {
                uart_init_hardware();
                uart_state = STATE_DATA;
            }
            break;
        case STATE_DATA:
            if(!ping_state) // 没有收到ping包
            {
                uart_state = STATE_IDLE;
                uart_to_idle();
            }
            ping_state = false;
        break;
    }
    
}

