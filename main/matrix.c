#include <stdint.h>
#include <stdbool.h>

#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "print.h"
#include "debug.h"
#include "util.h"
#include "matrix.h"
#include "keyboard_matrix.h"
#include "wait.h"

// 由于垃圾Keil的Armcc不支持-I附加include，所以这两个东西在项目设置那里定义。
// #define MATRIX_COLS    14
// #define MATRIX_ROWS    8

/** 行IO */
static const uint8_t row_pin_array[MATRIX_ROWS] = {21,22,23,24,25,26,27,29};
/** 列IO */
static const uint8_t column_pin_array[MATRIX_COLS] = {3,4,5,6,7,15,14,10,9,8,2,0,30,28};

#ifndef DEBOUNCE
#   define DEBOUNCE	5
#endif
static uint8_t debouncing = DEBOUNCE;

/* matrix state(1:on, 0:off) */
static matrix_row_t matrix[MATRIX_ROWS];
static matrix_row_t matrix_debouncing[MATRIX_ROWS];

static matrix_row_t read_cols(void);
static void select_row(uint8_t row);
static void unselect_rows(void);

/** 初始化键盘阵列 */
void matrix_init(void)
{
    for (uint_fast8_t i = MATRIX_ROWS; i--;)
    {
        nrf_gpio_cfg_output((uint32_t)row_pin_array[i]);
        NRF_GPIO->PIN_CNF[(uint32_t)row_pin_array[i]] |= 0x400; //Set pin to be "Disconnected 0 and standard 1"
        nrf_gpio_pin_clear((uint32_t)row_pin_array[i]);         //Set pin to low
    }
    for (uint_fast8_t i = MATRIX_COLS; i--;)
    {
        nrf_gpio_cfg_input((uint32_t)column_pin_array[i], NRF_GPIO_PIN_PULLDOWN);
    }
}
/** ???? */
static matrix_row_t read_cols(void)
{
    uint16_t result = 0;

    for (uint_fast8_t c = 0; c < MATRIX_COLS; c++)
    {
        if (nrf_gpio_pin_read((uint32_t)column_pin_array[c]))
            result |= 1 << c;
    }

    return result;
}

static void select_row(uint8_t row)
{
	  nrf_gpio_pin_set((uint32_t)row_pin_array[row]);
}

static void unselect_rows(void)
{
    for (uint_fast8_t i = 0; i < MATRIX_ROWS; i++)
    {
        nrf_gpio_pin_clear((uint32_t)row_pin_array[i]);
    }
}

static inline void delay_30ns(void)
{
#ifdef __GNUC__
#define __nop() __asm("NOP")
#endif
    for(int i=0; i<6; i++) {
        __nop();
    } 
}

uint8_t matrix_scan(void)
{
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        select_row(i);
#ifdef HYBRID_MATRIX
        init_cols();
#endif
        delay_30ns();  // without this wait read unstable value.
        matrix_row_t cols = read_cols();
        if (matrix_debouncing[i] != cols) {
            matrix_debouncing[i] = cols;
            if (debouncing) {
                debug("bounce!: "); debug_hex(debouncing); debug("\n");
            }
            debouncing = DEBOUNCE;
        }
        unselect_rows();
    }

    if (debouncing) {
        if (--debouncing) {
            wait_ms(1);
        } else {
            for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
                matrix[i] = matrix_debouncing[i];
            }
        }
    }

    return 1;
}

bool matrix_is_modified(void)
{
    if (debouncing) return false;
    return true;
}


inline
matrix_row_t matrix_get_row(uint8_t row)
{
    return matrix[row];
}

uint8_t matrix_key_count(void)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        count += bitpop16(matrix[i]);
    }
    return count;
}

void matrix_sleep_prepare(void)
{
    // 这里监听所有按键作为唤醒按键，所以真正的唤醒判断应该在main的初始化过程中
    for (uint8_t i = 0; i < MATRIX_COLS; i++)
        nrf_gpio_pin_set((uint32_t)row_pin_array[i]);
    for (uint8_t i = 0; i < MATRIX_ROWS; i++)
        nrf_gpio_cfg_sense_input((uint32_t)column_pin_array[i], NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_HIGH);
}

