#ifndef _KEYMAP_
#define _KEYMAP_

/** Keymap 配置文件
 * 
 *  用于配置当前的键盘按键序列
 */

#include "keymap_common.h"

#define MATRIX_COLS    14 // !< 键盘阵列的列数
#define MATRIX_ROWS    8  // !< 键盘阵列的行数

/** 键盘阵列行IO */
static const uint8_t row_pin_array[MATRIX_ROWS] = {21,22,23,24,25,26,27,29};
/** 键盘阵列列IO */
static const uint8_t column_pin_array[MATRIX_COLS] = {3,4,5,6,7,15,14,10,9,8,2,0,30,28};

/** 唤醒按钮所在的行 */
static const uint8_t wakeup_button_row_index = 0;	
/** 唤醒按钮所在的列 */	
static const uint8_t wakeup_button_column_index = 0;	
// 第0行第0列是ESC键


#endif
