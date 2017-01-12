#ifndef _KEYMAP_
#define _KEYMAP_

#include "keymap_common.h"

#define MATRIX_COLS    14 // !< Number of columns in the keyboard matrix
#define MATRIX_ROWS    8  // !< Number of rows in the keyboard matrix

static const uint8_t row_pin_array[MATRIX_ROWS] = {19,18,17,16,15,14,13,12};
static const uint8_t column_pin_array[MATRIX_COLS] = {7,5,4,3,2,21,22,28,29,30,8,9,10,11};

static const uint8_t wakeup_button_row_index = 0;		
static const uint8_t wakeup_button_column_index = 0;	
//wakeup button is default_matrix_lookup[0][0] = 0x29 and it's ESC key

static const uint8_t keymaps[2][MATRIX_COLS][MATRIX_ROWS] = {
    /* 0: qwerty */
    KEYMAP( \
           ESC, F1, F2,  F3,  F4,  F5,  F6,  F7,  F8,  F9,  F10,  NLCK, PSCR, SLCK, PAUS, \
                1,   2,   3,   4,   5,   6,   7,   8,   9,    0,  MINS,  EQL, BSPC, HOME, \
           TAB, Q,   W,   E,   R,   T,   Y,   U,   I,   O,    P,  LBRC, RBRC, BSLS, PGUP, \
           CAPS,A,   S,   D,   F,   G,   H,   J,   K,   L, SCLN,  QUOT,    ENT,   PGDOWN, \
           LSFT,  Z,   X,   C,   V,   B,   N,   M,   COMM,  DOT,  SLSH, RSFT,   UP,  END, \
           LCTL,FN0,LALT, GRV,         SPC,          RALT,  INS,   DEL, LEFT, DOWN, RIGHT ),
    /* 1: Fn */       
    KEYMAP( \
           NO,F11, F12,  NO,  NO,  NO,  NO,  NO,  NO,  NO,   NO,    NO,   NO,   NO, FN15, \
               NO,  NO,  NO,  NO,  NO,  NO,  P7,  P8,  P9, PAST,    NO,   NO,   NO,   NO, \
           NO, NO,  NO,  NO,  NO,  NO,  NO,  P4,  P5,  P6, PMNS,    NO,   NO,   NO,   NO, \
           NO, NO,  NO,  NO,  NO,  NO,  NO,  P1,  P2,  P3, PPLS,    NO,     PENT,     NO, \
           NO,   NO,  NO,  NO,  NO,  NO,  NO,  P0,     NO, PDOT,  PSLS,   NO,   NO,   NO, \
           RCTL,FN0,  NO,  NO,         NO,             NO,   NO,   NO,    NO,   NO,   NO ),       
};

#endif