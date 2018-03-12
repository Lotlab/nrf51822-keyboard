#include "keymap_common.h"

/** ??Keymap?? */
const uint8_t keymaps[2][MATRIX_COLS][MATRIX_ROWS] = {
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
         TRNS, F11, F12,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS, TRNS,  TRNS, TRNS, TRNS, FN15, \
              TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,  P7,  P8,  P9, PAST,  TRNS, TRNS, TRNS, TRNS, \
         TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,  P4,  P5,  P6, PMNS,  TRNS, TRNS, TRNS, TRNS, \
         TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,  P1,  P2,  P3, PPLS,  TRNS,     PENT,   TRNS, \
         TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,TRNS,  P0,   TRNS, PDOT,  PSLS,  TRNS, TRNS, TRNS, \
         RCTL,FN0, LGUI,TRNS,       TRNS,           TRNS, TRNS,  TRNS,  TRNS, TRNS, TRNS ),       
};

const action_t PROGMEM fn_actions[] = {};

