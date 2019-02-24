#include "keymap_storage.h"
#include "app_error.h"
#include "keycode.h"
#include "keymap.h"
#include "pstorage.h"
#include <stdint.h>

uint8_t keymap_data[1024];
const uint8_t layer_size = MATRIX_ROWS * MATRIX_COLS;
const uint8_t layer_offset = 0x55;
const uint8_t fn_offset = 0x15;

#define KEYMAP_VALID (keymap_data[0] == 0x55)
bool storage_keymap_valid = false;

/* user keymaps should be defined somewhere */
extern const uint8_t keymaps[][MATRIX_ROWS][MATRIX_COLS];
extern const action_t fn_actions[];

static pstorage_handle_t pstorage_base_block_id;
static pstorage_handle_t block_handle;

uint8_t keymap_key_to_keycode(uint8_t layer, keypos_t key)
{
    if (layer >= 8 || key.col >= MATRIX_COLS || key.row >= MATRIX_ROWS)
        return KC_NO;
    if (KEYMAP_VALID)
        return keymap_data[layer_offset + layer * layer_size + key.row * MATRIX_COLS + key.col];
    else
        return keymaps[layer][key.row][key.col];
}

action_t keymap_fn_to_action(uint8_t keycode)
{
    if (KEYMAP_VALID) {
        uint8_t index = fn_offset + FN_INDEX(keycode) * 2;
        uint16_t action = ((uint16_t)keymap_data[index + 1] << 8) + keymap_data[index];
        return (action_t)action;
    } else
        return fn_actions[FN_INDEX(keycode)];
}

static void pstorage_callback_handler(pstorage_handle_t* p_handle, uint8_t op_code, uint32_t result, uint8_t* p_data, uint32_t data_len)
{
    switch (op_code) {
    case PSTORAGE_LOAD_OP_CODE:
        if (result == NRF_SUCCESS) {
            // Store operation successful.
        } else {
            // Store operation failed.
        }
        // Source memory can now be reused or freed.
        break;
    case PSTORAGE_UPDATE_OP_CODE:
        if (result == NRF_SUCCESS) {
            // Update operation successful.
        } else {
            // Update operation failed.
        }
        break;
    case PSTORAGE_CLEAR_OP_CODE:
        if (result == NRF_SUCCESS) {
            // Clear operation successful.
        } else {
            // Clear operation failed.
        }
        break;
    }
}

void keymap_init(void)
{
    //pstorage init in device manager, so do not init here

    //registe some parames
    pstorage_module_param_t param;
    uint32_t err_code;

    param.block_size = 0x400; // wow, amazing.
    param.block_count = 1;
    param.cb = pstorage_callback_handler;

    err_code = pstorage_register(&param, &pstorage_base_block_id);
    APP_ERROR_CHECK(err_code);

    err_code = pstorage_block_identifier_get(&pstorage_base_block_id, 0, &block_handle);
    APP_ERROR_CHECK(err_code);

    keymap_read();
}

void keymap_write()
{
    uint32_t err_code;
    if (storage_keymap_valid) {
        if (KEYMAP_VALID) {
            err_code = pstorage_update(&block_handle, keymap_data, sizeof(keymap_data), 0);
            APP_ERROR_CHECK(err_code);
        } else {
            err_code = pstorage_clear(&block_handle, sizeof(keymap_data));
            APP_ERROR_CHECK(err_code);

            storage_keymap_valid = false;
        }
    } else {
        if (KEYMAP_VALID) {
            err_code = pstorage_update(&block_handle, keymap_data, sizeof(keymap_data), 0);
            APP_ERROR_CHECK(err_code);

            storage_keymap_valid = true;
        }
    }
}

void keymap_read()
{
    uint32_t err_code = pstorage_load(keymap_data, &block_handle, sizeof(keymap_data), 0);
    APP_ERROR_CHECK(err_code);

    storage_keymap_valid = KEYMAP_VALID;
}
