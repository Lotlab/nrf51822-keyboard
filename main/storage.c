#include <stdint.h>
#include "eeconfig.h"
#include "pstorage.h"
#include "app_error.h"

bool isInit = false;

pstorage_handle_t       pstorage_base_block_id;
pstorage_handle_t       block_handle;

uint8_t config_pstorage_read(uint8_t addr);
uint16_t config_pstorage_read_word(uint16_t addr);

void config_pstorage_write(uint8_t addr, uint8_t data);
void config_pstorage_write_word(uint16_t addr, uint16_t data);
void config_pstorage_init(void);

void eeconfig_set_default()
{
    config_pstorage_write_word(0,          EECONFIG_MAGIC_NUMBER);
    config_pstorage_write(*EECONFIG_DEBUG,          0);
    config_pstorage_write(*EECONFIG_DEFAULT_LAYER,  0);
    config_pstorage_write(*EECONFIG_KEYMAP,         0);
    config_pstorage_write(*EECONFIG_MOUSEKEY_ACCEL, 0);
#ifdef BACKLIGHT_ENABLE
    config_pstorage_write(EECONFIG_BACKLIGHT,      0);
#endif

}


bool eeconfig_is_enabled(void)
{
    return isInit;
}

void eeconfig_init(void)
{
    if(isInit)
    {
        eeconfig_set_default();
    }
    else
    {
        config_pstorage_init();
        isInit = true;
    }
}

void eeconfig_enable(void)
{
}

void eeconfig_disable(void)
{ 
}

uint8_t eeconfig_read_debug(void)
{
    return config_pstorage_read(*EECONFIG_DEBUG);
}

void eeconfig_write_debug(uint8_t val)
{
    config_pstorage_write(*EECONFIG_DEBUG, val);    
}

uint8_t eeconfig_read_default_layer(void)
{
    return config_pstorage_read(*EECONFIG_DEFAULT_LAYER);
}

void eeconfig_write_default_layer(uint8_t val)
{
    config_pstorage_write(*EECONFIG_DEFAULT_LAYER, val);    
}

uint8_t eeconfig_read_keymap(void)
{
    return config_pstorage_read(*EECONFIG_KEYMAP);
}
void eeconfig_write_keymap(uint8_t val)
{
    config_pstorage_write(*EECONFIG_KEYMAP, val);
}

#ifdef BACKLIGHT_ENABLE
uint8_t eeconfig_read_backlight(void)
{
    return config_pstorage_read(*EECONFIG_BACKLIGHT);
}
void eeconfig_write_backlight(uint8_t val)
{
    config_pstorage_write(*EECONFIG_BACKLIGHT, val);
}
#endif


void config_pstorage_callback_handler(pstorage_handle_t *p_handle, uint8_t op_code, uint32_t result, uint8_t *p_data, uint32_t data_len)
{
    switch(op_code)  
    {        
       case PSTORAGE_UPDATE_OP_CODE:  
           if (result == NRF_SUCCESS)  
           {  
               //my_flag = 1; //?flash update???????? Main???????flash???  
           }  
           else  
           {  
               // Update operation failed.  
           }  
           break;  
    }  
}

void config_pstorage_init(void)
{
    //pstorage init in device manager, so do not init here 
    
    //registe some parames
    pstorage_module_param_t param;
    uint32_t                err_code;
          
    param.block_size  = 0x10; // 4 byte is enough, but pstorage minimum block size is 0x10
    param.block_count = 1;
    param.cb          = config_pstorage_callback_handler;
        
    err_code = pstorage_register(&param, &pstorage_base_block_id);
    APP_ERROR_CHECK(err_code);
    
    err_code = pstorage_block_identifier_get(&pstorage_base_block_id, 0, &block_handle);
    APP_ERROR_CHECK(err_code);
}

void config_pstorage_write(uint8_t addr, uint8_t data)
{
    uint32_t err_code = pstorage_store(&block_handle, &data, 1, addr);
    APP_ERROR_CHECK(err_code);
}

void config_pstorage_write_word(uint16_t addr, uint16_t data)
{
    uint32_t err_code = pstorage_store(&block_handle, (uint8_t *)&data, sizeof(data), addr);
    APP_ERROR_CHECK(err_code);
}

uint8_t config_pstorage_read(uint8_t addr)
{
    uint8_t data;

    uint32_t err_code = pstorage_load(&data, &block_handle, 1, addr);
    APP_ERROR_CHECK(err_code);

    return data;
}

uint16_t config_pstorage_read_word(uint16_t addr)
{
    uint16_t data;

    uint32_t err_code = pstorage_load((uint8_t *)&data, &block_handle, 2, addr);
    APP_ERROR_CHECK(err_code);

    return data;
}


