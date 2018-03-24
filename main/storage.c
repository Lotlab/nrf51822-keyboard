#include <stdint.h>
#include "eeconfig.h"
#include "pstorage.h"
#include "app_error.h"

bool realIsInit = false;
bool fakeIsInit = true;

pstorage_handle_t       pstorage_base_block_id;
pstorage_handle_t       block_handle;

void config_pstorage_init(void);
void config_write(void);
void config_read(void);
void config_update(void);

static uint8_t config_buffer[8] __attribute__ ((aligned (4))) = {EECONFIG_MAGIC_NUMBER>>8, EECONFIG_MAGIC_NUMBER % 0x100 , 0,0,0,0,0,0}; 

static void eeconfig_set_default()
{
    config_buffer[0] = EECONFIG_MAGIC_NUMBER >> 8;
    config_buffer[1] = EECONFIG_MAGIC_NUMBER % 0x100;
    config_buffer[*EECONFIG_DEBUG] = 0;
    config_buffer[*EECONFIG_DEFAULT_LAYER] = 0;
    config_buffer[*EECONFIG_KEYMAP] = 0;
    config_buffer[*EECONFIG_MOUSEKEY_ACCEL] = 0;
#ifdef BACKLIGHT_ENABLE
    config_buffer[*EECONFIG_BACKLIGHT] = 0;
#endif
}
/*
    well, 让我解释一下发生了什么。此模块的pStorage需要在DeviceManager初始化之后才能初始化，
  然后DeviceManager初始化又需要BootMagic来选择是否删除绑定信息。在不更改tmk_core的相关代
  码的情况下，只能这么干：
  1. 在开始的时候伪造初始化状态，使得BootMagic模块认为此模块已经被初始化了，就不再初始化。
  2. 在BootMagic的自定义Hook的最后，手工调用eeconfig_init初始化此模块。
  3. 如果BootMagic调用了eeconfig_init设置默认数据，那么设置操作会变为初始化，接着在下一次
     手工调用的时候设置为默认数据。
*/
bool eeconfig_is_enabled(void)
{
    return fakeIsInit || realIsInit;
}

void eeconfig_init(void)
{
    if(realIsInit)
    {
        eeconfig_set_default();
        config_update();
    }
    else
    {
        config_pstorage_init();
        realIsInit = true;
    }
    fakeIsInit = false;
}

void eeconfig_enable(void)
{
}

void eeconfig_disable(void)
{ 
}

uint8_t eeconfig_read_debug(void)
{
    return config_buffer[*EECONFIG_DEBUG];
}

void eeconfig_write_debug(uint8_t val)
{
    config_buffer[*EECONFIG_DEBUG] = val;    
    config_update();
}

uint8_t eeconfig_read_default_layer(void)
{
    return config_buffer[*EECONFIG_DEFAULT_LAYER];
}

void eeconfig_write_default_layer(uint8_t val)
{
    config_buffer[*EECONFIG_DEFAULT_LAYER] = val;    
    config_update();
}

uint8_t eeconfig_read_keymap(void)
{
    return config_buffer[*EECONFIG_KEYMAP];
}
void eeconfig_write_keymap(uint8_t val)
{
    config_buffer[*EECONFIG_KEYMAP] = val;
    config_update();
}

#ifdef BACKLIGHT_ENABLE
uint8_t eeconfig_read_backlight(void)
{
    return config_buffer[*EECONFIG_BACKLIGHT];
}
void eeconfig_write_backlight(uint8_t val)
{
    config_buffer[*EECONFIG_BACKLIGHT] = val;
    config_update();
}
#endif


static void config_pstorage_callback_handler(pstorage_handle_t *p_handle, uint8_t op_code, uint32_t result, uint8_t *p_data, uint32_t data_len)
{
    switch(op_code)
    {
        case PSTORAGE_LOAD_OP_CODE:
           if (result == NRF_SUCCESS)
           {
               // Store operation successful.
           }
           else
           {
               // Store operation failed.
           }
           // Source memory can now be reused or freed.
           break;       
        case PSTORAGE_UPDATE_OP_CODE:
           if (result == NRF_SUCCESS)
           {
               // Update operation successful.
           }
           else
           {
               // Update operation failed.
           }
           break;
       case PSTORAGE_CLEAR_OP_CODE:
           if (result == NRF_SUCCESS)
           {
               // Clear operation successful.
           }
           else
           {
               // Clear operation failed.
           }
           break;
    }
}

static void config_pstorage_init(void)
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
    
    config_read();
    if(config_buffer[0] != EECONFIG_MAGIC_NUMBER>>8 || config_buffer[1] != EECONFIG_MAGIC_NUMBER % 0x100)
    {
        eeconfig_set_default();
        config_write();
    }
}

static void config_pstorage_write(uint8_t addr, uint8_t* data, uint8_t len)
{
    uint32_t err_code = pstorage_store(&block_handle, data, len, addr);
    APP_ERROR_CHECK(err_code);
}

static void config_pstorage_read(uint8_t addr, uint8_t* data, uint8_t len)
{
    uint32_t err_code = pstorage_load(data, &block_handle, len, addr);
    APP_ERROR_CHECK(err_code);
}

static void config_pstorage_update(uint8_t addr, uint8_t* data, uint8_t len)
{
    uint32_t err_code = pstorage_update(&block_handle, data, len, addr);
    APP_ERROR_CHECK(err_code);
}

static void config_update()
{
    config_pstorage_update(0,config_buffer,sizeof(config_buffer));
}
static void config_read()
{
    config_pstorage_read(0,config_buffer,sizeof(config_buffer));
}
static void config_write()
{
    config_pstorage_write(0,config_buffer,sizeof(config_buffer));
}
