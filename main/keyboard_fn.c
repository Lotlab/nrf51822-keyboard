#include <stdint.h>
#include "keyboard_fn.h"
#include "main.h"

void action_function(keyrecord_t *record, uint8_t id, uint8_t opt)
{
    switch(id)
    {
        case POWER_SLEEP:
            sleep_mode_enter();
        break;
        case POWER_ON:
            
        break;
        case CLEAR_BONDAGE:
            
        break;
        
        case SWITCH_DEVICE:
            
        break;
        default:
            break;
    }
}
