#ifndef __KEYMAP_STORAGE__
#define __KEYMAP_STORAGE__

#include <stdint.h>
extern uint8_t keymap_data[1024];

void keymap_init(void);
void keymap_write(void);
void keymap_read(void);

#endif
