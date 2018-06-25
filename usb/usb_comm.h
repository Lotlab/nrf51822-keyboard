#ifndef __USB_COMM__
#define __USB_COMM__

#include <stdint.h>
#include <stdbool.h>
extern bool usb_sleep;
void KeyboardGenericUpload(uint8_t * packet, uint8_t len);
void KeyboardExtraUpload(uint8_t * packet, uint8_t len);
void ResponseConfigurePacket(uint8_t * packet, uint8_t len);

#endif // __USB_COMM__
