#include <stdint.h>
#include "host.h"
#include "host_driver.h"

#include "ble_hid_service.h"

uint8_t keyboard_leds(void);
void send_keyboard(report_keyboard_t * report);
void send_mouse(report_mouse_t * report);
void send_system(uint16_t data);
void send_consumer(uint16_t data);

host_driver_t driver = {
        keyboard_leds,
        send_keyboard,
        send_mouse,
        send_system,
        send_consumer
};

static uint8_t keyboard_leds()
{
    return led_val;
}
static void send_keyboard(report_keyboard_t * report)
{
    hids_keys_send(KEYBOARD_REPORT_SIZE, report->raw);
}
static void send_mouse(report_mouse_t * report)
{
    // unsupport, and will not support in future.
}
static void send_system(uint16_t data)
{
    // may support in future.
}
static void send_consumer(uint16_t data)
{
    // may support in future.
}
