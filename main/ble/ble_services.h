#ifndef __BLE_SERVICES__
#define __BLE_SERVICES__

#include "ble.h"
#include <stdbool.h>

#include "device_manager.h"
extern dm_handle_t m_bonded_peer_handle;

void ble_services_init(bool erase_bond);
void ble_services_evt_dispatch(ble_evt_t* p_ble_evt);
void auth_key_reply(uint8_t* passkey);
bool auth_key_reqired(void);

#endif
