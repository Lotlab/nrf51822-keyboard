/****
 *
 *  This is a black box.
 *
 ****/
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "ble_services.h"
#include "config.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_conn_params.h"
#include "ble_dis.h"
#include "ble_advertising.h"
#include "ble_advdata.h"

#include "softdevice_handler_appsh.h"
#include "device_manager.h"
#include "pstorage.h"

#include "bootloader_util.h"
#include "../tmk_core/common/bootloader.h"

#ifdef BLE_DFU_APP_SUPPORT
    #include "ble_dfu.h"
    #include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT

#define DEVICE_NAME PRODUCT      /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME MANUFACTURER /**< Manufacturer. Will be passed to Device Information Service. */

#define PNP_ID_VENDOR_ID_SOURCE 0x02  /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID VENDOR_ID       /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID PRODUCT_ID      /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION DEVICE_VER /**< Product Version. */

/*lint -emacro(524, MIN_CONN_INTERVAL) // Loss of precision */
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(12.5, UNIT_1_25_MS) /**< Minimum connection interval (7.5 ms) */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(60, UNIT_1_25_MS)   /**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY 6                                     /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(850, UNIT_10_MS)     /**< Connection supervisory timeout (430 ms). */

#define APP_ADV_FAST_INTERVAL 0x0028 /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL 0x0C80 /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */
#define APP_ADV_FAST_TIMEOUT 30      /**< The duration of the fast advertising period (in seconds). */
#define APP_ADV_SLOW_TIMEOUT 180     /**< The duration of the slow advertising period (in seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                                            /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND 1                                        /**< Perform bonding. */
#define SEC_PARAM_MITM 1                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_KEYBOARD_ONLY /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0                                         /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE 7                                /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE 16                               /**< Maximum encryption key size. */

#ifdef BLE_DFU_APP_SUPPORT
    #define DFU_REV_MAJOR 0x00                                  /** DFU Major revision number to be exposed. */
    #define DFU_REV_MINOR 0x00                                  /** DFU Minor revision number to be exposed. */
    #define DFU_REVISION ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR) /** DFU Revision number to be exposed. Combined of major and minor versions. */
    #define APP_SERVICE_HANDLE_START 0x000C                     /**< Handle of first application specific service when when service changed characteristic is present. */
    #define BLE_HANDLE_MAX 0xFFFF                               /**< Max handle value in BLE. */

    #define APP_FEATURE_NOT_SUPPORTED BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2 /**< Reply when unsupported features are requested. */

    STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT); /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif                                          // BLE_DFU_APP_SUPPORT


static dm_application_instance_t m_app_handle; /**< Application identifier allocated by device manager. */
dm_handle_t m_bonded_peer_handle;       /**< Device reference handle to the current bonded central. */
static uint16_t passkey_conn_handle;
bool passkey_required = false;

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

#ifdef BLE_DFU_APP_SUPPORT
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}, {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE}, {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};
static ble_dfu_t m_dfus; /**< Structure used to identify the DFU service. */
#else
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}, {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE}};
#endif // BLE_DFU_APP_SUPPORT

/** @} */

typedef enum {
    BLE_NO_ADV,             /**< No advertising running. */
    BLE_DIRECTED_ADV,       /**< Direct advertising to the latest central. */
    BLE_FAST_ADV_WHITELIST, /**< Advertising with whitelist. */
    BLE_FAST_ADV,           /**< Fast advertising running. */
    BLE_SLOW_ADV,           /**< Slow advertising running. */
    BLE_SLEEP,              /**< Go to system-off. */
} ble_advertising_mode_t;


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/** DFU ?? **/
#ifdef BLE_DFU_APP_SUPPORT
/**@brief ????
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as 
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context 
 *                     should be loaded.
 */
static void app_context_load(dm_handle_t const *p_handle)
{
    uint32_t err_code;
    static uint32_t context_data;
    dm_application_context_t context;

    context.len = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by 
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(500);
}

static void dfu_init(void)
{
    uint32_t err_code;
    ble_dfu_init_t dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler = dfu_app_on_dfu_evt;
    dfus_init.revision = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
}
/** @snippet [DFU BLE Reset prepare] */
#endif // BLE_DFU_APP_SUPPORT


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = NULL;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_KEYBOARD);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(-4);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    uint32_t err_code;
    ble_dis_init_t dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id = PNP_ID_VENDOR_ID;
    pnp_id.product_id = PNP_ID_PRODUCT_ID;
    pnp_id.product_version = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init_obj.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
    case BLE_ADV_EVT_IDLE:
        sleep_mode_enter();
        break;

    case BLE_ADV_EVT_WHITELIST_REQUEST:
    {
        ble_gap_whitelist_t whitelist;
        ble_gap_addr_t *p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
        ble_gap_irk_t *p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];

        whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
        whitelist.irk_count = BLE_GAP_WHITELIST_IRK_MAX_COUNT;
        whitelist.pp_addrs = p_whitelist_addr;
        whitelist.pp_irks = p_whitelist_irk;

        err_code = dm_whitelist_create(&m_app_handle, &whitelist);
        APP_ERROR_CHECK(err_code);

        err_code = ble_advertising_whitelist_reply(&whitelist);
        APP_ERROR_CHECK(err_code);
        break;
    }
    case BLE_ADV_EVT_PEER_ADDR_REQUEST:
    {
        ble_gap_addr_t peer_address;

        // Only Give peer address if we have a handle to the bonded peer.
        if (m_bonded_peer_handle.appl_id != DM_INVALID_ID)
        {

            err_code = dm_peer_addr_get(&m_bonded_peer_handle, &peer_address);
            APP_ERROR_CHECK(err_code);

            err_code = ble_advertising_peer_addr_reply(&peer_address);
            APP_ERROR_CHECK(err_code);
        }
        break;
    }
    default:
        break;
    }
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t *p_ble_evt)
{
    uint32_t err_code;
    ble_gatts_rw_authorize_reply_params_t auth_reply;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        break;

    case BLE_EVT_TX_COMPLETE:
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        m_conn_handle = BLE_CONN_HANDLE_INVALID;

        // Reset m_caps_on variable. Upon reconnect, the HID host will re-send the Output
        // report containing the Caps lock state.
        break;

    case BLE_EVT_USER_MEM_REQUEST:
        err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        if (p_ble_evt->evt.gatts_evt.params.authorize_request.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
        {
            if ((p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ) || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) || (p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
            {
                if (p_ble_evt->evt.gatts_evt.params.authorize_request.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                }
                else
                {
                    auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                }
                auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                err_code = sd_ble_gatts_rw_authorize_reply(m_conn_handle, &auth_reply);
                APP_ERROR_CHECK(err_code);
            }
        }
        break;

    case BLE_GAP_EVT_AUTH_KEY_REQUEST:
        passkey_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        passkey_required = true;
        break;

    case BLE_GATTC_EVT_TIMEOUT:
    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server and Client timeout events.
        err_code = sd_ble_gap_disconnect(m_conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    default:
        // No implementation needed.
        break;
    }
    //nrf_gpio_pin_toggle(LED_NUM);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t err_code;
    uint8_t adv_flags;
    ble_advdata_t advdata;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    adv_flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    advdata.name_type = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags = adv_flags;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids = m_adv_uuids;

    ble_adv_modes_config_t options =
        {
            BLE_ADV_WHITELIST_ENABLED,
            BLE_ADV_DIRECTED_ENABLED,
            BLE_ADV_DIRECTED_SLOW_DISABLED, 0, 0,
            BLE_ADV_FAST_ENABLED, APP_ADV_FAST_INTERVAL, APP_ADV_FAST_TIMEOUT,
            BLE_ADV_SLOW_ENABLED, APP_ADV_SLOW_INTERVAL, APP_ADV_SLOW_TIMEOUT};

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, ble_advertising_error_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const *p_handle,
                                           dm_event_t const *p_event,
                                           ret_code_t event_result)
{
    APP_ERROR_CHECK(event_result);
    switch (p_event->event_id)
    {
        case DM_EVT_DEVICE_CONTEXT_LOADED: // Fall through.
        case DM_EVT_SECURITY_SETUP_COMPLETE:
            m_bonded_peer_handle = (*p_handle);
            break;
#ifdef BLE_DFU_APP_SUPPORT
        case DM_EVT_LINK_SECURED:
            app_context_load(p_handle);
            break;
#endif
    }

    return NRF_SUCCESS;
}

/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t err_code;
    dm_init_param_t init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize peer device handle.
    err_code = dm_handle_initialize(&m_bonded_peer_handle);
    APP_ERROR_CHECK(err_code);

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond = SEC_PARAM_BOND;
    register_param.sec_param.mitm = SEC_PARAM_MITM;
    register_param.sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler = device_manager_evt_handler;
    register_param.service_type = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}

// see dfu_app_handler.c for more information.
void bootloader_jump(void)
{
    reset_prepare();

    uint32_t err_code = sd_power_gpregret_set(BOOTLOADER_DFU_START);
    APP_ERROR_CHECK(err_code);

    err_code = sd_softdevice_disable();
    APP_ERROR_CHECK(err_code);

    err_code = sd_softdevice_vector_table_base_set(NRF_UICR->BOOTLOADERADDR);
    APP_ERROR_CHECK(err_code);

    NVIC_ClearPendingIRQ(SWI2_IRQn);
    
	      uint32_t interrupt_setting_mask;
    uint32_t irq;

    // Fetch the current interrupt settings.
    interrupt_setting_mask = NVIC->ISER[0];

    // Loop from interrupt 0 for disabling of all interrupts.
    for (irq = 0; irq < 32; irq++)
    {
        if (interrupt_setting_mask & (0x01 << irq))
        {
            // The interrupt was enabled, hence disable it.
            NVIC_DisableIRQ((IRQn_Type)irq);
        }
    }
	
    bootloader_util_app_start(NRF_UICR->BOOTLOADERADDR);
}

void ble_services_init(bool erase_bond)
{
    device_manager_init(erase_bond);
    gap_params_init();
    advertising_init();
    conn_params_init();
    
    dis_init();
#ifdef BLE_DFU_APP_SUPPORT
    dfu_init();
#endif // BLE_DFU_APP_SUPPORT
}

void ble_services_evt_dispatch(ble_evt_t *p_ble_evt)
{
#ifdef BLE_DFU_APP_SUPPORT /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
#endif // BLE_DFU_APP_SUPPORT
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
}

void auth_key_reply(uint8_t * passkey)
{
    uint32_t err_code = sd_ble_gap_auth_key_reply(passkey_conn_handle, BLE_GAP_AUTH_KEY_TYPE_PASSKEY, passkey);
    APP_ERROR_CHECK(err_code);
    passkey_required = false;
}

bool auth_key_reqired()
{
    return passkey_required;
}
