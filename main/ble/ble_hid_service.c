#include "ble_hid_service.h"
#include <stdint.h>
#include <string.h>

#include "app_error.h"
#include "ble_hids.h"
#include "device_manager.h"

#include "ble_services.h"
#include "keycode.h"
#include "keymap_storage.h"
#include "main.h"
#include "report.h"

#define OUTPUT_REPORT_MAX_LEN 1 /**< Maximum length of Output Report. */
#define INPUT_REPORT_KEYS_INDEX 0 /**< Index of Input Report. */
#define INPUT_REPORT_KEYS_MAX_LEN 8 /**< Maximum length of the Input Report characteristic. */

#define KEYBOARD_OUTPUT_REPORT_INDEX 0
#define OUTPUT_REPORT_COUNT 1

#define KEYBOARD_INPUT_REPORT_INDEX 0
#define SYSTEM_INPUT_REPORT_INDEX 1
#define CONSUMER_INPUT_REPORT_INDEX 2
#define INPUT_REPORT_COUNT 3

#define MAX_BUFFER_ENTRIES 0x10 /**< Number of elements that can be enqueued */

#define BASE_USB_HID_SPEC_VERSION 0x0101 /**< Version number of base USB HID Specification implemented by this application. */

static uint8_t report_map_data[] = {
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xA1, 0x01, // Collection (Application)
    0x85, 0x01, //     Report Id 1
    0x05, 0x07, //     Usage Page (Key Codes)
    0x19, 0xe0, //     Usage Minimum (224)
    0x29, 0xe7, //     Usage Maximum (231)
    0x15, 0x00, //     Logical Minimum (0)
    0x25, 0x01, //     Logical Maximum (1)
    0x75, 0x01, //     Report Size (1)
    0x95, 0x08, //     Report Count (8)
    0x81, 0x02, //     Input (Data, Variable, Absolute)

    0x95, 0x01, //     Report Count (1)
    0x75, 0x08, //     Report Size (8)
    0x81, 0x01, //     Input (Constant) reserved byte(1)

    0x95, 0x05, //     Report Count (5)
    0x75, 0x01, //     Report Size (1)
    0x05, 0x08, //     Usage Page (Page# for LEDs)
    0x19, 0x01, //     Usage Minimum (1)
    0x29, 0x05, //     Usage Maximum (5)
    0x91, 0x02, //     Output (Data, Variable, Absolute), Led report
    0x95, 0x01, //     Report Count (1)
    0x75, 0x03, //     Report Size (3)
    0x91, 0x01, //     Output (Data, Variable, Absolute), Led report padding

    0x95, 0x06, //     Report Count (6)
    0x75, 0x08, //     Report Size (8)
    0x15, 0x00, //     Logical Minimum (0)
    0x25, 0x65, //     Logical Maximum (255)
    0x05, 0x07, //     Usage Page (Key codes)
    0x19, 0x00, //     Usage Minimum (0)
    0x29, 0x65, //     Usage Maximum (101)
    0x81, 0x00, //     Input (Data, Array) Key array(6 bytes)

    0xC0, // End Collection (Application)

    // system
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x80, // USAGE (System Control)
    0xa1, 0x01, // COLLECTION (Application)
    0x85, REPORT_ID_SYSTEM, //   REPORT_ID (2)
    0x15, 0x01, //   LOGICAL_MINIMUM (0x1)
    0x26, 0xb7, 0x00, //   LOGICAL_MAXIMUM (0xb7)
    0x19, 0x01, //   USAGE_MINIMUM (0x1)
    0x29, 0xb7, //   USAGE_MAXIMUM (0xb7)
    0x75, 0x10, //   REPORT_SIZE (16)
    0x95, 0x01, //   REPORT_COUNT (1)
    0x81, 0x00, //   INPUT (Data,Array,Abs)
    0xc0, // END_COLLECTION

    // consumer
    0x05, 0x0c, // USAGE_PAGE (Consumer Devices)
    0x09, 0x01, // USAGE (Consumer Control)
    0xa1, 0x01, // COLLECTION (Application)
    0x85, REPORT_ID_CONSUMER, //   REPORT_ID (3)
    0x15, 0x01, //   LOGICAL_MINIMUM (0x1)
    0x26, 0x9c, 0x02, //   LOGICAL_MAXIMUM (0x29c)
    0x19, 0x01, //   USAGE_MINIMUM (0x1)
    0x2a, 0x9c, 0x02, //   USAGE_MAXIMUM (0x29c)
    0x75, 0x10, //   REPORT_SIZE (16)
    0x95, 0x01, //   REPORT_COUNT (1)
    0x81, 0x00, //   INPUT (Data,Array,Abs)
    0xc0, // END_COLLECTION
};

static ble_hids_t m_hids; /**< Structure used to identify the HID service. */
static bool m_in_boot_mode = false; /**< Current protocol mode. */

uint8_t led_val;

/** Abstracts buffer element */
typedef struct hid_key_buffer {
    uint8_t data_len; /**< Total length of data */
    uint8_t* p_data; /**< Scanned key pattern */
    ble_hids_t* p_instance; /**< Identifies peer and service instance */
} buffer_entry_t;

STATIC_ASSERT(sizeof(buffer_entry_t) % 4 == 0);

/** Circular buffer list */
typedef struct
{
    buffer_entry_t buffer[MAX_BUFFER_ENTRIES]; /**< Maximum number of entries that can enqueued in the list */
    uint8_t rp; /**< Index to the read location */
    uint8_t wp; /**< Index to write location */
    uint8_t count; /**< Number of elements in the list */
} buffer_list_t;

STATIC_ASSERT(sizeof(buffer_list_t) % 4 == 0);

/** 缓存队列访问宏
 *
 * @{ */
/** Initialization of buffer list */
#define BUFFER_LIST_INIT()     \
    do {                       \
        buffer_list.rp = 0;    \
        buffer_list.wp = 0;    \
        buffer_list.count = 0; \
    } while (0)

/** Provide status of data list is full or not */
#define BUFFER_LIST_FULL() \
    ((MAX_BUFFER_ENTRIES == buffer_list.count - 1) ? true : false)

/** Provides status of buffer list is empty or not */
#define BUFFER_LIST_EMPTY() \
    ((0 == buffer_list.count) ? true : false)

#define BUFFER_ELEMENT_INIT(i)                 \
    do {                                       \
        buffer_list.buffer[(i)].p_data = NULL; \
    } while (0)

/** List to enqueue not just data to be sent, but also related information like the handle, connection handle etc */
static buffer_list_t buffer_list;

static void on_hids_evt(ble_hids_t* p_hids, ble_hids_evt_t* p_evt);
void hids_buffer_init(void);

/**@brief Function for initializing HID Service.
 */
void hids_init(void)
{
    uint32_t err_code;
    ble_hids_init_t hids_init_obj;
    ble_hids_inp_rep_init_t input_report_array[INPUT_REPORT_COUNT];
    ble_hids_inp_rep_init_t* p_input_report;
    ble_hids_outp_rep_init_t output_report_array[OUTPUT_REPORT_COUNT];
    ble_hids_outp_rep_init_t* p_output_report;
    uint8_t hid_info_flags;

    memset((void*)input_report_array, 0, sizeof(ble_hids_inp_rep_init_t) * INPUT_REPORT_COUNT);
    memset((void*)output_report_array, 0, sizeof(ble_hids_outp_rep_init_t) * OUTPUT_REPORT_COUNT);

    // Initialize HID Service
    // keyboard input report
    p_input_report = &input_report_array[KEYBOARD_INPUT_REPORT_INDEX];
    p_input_report->max_len = INPUT_REPORT_KEYS_MAX_LEN;
    p_input_report->rep_ref.report_id = 1;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

#ifdef BLE_LINK_SEC
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_input_report->security_mode.write_perm);
#else
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);
#endif

    // system input report
    p_input_report = &input_report_array[SYSTEM_INPUT_REPORT_INDEX];
    p_input_report->max_len = 2;
    p_input_report->rep_ref.report_id = 2;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

#ifdef BLE_LINK_SEC
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_input_report->security_mode.write_perm);
#else
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);
#endif

    // consumer input report
    p_input_report = &input_report_array[CONSUMER_INPUT_REPORT_INDEX];
    p_input_report->max_len = 2;
    p_input_report->rep_ref.report_id = 3;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

#ifdef BLE_LINK_SEC
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_input_report->security_mode.write_perm);
#else
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);
#endif

    // keyboard output report
    p_output_report = &output_report_array[KEYBOARD_OUTPUT_REPORT_INDEX];
    p_output_report->max_len = OUTPUT_REPORT_MAX_LEN;
    p_output_report->rep_ref.report_id = 1;
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

#ifdef BLE_LINK_SEC
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_output_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_output_report->security_mode.write_perm);
#else
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.write_perm);

#endif

    // init main
    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler = on_hids_evt;
    hids_init_obj.error_handler = service_error_handler;
    hids_init_obj.is_kb = true;
    hids_init_obj.is_mouse = false;
    hids_init_obj.inp_rep_count = INPUT_REPORT_COUNT;
    hids_init_obj.p_inp_rep_array = input_report_array;
    hids_init_obj.outp_rep_count = OUTPUT_REPORT_COUNT;
    hids_init_obj.p_outp_rep_array = output_report_array;
    hids_init_obj.feature_rep_count = 0;
    hids_init_obj.p_feature_rep_array = NULL;
    hids_init_obj.rep_map.data_len = sizeof(report_map_data);
    hids_init_obj.rep_map.p_data = report_map_data;
    hids_init_obj.hid_information.bcd_hid = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags = hid_info_flags;
    hids_init_obj.included_services_count = 0;
    hids_init_obj.p_included_services_array = NULL;

#ifdef BLE_LINK_SEC
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_boot_kb_inp_rep.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&hids_init_obj.security_mode_protocol.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&hids_init_obj.security_mode_protocol.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);
#else
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_boot_kb_inp_rep.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);
#endif

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
    hids_buffer_init();
}

/**@brief Function for handling the HID Report Characteristic Write event.
 *
 * @param[in]   p_evt   HID service event.
 */
static void on_hid_rep_char_write(ble_hids_evt_t* p_evt)
{
    if (p_evt->params.char_write.char_id.rep_type == BLE_HIDS_REP_TYPE_OUTPUT) {
        uint32_t err_code;
        uint8_t report_val;
        uint8_t report_index = p_evt->params.char_write.char_id.rep_index;

        switch (report_index) {
        case KEYBOARD_OUTPUT_REPORT_INDEX: {
            // This code assumes that the outptu report is one byte long. Hence the following
            // static assert is made.
            STATIC_ASSERT(OUTPUT_REPORT_MAX_LEN == 1);

            err_code = ble_hids_outp_rep_get(&m_hids, report_index, OUTPUT_REPORT_MAX_LEN, 0, &report_val);
            APP_ERROR_CHECK(err_code);

            led_val = report_val;
            break;
        }
        }
    }
}

/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t* p_hids, ble_hids_evt_t* p_evt)
{
    switch (p_evt->evt_type) {
    case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
        m_in_boot_mode = true;
        break;

    case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
        m_in_boot_mode = false;
        break;

    case BLE_HIDS_EVT_REP_CHAR_WRITE:
        on_hid_rep_char_write(p_evt);
        break;

    case BLE_HIDS_EVT_NOTIF_ENABLED: {
        dm_service_context_t service_context;
        service_context.service_type = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;
        service_context.context_data.len = 0;
        service_context.context_data.p_data = NULL;

        if (m_in_boot_mode) {
            // Protocol mode is Boot Protocol mode.
            if (
                p_evt->params.notification.char_id.uuid == BLE_UUID_BOOT_KEYBOARD_INPUT_REPORT_CHAR) {
                // The notification of boot keyboard input report has been enabled.
                // Save the system attribute (CCCD) information into the flash.
                uint32_t err_code;

                err_code = dm_service_context_set(&m_bonded_peer_handle, &service_context);
                if (err_code != NRF_ERROR_INVALID_STATE) {
                    APP_ERROR_CHECK(err_code);
                } else {
                    // The system attributes could not be written to the flash because
                    // the connected central is not a new central. The system attributes
                    // will only be written to flash only when disconnected from this central.
                    // Do nothing now.
                }
            } else {
                // Do nothing.
            }
        } else if (p_evt->params.notification.char_id.rep_type == BLE_HIDS_REP_TYPE_INPUT) {
            // The protocol mode is Report Protocol mode. And the CCCD for the input report
            // is changed. It is now time to store all the CCCD information (system
            // attributes) into the flash.
            uint32_t err_code;

            err_code = dm_service_context_set(&m_bonded_peer_handle, &service_context);
            if (err_code != NRF_ERROR_INVALID_STATE) {
                APP_ERROR_CHECK(err_code);
            } else {
                // The system attributes could not be written to the flash because
                // the connected central is not a new central. The system attributes
                // will only be written to flash only when disconnected from this central.
                // Do nothing now.
            }
        } else {
            // The notification of the report that was enabled by the central is not interesting
            // to this application. So do nothing.
        }
        break;
    }

    default:
        // No implementation needed.
        break;
    }
}

/**@brief   Function for transmitting a key scan Press & Release Notification.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  p_instance     Identifies the service for which Key Notifications are requested.
 * @param[in]  p_key_pattern  Pointer to key pattern.
 * @param[in]  pattern_len    Length of key pattern. 0 < pattern_len < 7.
 * @return     NRF_SUCCESS on success, BLE_ERROR_NO_TX_BUFFERS in case transmission could not be
 *             completed due to lack of transmission buffer or other error codes indicating reason
 *             for failure.
 *
 */
static uint32_t send_key_scan_press_release(ble_hids_t* p_hids,
    uint8_t* p_key_pattern,
    uint16_t pattern_len)
{
    uint32_t err_code;

// 处理键盘状态灯
#ifdef LED_STATE_FIX
    for (int i = 2; i < pattern_len; i++) {
        switch (p_key_pattern[i]) {
        case KC_NUMLOCK:
            led_val ^= 0x01;
            break;
        case KC_CAPSLOCK:
            led_val ^= 0x02;
            break;
        case KC_SCROLLLOCK:
            led_val ^= 0x04;
            break;
        default:
            break;
        }
    }
#endif
    if (!m_in_boot_mode) {
        err_code = ble_hids_inp_rep_send(p_hids,
            0,
            INPUT_REPORT_KEYS_MAX_LEN,
            p_key_pattern);
    } else {
        err_code = ble_hids_boot_kb_inp_rep_send(p_hids,
            INPUT_REPORT_KEYS_MAX_LEN,
            p_key_pattern);
    }

    return err_code;
}
/**@brief   Function for initializing the buffer queue used to key events that could not be
 *          transmitted
 *
 * @warning This handler is an example only. You need to analyze how you wish to buffer or buffer at
 *          all.
 *
 * @note    In case of HID keyboard, a temporary buffering could be employed to handle scenarios
 *          where encryption is not yet enabled or there was a momentary link loss or there were no
 *          Transmit buffers.
 */
void hids_buffer_init(void)
{
    uint32_t buffer_count;

    BUFFER_LIST_INIT();

    for (buffer_count = 0; buffer_count < MAX_BUFFER_ENTRIES; buffer_count++) {
        BUFFER_ELEMENT_INIT(buffer_count);
    }
}

/**@brief Function for enqueuing key scan patterns that could not be transmitted either completely
 *        or partially.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  p_hids         Identifies the service for which Key Notifications are buffered.
 * @param[in]  p_key_pattern  Pointer to key pattern.
 * @param[in]  pattern_len    Length of key pattern.
 * @param[in]  offset         Offset applied to Key Pattern when requesting a transmission on
 *                            dequeue, @ref buffer_dequeue.
 * @return     NRF_SUCCESS on success, else an error code indicating reason for failure.
 */
uint32_t hids_buffer_enqueue(ble_hids_t* p_hids,
    uint8_t* p_key_pattern,
    uint16_t pattern_len)
{
    buffer_entry_t* element;
    uint32_t err_code = NRF_SUCCESS;

    if (BUFFER_LIST_FULL()) {
        // Element cannot be buffered.
        err_code = NRF_ERROR_NO_MEM;
    } else {
        // Make entry of buffer element and copy data.
        element = &buffer_list.buffer[(buffer_list.wp)];
        element->p_instance = p_hids;
        element->p_data = p_key_pattern;
        element->data_len = pattern_len;

        buffer_list.count++;
        buffer_list.wp++;

        if (buffer_list.wp == MAX_BUFFER_ENTRIES) {
            buffer_list.wp = 0;
        }
    }

    return err_code;
}

/**@brief   Function to dequeue key scan patterns that could not be transmitted either completely of
 *          partially.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  tx_flag   Indicative of whether the dequeue should result in transmission or not.
 * @note       A typical example when all keys are dequeued with transmission is when link is
 *             disconnected.
 *
 * @return     NRF_SUCCESS on success, else an error code indicating reason for failure.
 */
uint32_t hids_buffer_dequeue(bool tx_flag)
{
    buffer_entry_t* p_element;
    uint32_t err_code = NRF_SUCCESS;

    if (BUFFER_LIST_EMPTY()) {
        err_code = NRF_ERROR_NOT_FOUND;
    } else {
        bool remove_element = true;

        p_element = &buffer_list.buffer[(buffer_list.rp)];

        if (tx_flag) {
            err_code = send_key_scan_press_release(p_element->p_instance,
                p_element->p_data,
                p_element->data_len);
            // An additional notification is needed for release of all keys, therefore check
            // is for actual_len <= element->data_len and not actual_len < element->data_len
            if (err_code == BLE_ERROR_NO_TX_BUFFERS) {
                // Transmission could not be completed, do not remove the entry, adjust next data to
                // be transmitted
                remove_element = false;
            }
        }

        if (remove_element) {
            BUFFER_ELEMENT_INIT(buffer_list.rp);

            buffer_list.rp++;
            buffer_list.count--;

            if (buffer_list.rp == MAX_BUFFER_ENTRIES) {
                buffer_list.rp = 0;
            }
        }
    }

    return err_code;
}

/**@brief Function for sending sample key presses to the peer.
 *
 * @param[in]   key_pattern_len   Pattern length.
 * @param[in]   p_key_pattern     Pattern to be sent.
 */
void hids_keys_send(uint8_t key_pattern_len, uint8_t* p_key_pattern)
{
    uint32_t err_code;

    err_code = send_key_scan_press_release(&m_hids,
        p_key_pattern,
        key_pattern_len);
    // An additional notification is needed for release of all keys, therefore check
    // is for actual_len <= key_pattern_len and not actual_len < key_pattern_len.
    if (err_code == BLE_ERROR_NO_TX_BUFFERS) {
        // Buffer enqueue routine return value is not intentionally checked.
        // Rationale: Its better to have a a few keys missing than have a system
        // reset. Recommendation is to work out most optimal value for
        // MAX_BUFFER_ENTRIES to minimize chances of buffer queue full condition
        UNUSED_VARIABLE(hids_buffer_enqueue(&m_hids, p_key_pattern, key_pattern_len));
    }

    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE) && (err_code != BLE_ERROR_NO_TX_BUFFERS) && (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
        APP_ERROR_HANDLER(err_code);
    }
}
/**
 * @brief 发送System Key
 * 
 * @param key_pattern_len 
 * @param p_key_pattern 
 */
void hids_system_key_send(uint8_t key_pattern_len, uint8_t* p_key_pattern)
{

    uint32_t err_code;
    err_code = ble_hids_inp_rep_send(&m_hids,
        REPORT_ID_SYSTEM - 1,
        key_pattern_len,
        p_key_pattern);
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE) && (err_code != BLE_ERROR_NO_TX_BUFFERS) && (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
        APP_ERROR_HANDLER(err_code);
    }
}
/**
 * @brief 发送Consumer Key
 * 
 * @param key_pattern_len 
 * @param p_key_pattern 
 */
void hids_consumer_key_send(uint8_t key_pattern_len, uint8_t* p_key_pattern)
{

    uint32_t err_code;
    err_code = ble_hids_inp_rep_send(&m_hids,
        REPORT_ID_CONSUMER - 1,
        key_pattern_len,
        p_key_pattern);
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE) && (err_code != BLE_ERROR_NO_TX_BUFFERS) && (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
        APP_ERROR_HANDLER(err_code);
    }
}

void hids_on_ble_evt(ble_evt_t* p_ble_evt)
{
    ble_hids_on_ble_evt(&m_hids, p_ble_evt);
    switch (p_ble_evt->header.evt_id) {
    case BLE_EVT_TX_COMPLETE:
        // Send next key event
        (void)hids_buffer_dequeue(true);
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        // Dequeue all keys without transmission.
        (void)hids_buffer_dequeue(false);
        break;
    default:
        break;
    }
}
