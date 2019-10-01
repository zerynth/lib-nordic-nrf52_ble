#ifndef __BLE_SERVICE__
#define __BLE_SERVICE__

// fix for conflict between __opcodes.h and the internal registers of the chip:
#undef IN

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "vhal_sys.h"

#define NUM_BLE_EVTS 16
#define NUM_SCAN_EVTS 32
#define BLE_GATT_EVENT 1
#define BLE_GAP_EVENT 0
#define BLE_GAP_EVT_IS_CONNECTED 0
#define BLE_GAP_EVT_IS_DISCONNECTED 1
#define BLE_GAP_EVT_SCAN_REPORT 2
#define BLE_GAP_EVT_SCAN_STARTED 3
#define BLE_GAP_EVT_SCAN_STOPPED 4
#define BLE_GAP_EVT_SHOW_PASSKEY 5
#define BLE_GAP_EVT_MATCH_PASSKEY 6
#define BLE_GAP_EVT_AUTH_FAILED 7
#define BLE_GAP_EVT_ADV_STARTED 8
#define BLE_GAP_EVT_ADV_STOPPED 9

#define HANDLE(x) (ble.handles[x])
// #define CHANDLE() (ble.handles[ble.hh_created])
#define SERVICE(x) (ble.services[x])
#define CHARACTERISTIC(i,j)  (SERVICE(i).chars[j])
// #define CSERVICE() (ble.services[ble.srv_created])

#define BLE_SERVICE 0
#define BLE_DECL 1
#define BLE_VAL 2
#define BLE_CCCD 3
#define BLE_PERM_NOTIFY 16
#define BLE_PERM_INDICATE 32
#define BLE_PERM_WRITE 8

#define CAP_OUT  0
#define CAP_IO  1
#define CAP_KB  2
#define CAP_NONE  3
#define CAP_KBD  4
#define AUTH_NO_BOND  1
#define AUTH_BOND  1
#define AUTH_MITM  2
#define AUTH_SC  4
#define AUTH_KEYPRESS  8
#define KEY_ENC  1
#define KEY_ID   2
#define KEY_CSR 4
#define KEY_LNK 8

#define SDH_ERROR_CHECK(err,msg) do {\
    if (err!=NRF_SUCCESS){      \
        printf(msg);            \
        printf(" %x\n",err);      \
        return err;             \
    }                           \
    } while(0)

#define BLE_ERROR_CHECK(err,msg,exc) do {\
    if (err!=NRF_SUCCESS){      \
        printf(msg);            \
        printf(" %x\n",err);      \
        return exc;             \
    }                           \
    } while(0)

typedef struct _ble_service BLEService;
typedef struct _ble_char BLECharacteristic;

struct _ble_char {
    ble_gatts_char_handles_t handles;
    uint16_t cccd;
    uint8_t max_vsize;
    uint8_t vsize;
    uint8_t value[20];
    uint16_t perm;
    uint16_t uuid;
    BLEService *service;
};

struct _ble_service {
    uint16_t handle;
    uint8_t nchar;
    uint16_t uuid;
    uint8_t base_uuid[16];
    // BLEUUID uuid;
    BLECharacteristic *chars;
    // esp_gatts_attr_db_t *srv;
};

typedef struct _ble_scan_res {
    uint8_t data[32];
    uint8_t addr[6];
    int16_t rssi;
    uint8_t datalen;
    uint8_t addrtype;
    uint8_t scantype;
    uint8_t unused;
} BLEScanResult;

typedef struct _ble_scan_dup {
    uint8_t data[32];
    uint8_t addr[6];
    int16_t rssi;
} BLEScanDup;

#define BLE_PERM_BROADCAST 1
#define BLE_PERM_READ  2
#define BLE_PERM_WRITE_WO  4
#define BLE_PERM_WRITE  8
#define BLE_PERM_NOTIFY  16
#define BLE_PERM_INDICATE  32
#define BLE_PERM_WRITE_SIGNED  64


typedef struct _ble_event {
    uint8_t type;
    uint8_t status;
    uint16_t service;
    uint16_t characteristic;
    void *object;
} BLEEvent;

typedef struct _ble_handle {
    uint16_t handle;
    uint8_t srv;
    uint8_t chr;
    uint8_t pos;
    uint8_t type;
} BLEHandle;

typedef struct _ble_device {
    uint8_t* name;
    uint32_t namelen;
    uint32_t appearance;
    uint32_t level;
    uint32_t security;
    uint32_t min_conn;
    uint32_t max_conn;
    uint32_t latency;
    uint32_t conn_sup;
    uint32_t adv_mode;
    uint32_t advertising_timeout;
    uint32_t advertising_interval;

    uint32_t connected;
    uint16_t conn_set;
    ble_gap_conn_params_t conn_params;
    ble_gap_addr_t  peer_addr;
    
    uint8_t advert_data[31];
    uint32_t advert_data_len;
    uint8_t scanrsp_data[31];
    uint32_t scanrsp_data_len;

    ble_gap_scan_params_t scan_param;
    uint16_t scanning_set;
    uint16_t duplicates;
    int16_t scan_p;
    int16_t scan_n;
    BLEScanResult scr[NUM_SCAN_EVTS];


    ble_gap_sec_params_t sec_param;
    uint32_t security_set;
    uint32_t capabilities;
    uint32_t scheme;
    uint32_t key_size;
    uint32_t bonding;
    uint32_t init_key;
    uint32_t resp_key;
    uint32_t oob;
    uint8_t passkey[6];
    uint32_t removed_bond;

    int16_t evt_p;
    int16_t evt_n;
    BLEEvent events[NUM_BLE_EVTS];

    uint32_t nserv;
    BLEService *services;
} BLEDevice;

//default parameters for security
#define SEC_PARAM_BOND                  0                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(400, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(650, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#endif

