#ifndef __BLE_SERVICE__
#define __BLE_SERVICE__

#include "ble_gatts.h"

typedef struct _ble_char {
    uint16_t uuid;
    uint16_t permission;
    uint16_t dlen;
    uint16_t ilen;
    uint8_t *descriptor;
    uint16_t cccd;
    uint16_t status;
    ble_gatts_char_handles_t handles;
} BLEChar;

typedef struct _ble_service {
    uint16_t uuid;
    uint16_t handle;
    uint8_t uuid_type;
    uint8_t nchs;
    BLEChar *chs;
} BLEService;


#define BLE_PERM_BROADCAST 1
#define BLE_PERM_READ  2
#define BLE_PERM_WRITE_WO  4
#define BLE_PERM_WRITE  8
#define BLE_PERM_NOTIFY  16
#define BLE_PERM_INDICATE  32
#define BLE_PERM_WRITE_SIGNED  64


typedef struct _ble_event {
    uint16_t type;
    uint16_t service;
    uint16_t characteristic;
    uint16_t status;
} BLEEvent;


#endif