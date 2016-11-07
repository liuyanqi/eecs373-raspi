#ifndef _BLE_READ_WRITE_H_
#define _BLE_READ_WRITE_H_ 
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "gattlib.h"

#define BLE_ADDR "FB:03:F9:0B:6A:DA"
#define BLE_UUID ""

 static bt_uuid_t g_uuid;

 gatt_connection_t* connection;
 int ble_init();
 int ble_read();
 int ble_write(uint8_t* send);
#endif
