#ifndef _BLE_READ_WRITE_H_
#define _BLE_READ_WRITE_H_ 
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "gattlib.h"

#define BLE_ADDR "C5:18:3C:48:E6:F4"
#define BLE_UUID ""
extern  gatt_connection_t* connection;
 int ble_init();
 uint8_t ble_read();
 int ble_write(uint8_t* send);
#endif
