/*
 *
 *  GattLib - GATT Library
 *
 *  Copyright (C) 2016  Olivier Martin <olivier@labapart.org>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "ble_read_write.h"

 int ble_init(){
 	int i;
	connection = gattlib_connect(NULL, BLE_ADDR, BDADDR_LE_RANDOM, BT_IO_SEC_LOW, 0, 0);
	if (connection == NULL) {
		fprintf(stderr, "Fail to connect to the bluetooth device.\n");
		return 1;
	}
	return 0;
 }

 int ble_read(){
 	if (connection == NULL) {
		fprintf(stderr, "Fail to connect to the bluetooth device.\n");
		return 1;
	}
 	uint8_t buffer[100];
 	if (bt_string_to_uuid(&g_uuid, BLE_UUID) < 0) {
		return 1;
	}
 	int len = gattlib_read_char_by_uuid(connection, &g_uuid, buffer, sizeof(buffer));
 	printf("Read UUID completed: ");
		int i =0;
		for (i = 0; i < len; i++)
			printf("%02x ", buffer[i]);
		printf("\n");
	return 0;

 }

 int ble_write(uint8_t* send){
 	if (connection == NULL) {
		fprintf(stderr, "Fail to connect to the bluetooth device.\n");
		return 1;
	}
 	uint16_t handle = 0x0037; //TODO: FIXME
	//printf("value data: %ld\n", value_data);
	//for(int i =0; i<sizeof(value_data); i++){
		//buffer[i] = (value_data>>(i*8))& 0xFF;
	//}
	//for (int i =0; i < 10; i++){
		//printf("buffer %d\n", buffer[i]);
	//}
	// send[0] = (value_data>>8)&0xFF;
	// send[1] = (value_data)&0xFF;

	int ret = gattlib_write_char_by_handle(connection, handle, send, 2);
	
	assert(ret == 0);
	return 0;
 }
