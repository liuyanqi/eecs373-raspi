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

 static bt_uuid_t g_uuid;
 gatt_connection_t* connection;
 int ble_init(){
 	int i;
	//char mac[18] = "C5:18:3C:48:E6:F4";
	char mac[18] = "FB:03:F9:0B:6A:DA";
	connection = gattlib_connect(NULL, mac, BDADDR_LE_RANDOM, BT_IO_SEC_LOW, 0, 0);
	if (connection == NULL) {
		fprintf(stderr, "Fail to connect to the bluetooth device.\n");
		return 1;
	}
	return 0;
 }

uint8_t ble_read(){
 	if (connection == NULL) {
		fprintf(stderr, "Fail to connect to the bluetooth device.\n");
		return 1;
	}
 	uint8_t buffer[100];
	//char ble_uuid[37] = "00002a00-0000-1000-8000-00805f9b34fb";
	//char ble_uuid[37] = "0011a0aa-4444-4444-4444-444444444444";
	char ble_uuid[37] = "00110a0a-4455-6677-8899-aabbccddeeff";
 	if (bt_string_to_uuid(&g_uuid, ble_uuid) < 0) {
		return 1;
	}
	printf("Read Char\n");
 	int len = gattlib_read_char_by_uuid(connection, &g_uuid, buffer, sizeof(buffer));
 	printf("Read UUID completed: ");
		int i =0;
		for (i = 0; i < len; i++)
			printf("%02x ", buffer[i]);
		printf("\n");
	return buffer[0];

 }

 int ble_write(uint8_t* send){
 	if (connection == NULL) {
		fprintf(stderr, "Fail to connect to the bluetooth device.\n");
		return 1;
	}
 	uint16_t handle = 0x002d;  //TODO: FIXME
	//printf("value data: %ld\n", value_data);
	//for(int i =0; i<sizeof(value_data); i++){
		//buffer[i] = (value_data>>(i*8))& 0xFF;
	//}
	//for (int i =0; i < 10; i++){
		//printf("buffer %d\n", buffer[i]);
	//}
	// send[0] = (value_data>>8)&0xFF;
	// send[1] = (value_data)&0xFF;
	printf("start write\n");
	int ret = gattlib_write_char_by_handle(connection, handle, send, 1);

	printf("wrote: %d\n",send[0]); 

	assert(ret == 0);
	return 0;
 }
void ble_disconnect(){
	gattlib_disconnect(connection);
}



