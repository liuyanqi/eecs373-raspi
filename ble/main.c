#include "ble_read_write.h"
//static uint8_t notification_result = 0;
volatile uint8_t notification_result;
uint32_t One = 1;
uint32_t* PtrToOne = &One;

struct status{
        int systemon;
};

void notification_cb(uint16_t handle, const uint8_t* data, size_t data_length, void* user_data){
	struct status* s1 = (struct status *)(user_data);
	s1->systemon = s1->systemon+1;
	printf("Notification on handle %d\n", handle);
//	*user_data = PtrToOne[0];

}




int main(){
	size_t i = 0, j = 0;
	uint8_t send[1];
/*
	uint32_t isnotified[1];
	struct status s;
	int counter_for_ble;
	s.systemon = 0;

	isnotified[0] = 0;
	send[0] = 27;



	ble_init();
        printf("status notif\n");
        uint16_t status_handle = 0x0031;
        uint16_t enable_notif = 0x0001;
        // Enable status notif
        gattlib_write_char_by_handle(connection, status_handle+1, &enable_notif, sizeof(enable_notif));
        printf("register\n");
        // register notif handler
        gattlib_register_notification(connection, notification_cb, &s);


	while(1){
		for(i = 0; i < 10000000; ++i){}
		printf("waiting %d: Notification result is %d\n", j, s.systemon);
		j++;
		//read ble
        	if(s.systemon == 1){
            	    for(counter_for_ble = 0; counter_for_ble < 1000; ++counter_for_ble){}
            	    printf("start check system\n");
            	    ble_read();
            	    s.systemon = 0;
            	    printf("check system\n");
        	    //counter_for_ble = 0;
	        }


	}
*/
	
	ble_init();
	for(i = 0; i < 100; ++i){
		for(j = 0; j<10000000; ++j){}
		send[0] = i;
		ble_write(send);
	}
	



	ble_disconnect();
}
