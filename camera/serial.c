#include "serial.h"

volatile int eventCounter =0;
int fd=-1;

int serial_init(){
	if(wiringPiSetup() < 0){
		printf("Unable to setup WiringPi\n");
		return -1;
	}

	fd = serialOpen("/dev/ttyS0", 115200);
	if(fd < 0){
		printf("fail to open serial port");
		return -1;
	}
	return fd;
}

int serial_send( char* message){
	   if(fd <0){
		printf("serial hasn't open yet\n");
		return 1;
	}
	    serialPuts(fd, message);
	    usleep(100000);
return 0;
}

void myInterrupt(void){
	eventCounter++;
	printf("eventCounter %d\n", eventCounter);
	char message[7] = "color\0";
	    serial_send(message);
	   printf("serial sent\n");
}

int interrupt_init(){
	if(wiringPiISR(BUTTON_PIN, INT_EDGE_FALLING,&myInterrupt)<0){
		printf("Unable to set up ISR\n");
		return 1;
	}
	return 0;
}


