#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <unistd.h>

int fd=-1;

int serial_init(){

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


int main(){
	int setup = wiringPiSetup();
	pinMode(3, OUTPUT);
	serial_init();
	while(1){
		digitalWrite(3, 1);
		usleep(100000);
		digitalWrite(3, 0);
		usleep(2000000);
		char message[4] ="run";
		serial_send(message);
		usleep(2000000);

	}
}
