#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <wiringSerial.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <unistd.h>

#define BUTTON_PIN 0

int serial_init();

int interrupt_init();

int serial_send(char* msg);

#endif
