#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <util/delay.h>
#include <string.h>
#include "usb_serial.h"

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))


#define DEFAULT_BAUD_RATE 115200
#define PACKET_SIZE 64

//Packet and control
#define PACKET_START_1 0x66
#define PACKET_START_2 0x66
#define PACKET_END_1 0x042
#define PACKET_END_2 0x042

#define CONTROL_START 0x05
#define CONTROL_END 0x10


//Data types
#define DATA_ANALOG_1 1
#define DATA_ANALOG_2 2
#define DATA_ANALOG_3 3

#define DATA_ANALOG_4 4
#define DATA_ANALOG_5 5
#define DATA_ANALOG_6 6

#define DATA_ANALOG_7 7
#define DATA_ANALOG_8 8
#define DATA_ANALOG_9 9

#define DATA_ANALOG_10 10 //unused
#define DATA_ANALOG_11 11 //unused
#define DATA_ANALOG_12 12 //unused

#define DATA_ANALOG_13 13
#define DATA_ANALOG_14 14

#define DATA_ANALOG_15 15
#define DATA_ANALOG_16 16

#define DATA_DIGITAL_1 17
#define DATA_DIGITAL_2 18
#define DATA_DIGITAL_3 19 //F is used for analog and is unused here
#define DATA_DIGITAL_4 20


#define analog_filter .6

struct dataStorage{
		uint16_t analog_1;
		uint16_t analog_2;
		uint16_t analog_3;
		uint16_t analog_4;
		uint16_t analog_5;
		uint16_t analog_6;
		uint16_t analog_7;
		uint16_t analog_8;
		uint16_t analog_9;
		uint16_t analog_10;
		uint16_t analog_11;
		uint16_t analog_12;
		uint16_t analog_13;
		uint16_t analog_14;
		uint16_t analog_15;
		uint16_t analog_16;
		
		uint16_t digital_1;
		uint16_t digital_2;
		uint16_t digital_3;
		uint16_t digital_4;
	} data;