/*
*
*
*
*/
#include "comDefines.h"



void setupHardware(void);
void read_buttons(void);
void initStruct(void);
void packData(void);

//ADC ISR values
volatile uint8_t conversion_status = 0;
volatile uint8_t thumb_count = 0;

//volatile struct dataStorage *data;
	
//digital debounced values (register logged as an unsigned int)
//Only c and d are being used
uint8_t digital_a=0xFF, digital_b=0xFF, digital_c=0xFF, digital_d=0xFF,digital_e=0xFF, digital_f=0xFF, digital_g=0xFF, digital_h=0xFF;

uint8_t buffer[PACKET_SIZE];

// Basic command interpreter for controlling port pins
int main(void)
{
	setupHardware();
	sei();
	//initStruct();

	//set the next ADC conversion
	ADCSRA |= (1<<ADSC);
	conversion_status = 1;
	
	while (1) {
		// wait for the user to run their terminal emulator program
		// which sets DTR to indicate it is ready to receive.
		while (!(usb_serial_get_control() & USB_SERIAL_DTR)) /* wait */ ;

		// discard anything that was received prior.  Sometimes the
		// operating system or other software will send a modem
		// "AT command", which can still be buffered.
		usb_serial_flush_input();
		
		while (1) {
			read_buttons();
		
			if(conversion_status == 0 && usb_serial_available() >= 1)
			{
				if(usb_serial_getchar() == CONTROL_START){
					//usb_serial_flush_input();
					//Set unused bits in used ports
					digital_c |= (1<<7);
					digital_d |= (1<<6) | (1<<7);
					data.digital_1 = (digital_a << 8) | digital_b;
					data.digital_2 = (digital_c << 8) | digital_d;
					data.digital_3 = (digital_e << 8) | digital_f;
					data.digital_4 = (digital_g << 8) | digital_h;
					//Transmit data
					packData();
					usb_serial_write(buffer, sizeof(buffer));
					//set the next ADC conversion
					ADCSRA |= (1<<ADSC);
					conversion_status = 1;
				}
			}
			//If the PC isn't ready for the data go ahead and do another analog sample
			else if (conversion_status == 0){
				//set the next ADC conversion
				ADCSRA |= (1<<ADSC);
				conversion_status = 1;
			}
		}
	}
}

void packData(void){
	buffer[0] = PACKET_START_1;
	buffer[1] = PACKET_START_2;
	
	buffer[2] = DATA_ANALOG_1;
		buffer[3] = (data.analog_1 >> 8);
		buffer[4] = data.analog_1 & 0xFF;
		
	buffer[5] = DATA_ANALOG_2;
		buffer[6] = (data.analog_2 >> 8);
		buffer[7] = data.analog_2 & 0xFF;
		
	buffer[8] = DATA_ANALOG_3;
		buffer[9] = (data.analog_3 >> 8);
		buffer[10] = data.analog_3 & 0xFF;
		
	buffer[11] = DATA_ANALOG_4;
		buffer[12] = (data.analog_4 >> 8);
		buffer[13] = data.analog_4 & 0xFF;
		
	buffer[14] = DATA_ANALOG_5;
		buffer[15] = (data.analog_5 >> 8);
		buffer[16] = data.analog_5 & 0xFF;
		
	buffer[17] = DATA_ANALOG_6;
		buffer[18] = (data.analog_6 >> 8);
		buffer[19] = data.analog_6 & 0xFF;
		
	buffer[20] = DATA_ANALOG_7;
		buffer[21] = (data.analog_7 >> 8);
		buffer[22] = data.analog_7 & 0xFF;
		
	buffer[23] = DATA_ANALOG_8;
		buffer[24] = (data.analog_8 >> 8);
		buffer[25] = data.analog_8 & 0xFF;
		
	buffer[26] = DATA_ANALOG_9;
		buffer[27] = (data.analog_9 >> 8);
		buffer[28] = data.analog_9 & 0xFF;
		
	buffer[29] = DATA_ANALOG_10;
		buffer[30] = (data.analog_10 >> 8);
		buffer[31] = data.analog_10 & 0xFF;
		
	buffer[32] = DATA_ANALOG_11;
		buffer[33] = (data.analog_11 >> 8);
		buffer[34] = data.analog_11 & 0xFF;
		
	buffer[35] = DATA_ANALOG_12;
		buffer[36] = (data.analog_12 >> 8);
		buffer[37] = data.analog_12 & 0xFF;
		
	buffer[38] = DATA_ANALOG_13;
		buffer[39] = (data.analog_13 >> 8);
		buffer[40] = data.analog_13 & 0xFF;
		
	buffer[41] = DATA_ANALOG_14;
		buffer[42] = (data.analog_14 >> 8);
		buffer[43] = data.analog_14 & 0xFF;
		
	buffer[44] = DATA_ANALOG_15;
		buffer[45] = (data.analog_15 >> 8);
		buffer[46] = data.analog_15 & 0xFF;
		
	buffer[47] = DATA_ANALOG_16;
		buffer[48] = (data.analog_16 >> 8);
		buffer[49] = data.analog_16 & 0xFF;
		
	buffer[50] = DATA_DIGITAL_1;
		buffer[51] = (data.digital_1 >> 8);
		buffer[52] = data.digital_1 & 0xFF;
		
	buffer[53] = DATA_DIGITAL_2;
		buffer[54] = (data.digital_2 >> 8);
		buffer[55] = data.digital_2 & 0xFF;
		
	buffer[56] = DATA_DIGITAL_3;
		buffer[57] = (data.digital_3 >> 8);
		buffer[58] = data.digital_3 & 0xFF;
		
	buffer[59] = DATA_DIGITAL_4;
		buffer[60] = (data.digital_4 >> 8);
		buffer[61] = data.digital_4 & 0xFF;
	
	buffer[PACKET_SIZE-2] = PACKET_END_1;
	buffer[PACKET_SIZE-1] = PACKET_END_2;
}

/** Configures the board hardware and chip peripherals */
void setupHardware(void)
{
	// set for 16 MHz clock, and turn on the LED
	CPU_PRESCALE(0);

	//This is set up for a Teensy 2.0++ (AT90USB1286) running at 16MHz
	//if using a different board these may need to be changed
	
	DDRD = 0x00; 		//configure port D as inputs with pullup resistors
	PORTD = 0xFF;
	DDRC = 0x00; 		//configure port C as inputs with pullup resistors
	PORTC = 0xFF;
	DDRB = 0x00; 		//configure port B as inputs with pullup resistors
	PORTB = 0xFF;
	DDRA = 0x00; 		//configure port A as inputs with pullup resistors
	PORTA = 0xFF;
	DDRE = 0x23; 	//set bit 0, 1, 6 as outputs
	PORTE = 0x20; 	//set bit 6 to high
	
	//setup the ADC
	ADMUX = 0x00; //set to external Vref, channel 0
	//ADMUX bits 0, 1, 2 set the ADC channel
	//ADEN - enable ADC, ADSC - start conversion
	//enables the ADC and the interrupt, set the division factor to 128
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	DIDR0 = 0xFF;  		// all analog pins (F0-F7) have digital input disabled
	
	/* Hardware Initialization */
	usb_init();
	while (!usb_configured()) /* wait */ ;
	_delay_ms(1000);
}

//<-----------------------------------Read and debounce button input---------------------------------->
void read_buttons(void)
{
	static uint8_t flag_d[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	static uint8_t old_d[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t i;
	static uint8_t flag_c[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	static uint8_t old_c[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	
	for(i=0;i<8;i++)
	{
		//PIND
		//analog style filter for the button inputs
		old_d[i] = old_d[i] * .75;
		if(bit_is_clear(PIND, i)){old_d[i] = old_d[i] + 0x3F;}
		//trigger to switch when the analog value gets high or low
		if((old_d[i] > 0xF0)&&(flag_d[i]==0)){flag_d[i]=1; digital_d &= ~(1 << i);}
		if((old_d[i] < 0x0F)&&(flag_d[i]==1)){flag_d[i]=0; digital_d |= (1 << i);}
		//PINC
		//analog style filter for the button inputs
		old_c[i] = old_c[i] * .75;
		if(bit_is_clear(PINC, i)){old_c[i] = old_c[i] + 0x3F;}
		//trigger to switch when the analog value gets high or low
		if((old_c[i] > 0xF0)&&(flag_c[i]==0)){flag_c[i]=1; digital_c &= ~(1 << i);}
		if((old_c[i] < 0x0F)&&(flag_c[i]==1)){flag_c[i]=0; digital_c |= (1 << i);}
	}
	//debounced values are stored in digital_d and digital_c
}

ISR(ADC_vect)
{	
	//first entry assumes ADMUX = 0x00;
	//PORTE B0, B1, B6 all == 0
	if(conversion_status == 1) //gyro_120_x
	{
		data.analog_4 =  (data.analog_4 * analog_filter) + ((1 - analog_filter) * (ADCL | (ADCH << 8)));
		ADMUX = 0x01; 						
		conversion_status++;
		_delay_us(1);
		ADCSRA |= (1<<ADSC); 				//enable the next conversion
		
	}else if(conversion_status == 2) //gyro_120_y
	{
		data.analog_5 = (data.analog_5 * analog_filter) + ((1 - analog_filter) * (ADCL | (ADCH << 8)));
		ADMUX = 0x02;
		conversion_status++;
		_delay_us(1);
		ADCSRA |= (1<<ADSC); 
	}else if(conversion_status == 3) //accel_x
	{
		data.analog_7 = (data.analog_7 * analog_filter) + ((1 - analog_filter) * (ADCL | (ADCH << 8)));
		ADMUX = 0x03; 						//set to fourth input port
		conversion_status++;
		_delay_us(1);
		ADCSRA |= (1<<ADSC); 				//enable the next conversion
	}else if(conversion_status == 4) //accel_y
	{
		data.analog_8 = (data.analog_8 * analog_filter) + ((1 - analog_filter) * (ADCL | (ADCH << 8)));
		ADMUX = 0x04;
		conversion_status++;
		_delay_us(1);
		ADCSRA |= (1<<ADSC); 				//enable the next conversion
	}else if(conversion_status == 5) //accel_z
	{
		data.analog_9 = (data.analog_9 * analog_filter) + ((1 - analog_filter) * (ADCL | (ADCH << 8)));
		ADMUX = 0x05; 						//set to sixth input port
		conversion_status++;
		_delay_us(1);
		ADCSRA |= (1<<ADSC); 				//enable the next conversion
	}else if(conversion_status == 6) //gyro_30_y
	{
		data.analog_2 = (data.analog_2 * analog_filter) + ((1 - analog_filter) * (ADCL | (ADCH << 8)));
		ADMUX = 0x06; 						//set to seventh input port
		conversion_status++;
		_delay_us(1);
		ADCSRA |= (1<<ADSC); 				//enable the next conversion
	}else if(conversion_status == 7) //gyro_30_x
	{
		data.analog_1 = (data.analog_1 * analog_filter) + ((1 - analog_filter) * (ADCL | (ADCH << 8)));
		ADMUX = 0x07; 						//set to eighth input port
		conversion_status++;
		_delay_us(2);
		ADCSRA |= (1<<ADSC);				//enable the next conversion
	}else if(conversion_status == 8)
	{
		if(thumb_count == 0)
		{
			data.analog_16 = (data.analog_16 * analog_filter) + ((1 - analog_filter) * (ADCL | (ADCH << 8)));
			//PORTE |= (1 << 0); //set 101
			PORTE = 0b00000001;
			thumb_count++;
		}
		else if(thumb_count == 1)
		{
			data.analog_15 = (data.analog_15 * analog_filter) + ((1 - analog_filter) * (ADCL | (ADCH << 8)));
			//PORTE &= ~(1 << 0);
			//PORTE |= (1 << 1); //set 110
			PORTE = 0b00100010;
			thumb_count++;
		}
		else if(thumb_count == 2)
		{
			data.analog_13 = (data.analog_13 * analog_filter) + ((1 - analog_filter) * (ADCL | (ADCH << 8)));
			//PORTE |= (1 << 0); //set 111
			PORTE = 0b00100011;
			thumb_count++;
		}
		else if(thumb_count == 3)
		{
			data.analog_14 = (data.analog_14 * analog_filter) + ((1 - analog_filter) * (ADCL | (ADCH << 8)));
			//PORTE &= ~(1 << 1) & ~(1 << 0); //set 100
			PORTE = 0b00000000;
			thumb_count = 0;
		}
		ADMUX = 0x00;
		conversion_status = 0;
	}
}


