/*
 *  Created: 04.02.2016
 *  Author: Max Brueggemann
 */ 

/*
*	An example project implementing a simple modbus slave device using an
*	ATmega88PA running at 20MHz.
*	Baudrate: 38400, 8 data bits, 1 stop bit, no parity
*	Your busmaster can read/write the following data:
*	coils: 0 to 7
*	discrete inputs: 0 to 7
*	input registers: 0 to 3
*	holding registers: 0 to 3
*/

#define clientAddress 0x01
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>
#include <avr/wdt.h>
//#define F_CPU 20000000
#include "../yaMBSiavr.h"
#define VREF 5
#define POT 1000
volatile uint8_t instate = 0;
volatile uint8_t outstate = 0;
volatile uint16_t inputRegisters[4];
volatile uint16_t holdingRegisters[100];
volatile uint16_t analogVal;
volatile uint16_t readFlag;
uint16_t ReadADC(uint8_t );

double vbg, potval;
void timer0100us_start(void) {
	TCCR0B|=(1<<CS01); //prescaler 8
	TIMSK0|=(1<<TOIE0);
}

/*
*   Modify the following 3 functions to implement your own pin configurations...
*/

void SetOuts(volatile uint8_t in) {
	PORTB |= (((in & (1<<3))>>2) | ((in & (1<<0)))) ;
	in=~in;
	PORTB &= ~(((in & (1<<3))>>2) | ((in & (1<<0)))) ;
}

uint8_t ReadIns(void) {
	uint8_t ins=0x00;
	ins |= (PIND&((1<<PD2)|(1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7)));
	//ins |= ((PINB &(1<<PB3))>>2);
	//ins |= PIND;
	return ins;
}


void io_conf(void) { 

	/*
	 Outputs: PB2,PB1,PB0,PD7,PD5,PD6
	 Inputs: PC0, PC1, PC2, PC3, PC4, PC6, PD4, PD3
	*/

	DDRD |= 0x00;
	DDRB |= 0xFF;

//DDRB &= ~(1<<PB3);

	//DDRC=0x00;
	PORTD=0xFF;
	PORTB=0x00;
	PORTB |= (0<<PB1)|(0<<PB2);

	//PORTC=0x00;

//DDRD &= ~((1<<2)|(1<<5)|(1<<6)|(1<<7)|(1<<4)|(1<<3));


}


ISR(TIMER0_OVF_vect) { //this ISR is called 9765.625 times per second 65535
	modbusTickTimer();
}


void loopit(void)
{


	for(int i = 0 ; i< 6 ; i++)
	{
	    vbg=(double)VREF/1024*ReadADC(i);
	    //printing value to terminal

	    holdingRegisters[i]=vbg;


}
}

void modbusGet(void) {

	if (modbusGetBusState() & (1<<ReceiveCompleted))
	{
		switch(rxbuffer[1]) {
			case fcReadCoilStatus: {
				modbusExchangeBits(&outstate,0,8);
			}
			break;
			
			case fcReadInputStatus: {
				volatile uint8_t inps = ReadIns();
				modbusExchangeBits(&inps,0,8);
			}
			break;
			
			case fcReadHoldingRegisters: {
				modbusExchangeRegisters(holdingRegisters,0,4);
			}
			break;
			
			case fcReadInputRegisters: {
				modbusExchangeRegisters(inputRegisters,0,4);
			}
			break;
			
			case fcForceSingleCoil: {
				modbusExchangeBits(&outstate,0,8);
				SetOuts(outstate);
			}
			break;
			
			case fcPresetSingleRegister: {
				modbusExchangeRegisters(holdingRegisters,0,4);
			}
			break;
			
			case fcForceMultipleCoils: {
				modbusExchangeBits(&outstate,0,8);
				SetOuts(outstate);
			}
			break;
			
			case fcPresetMultipleRegisters: {
				modbusExchangeRegisters(holdingRegisters,0,4);
			}
			break;
			
			default: {
				modbusSendException(ecIllegalFunction);
			}
			break;
		}
	}
}

void InitADC()
{
    // Select Vref=AVcc
    ADMUX |= (1<<REFS0);
    //set prescaller to 128 and enable ADC
    ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}
uint16_t ReadADC(uint8_t ADCchannel)
{
	PORTB ^= (1<<PORTB5);
    //select ADC channel with safety mask
    ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
    //single conversion mode
    ADMUX |= ADCchannel;
    ADCSRA |= (1<<ADSC);
    // wait until ADC conversion is complete
    while( ADCSRA & (1<<ADSC) );
   return ADC;
}

int main(void)
{
	io_conf();
	sei();

	//initialize ADC
	InitADC();
	//Initialize USART0
	//assign our stream to standart I/O streams
	//stdout=&usart0_str;


	modbusSetAddress(clientAddress);
	modbusInit();
    wdt_enable(7);
	timer0100us_start();


    while(1)
    {
		wdt_reset();
	    modbusGet();
	    //loopit();

    }
}
