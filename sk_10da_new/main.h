#ifndef __MAIN_H
#define __MAIN_H

#include  "macros.h"
#include "comm.h"
#include "uart.h"
#include "i2c_master.h"

#define TRUE	1
#define FALSE	0

#define HIGH    1
#define LOW     0

#define YES		1
#define NO		0

#define INIT_DDRB	0xd3	// PortB Input:0
#define INIT_PORTB	0x00	//  Make port low
#define INIT_DDRC	0x10	// PortC All Input:
#define INIT_PORTC	0x0f	//  Pull-up PC	
#define INIT_DDRD	0xfa	// PortD all input except PD7
#define INIT_PORTD	0x81	// Make PD0 pull up

#define INIT_OSCCAL	0x00	//
#define INIT_MCUCR	0x00	// B'00000000'	Sleep disable
#define INIT_GIMSK	0x20	// B'00100000'	PC Int.
#define INIT_TIMSK	0x10	// OCIE0A
#define INIT_TCCR0A	((0<<COM0A1)|(0<<COM0A0)|(0<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(0<<WGM00))    // Mode 2,CTC , OCR0A to top
											// clear OCRB on Compare match 
//#define INIT_TCCR0B	((0<<WGM02)|(1<<CS02)|(0<<CS01)|(1<<CS00)) 	// clk(14745600Hz)/1024/78 =  100Hz
//#define INIT_OCR0A	144
#define INIT_TCCR0B	((0<<WGM02)|(1<<CS02)|(0<<CS01)|(1<<CS00)) 	// clk(8000000Hz)/1024/39 =  200Hz
#define INIT_OCR0A	39
#define INIT_OCR0B	24	
#define INIT_TIMSK0	((1<<OCIE0A))
	
#define INIT_TCCR1A	((0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM10))	 //Mode 0, NORMAL
#define INIT_TCCR1B	((1<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(0<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10))	// Clk/64 = 125KHz, ICS Noise canceler On
#define INIT_TCCR1C 	0
#define INIT_OCR1A	0xffff
#define INIT_OCR1B	0xffff
#define INIT_ICR1	0
#define INIT_TIMSK1	((1<<ICIE1))

#define INIT_GTCCR	0x00

#define INIT_TCCR2A	((0<<COM2B1)|(0<<COM2B0)|(1<<WGM21)|(1<<WGM20))		//Mode 7, Fast PWM, OCRA to top 
#define INIT_TCCR2B	((1<<WGM22)|(1<<CS22)|(0<<CS21)|(1<<CS20))		//  Clk/8/128 
#define INIT_OCR2A	100			//
#define INIT_OCR2B	100			//
#define INIT_TCNT2	0
#define INIT_TIMSK2  ((0<<OCIE2A))

#define INIT_ASSR	0		// Make Timer2 to Sync
			
#define INIT_ADCSRA	((0<<ADEN)|(0<<ADSC)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0))
						// ADC Disable , ADC start, interrupt enable,  prescale = 1/128
#define INIT_ADCSRB	((1<<ACME))	    // Analog comaprator mux enable
#define INIT_ADMUX	((0<<REFS1)|(0<<REFS0)|(1<<ADLAR)|0x07)	// Aref is AVCC with ext. cap at AREF, ADC result Left adjustment , Select ADC7
#define INIT_ACSR	((0 << ACD)|(1<<ACBG)|(1<<ACO))		// Analog comparator ON, Fixed Bandgap , Direct connect to ACO

#define INIT_UCSR0A	((0<<U2X0))					// normal
#define INIT_UCSR0B	

#define INIT_EICRA	((1<<ISC01)|(0<<ISC00)|(1<<ISC11)|(0<<ISC10))				// INT0/1 falling edge
//#define INIT_EICRA	((0<<ISC01)|(1<<ISC00))				// INT0 rising edge
#define INIT_EIMSK	((1<<INT0)|(1<<INT1))							// INT 0,1 Enable
#define INIT_PCICR	(1<<PCIE2)
#define INIT_PCMSK0	0
#define INIT_PCMSK1	0
#define INIT_PCMSK2	((0<<PCINT21)|(0<<PCINT22))

// PORT 
#define LOAD_CTRL_PORT		PORTD	//
#define LOAD_CTRL			PD7	//
#define CHK_SW_PORT			PIND
#define CHK_SW				PD2
#define  CHK_SW_CNT			50

#define LED1_PORT			PORTB
#define LED1				PB0
#define FET_ON				1
#define FET_OFF				0


#endif
