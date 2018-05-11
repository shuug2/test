//*********************************************************************
//
//	UART low level functions for ATmega48
//
//*******************************************************************

#include	<avr/io.h>
#include	<avr/pgmspace.h>
#include	<avr/interrupt.h>
#include	<avr/wdt.h>
#include  "macros.h"
#include	"uart.h"

#define	COM0

#ifdef	COM0	// Use USART0
	#define	UDR		UDR0
	#define	UBRRH		UBRR0H
	#define	UBRRL		UBRR0L
	#define	UCSRA		UCSR0A
	#define	UCSRB		UCSR0B
	#define	UCSRC		UCSR0C
#else			// Use USART1
	#define	UDR		UDR1
	#define	UBRRH		UBRR1H
	#define	UBRRL		UBRR1L
	#define	UCSRA		UCSR1A
	#define	UCSRB		UCSR1B
	#define	UCSRC		UCSR1C
#endif

#define UBR_CAL		(CRYSTAL_FRQ/(BAUDRATE*16L-1L))*(100L-ERROR_RATIO)/100L


unsigned char rx_data[RBUF_SIZE];
unsigned char tx_buf[TBUF_SIZE];
unsigned char rx_header,rx_tail;
unsigned char rx_length;

unsigned char tx_flag=0;
//unsigned char packet_ok=0;
//unsigned char wait=0;
//unsigned char eop=0;
//signed char value;
//unsigned char command_1;
unsigned char rx_data_tail=0;
volatile unsigned char tx_len, tx_cnt;
volatile unsigned char * int_tx_p, * incomm_flag, tx_mode;
 
//#pragma vector = USART_RX_vect
//__interrupt void UART_RX_interrupt(void)
ISR(USART_RX_vect)
{
	rx_data[rx_header++] = UDR;
	if(rx_header >= RBUF_SIZE) rx_header=0;
}

/*
#pragma vector = USART_UDRE_vect
__interrupt void USART_UDRE_interrupt(void) 
{
	if(tx_mode){
		UDR = int_tx_p[tx_cnt++];
		if(tx_cnt >= tx_len){
			tx_mode = 0;
			cbi(UCSR0B, UDRIE0);
			sbi(UCSR0B, TXCIE0);
		}
	}
}
*/
//#pragma vector = USART_TX_vect
//__interrupt void UART_TX_interrupt(void) 
ISR(USART_TX_vect)
{
	if(tx_mode){
		UDR = int_tx_p[tx_cnt++];
		if(tx_cnt >= tx_len){
			tx_mode = 0;
		}
	} else {
		tx_flag = 1;
		*incomm_flag = 1; //YES
	}
}


void wait_tx_end(void)
{
	while (!tx_flag); // 
}

void PutChar ( unsigned char ch )
{
	while(1){
		if(tx_flag==1) break;
		 wdt_reset();
	} 
	tx_flag = 0;
	UDR = ch;
}

unsigned char hextab2[] = "0123456789abcdef";

void PutHex(unsigned char ch)
{
	unsigned char nibble;
	nibble = (ch >> 4) & 0x0F;
	PutChar(hextab2[nibble]);
	nibble = ch & 0x0F;
	PutChar(hextab2[nibble]);
}

void PutDigit(unsigned char ch)
{
	unsigned char temp;
	temp = ch / 100;
	PutChar(hextab2[temp]);
	temp = ch % 100;
	PutChar(hextab2[temp/10]);
	PutChar(hextab2[temp%10]);
}

void PutBCD(unsigned char ch)
{
	unsigned char nibble;
	nibble = (ch / 16) & 0x0F;
	PutChar(hextab2[nibble]);
	nibble = (ch % 16) & 0x0F;
	PutChar(hextab2[nibble]);
}

void PutStr(unsigned char *s)
{
	unsigned char i=0;
	while(s[i]!=0) PutChar(s[i++]);  
}

void PutStr_f(unsigned char __attribute__((progmem)) *s)
{
	unsigned char i=0;
	while(s[i]!=0) PutChar(s[i++]);  
}

void PutStrn(unsigned char *s, unsigned int len)
{
	unsigned int i;
	for(i = 0 ; i < len ; i++)
		PutChar(s[i]);  
}

void PutStrInt(unsigned char *s, unsigned int len, unsigned char * flag)
{
	int_tx_p = s;
	tx_len = len;
	tx_cnt = 1;
	tx_flag = 0;
	tx_mode = 1;
	incomm_flag = flag;
//	sbi(UCSRA, TXC);		// generate tx int
	UDR = int_tx_p[0];
}


unsigned char GetChar(unsigned char * ch)
{
	if ( rx_header == rx_tail ) return 0;
	else{
		*ch = (signed int)rx_data[rx_tail++];
		if ( rx_tail >= RBUF_SIZE ) rx_tail = 0;
	}
	return 1;
}

void GetStr(unsigned char *str)
{
    signed int c,i=0;
    do {
        if( (GetChar((unsigned char *)&c))) { str[i++]=(unsigned char)c;  }
    } while(c!=0x0d);
//    putchar(0x0a);
    str[i-1] = '\0';
}

void init_uart(void)
{

	// 19200-N-8-1
	UBRRH = 0x00;
//	UBRRL = 7;		// 115200 @ 14.7456M , x2 = 0
	UBRRL = 8;		// 115200 @ 8M , x2 = 1
//	UBRRL = 12;		// 38400 @ 8M , x2 = 0
//	UBRRL = 25;		// 19200 @ 8M , x2 = 0
//	UBRRL = 51;		// 9600 @ 8M , x2 = 0
//	UBRRL = 103;		// 4800 @ 8M , x2 = 0
//	UBRRL = 207;		// 2400 @ 8M , x2 = 0
//	UBRRL = 23;		// 19200 @ 7.3728M , x2 = 0
//	UBRRL = 47;		// 9600 @ 7.3728M , x2 = 0
//	UBRRL = 95;		// 4800 @ 7.3728M , x2 = 0
//	UBRRL = 191;		// 2400 @ 7.3728M , x2 = 0
//	UBRRL = 12;		// 9600 @ 1M , x2 = 0
//	UBRRL = 12;		// 9600 @ 1M , x2 = 1
//	UCSRA = 0x00;       //x2 =0, USART
	UCSRA = 0x02;       //x2 =1, USART
	UCSRB = 0xd8;		// RX & Tx Enable
	UCSRC = 0x06;		// Async, N-8-1

	tx_flag = 1;           // tx_ready
	rx_header = rx_tail = 0;
	tx_len = tx_cnt = 0;
	tx_mode = 0;
}
