#ifndef comm_h
#define comm_h

#define	C_KEY				0
#define	COMM_NO			1

#define	COMM_PORT		PORTD
#define	COMM_DIR		_BV(PD4)


#define START_CHAR 		0xe7
#define STOP_CHAR		0xe9
#define DLE				0xf0

#define COM_DIMM		0
#define COM_ONOFF		1

extern unsigned char comm_buf[];
//extern volatile unsigned char comm_cnt;

extern void Send_strn485(unsigned char * buf, unsigned char len);
extern void Display_Prompt(void);
extern signed char check_command(unsigned char *, unsigned char *);
extern signed char String2Hex(unsigned char *);
extern signed char String2Decimal(unsigned char *, char);
extern void send_int_hex(unsigned int );
extern void send_int_dec(unsigned int );
extern void send_CR(void);
extern void send_OK(void);
extern void send_NG(void);
extern void Hex2String(unsigned char hex, unsigned char * to);


#endif
