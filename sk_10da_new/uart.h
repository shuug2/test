#define RBUF_SIZE	10
#define TBUF_SIZE	128
#define SEND_DATA_LENTN		2
#define SEND_DATA_LENTN2	2
#define MAX_COMM	10


extern unsigned char rx_data[RBUF_SIZE];
extern unsigned char tx_buf[TBUF_SIZE];

void init_uart(void);

extern void wait_tx_end(void);
extern void PutHex(unsigned char ch);
extern unsigned char GetChar(unsigned char *);
extern void PutChar ( unsigned char  );
extern void PutStr(unsigned char *s);
extern void PutStrn(unsigned char *s, unsigned int len);
extern void PutDigit(unsigned char ch);
extern void PutBCD(unsigned char ch);
extern void PutStrInt(unsigned char *s, unsigned int len, unsigned char * incomm_flag);
