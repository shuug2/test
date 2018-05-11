//*********************************************************************
//
//	1CH_DVM / MCP3428, ATmega48
//  Fuse : F9 CD D7  
//  OSC : EXTAL 14.7456MHz
//
//	Lunix Co,. LTD.
//
//	16/12/11	TKN		Derived from 4CH_DVM_M88
//	16/12/6		TKN		Derived from PC_slave_M48
//	
//
//*******************************************************************

#include	<avr/io.h>
#include	<avr/pgmspace.h>
#include	<avr/interrupt.h>
#include	<avr/wdt.h>
#include	<avr/eeprom.h>

#include "main.h"

#define VER	"4CH_DVM_V01"

#define ON		1
#define OFF		0

#define MODE_RUN	0
#define MODE_CONFIG	1
#define MODE_CALIB	2

//------ I2C
#define I2C_WR_DATA_LENGTH 1
#define I2C_RD_DATA_LENGTH 3

#define ADC_ADDR1			0xd0		//MCP3426
#define ADC_ADDR2			0xd4

// MCP3428 Configuration
#define START_CONVERSION	0x80
#define CH1					0x00
#define CH2					0x20
#define CH3					0x40
#define CH4					0x60
#define CONV_MODE_CONT		0x10
#define CONV_MODE_OS		0x00
#define SPS_240_12_BIT		0x00
#define SPS_60_14_BIT		0x04
#define SPS_15_16_BIT		0x08
#define PGA_X1				0x00
#define PGA_X2				0x01
#define PGA_X4				0x02
#define PGA_X8				0x03

#define ADC_RES				8192
#define ADC_V_SCALE			16
#define ADC_I_GAIN			8
#define ADC_RS				500
#define ADC_VREF			2048

//INA226
#define ADC_ADDR0			0x80		//INA226

#define ADC_CONFIG_REG			0x00
#define ADC_SHUNT_VOLTAGE_REG	0x01
#define ADC_BUS_VOLTAGE_REG		0x02
#define ADC_POWER_REG			0x03
#define ADC_CURRENT_REG			0x04
#define ADC_CALIBRATION_REG		0x05
#define ADC_MASK_ENABLE_REG		0x06
#define ADC_ALERT_LIMIT_REG		0x07
#define ADC_MAN_ID_REG			0xfe
#define ADC_DIE_ID_REG			0xff

#define I_OFFSET				3700
#define SK10D

#ifdef SK10D
#define VIN_MIN		2240
#define VIN_MAX		2300
#define I_MIN		1500
#define I_MAX		4500
#define V8_5_MIN	760
#define V8_5_MAX	820
#define V5_3_MIN	480
#define V5_3_MAX	510
#endif

// AVG BIT SETTING	(bit 11,10,9)
#define ADC_CONF_AVG_1		0	// default
#define ADC_CONF_AVG_4		1
#define ADC_CONF_AVG_16		2
#define ADC_CONF_AVG_64		3
#define ADC_CONF_AVG_128	4
#define ADC_CONF_AVG_256	5
#define ADC_CONF_AVG_512	6
#define ADC_CONF_AVG_1024	7
// BUS VOLTAGE CONVERSION TIME	(bit 8,7,6)
#define ADC_CONF_VBUSCT_0	0	// 140us
#define ADC_CONF_VBUSCT_1	1	// 204us
#define ADC_CONF_VBUSCT_2	2	// 332us
#define ADC_CONF_VBUSCT_3	3	// 588us
#define ADC_CONF_VBUSCT_4	4	// 1.1ms	default
#define ADC_CONF_VBUSCT_5	5	// 2.116ms
#define ADC_CONF_VBUSCT_6	6	// 4.156ms
#define ADC_CONF_VBUSCT_7	7	// 8.244ms
// SHUNT VOLTAGE CONVERSION TIME	(bit 5,4,3)
#define ADC_CONF_VSHSCT_0	0	// 140us
#define ADC_CONF_VSHSCT_1	1	// 204us
#define ADC_CONF_VSHSCT_2	2	// 332us
#define ADC_CONF_VSHSCT_3	3	// 588us
#define ADC_CONF_VSHSCT_4	4	// 1.1ms	default
#define ADC_CONF_VSHSCT_5	5	// 2.116ms
#define ADC_CONF_VSHSCT_6	6	// 4.156ms
#define ADC_CONF_VSHSCT_7	7	// 8.244ms
// OPERATING MODE (bit 2,1,0)
#define ADC_CONF_MODE_POWERDOWN1		0
#define ADC_CONF_MODE_SHUNT_TRIGGER		1
#define ADC_CONF_MODE_BUS_TRIGGER		2
#define ADC_CONF_MODE_ALL_TRIGGER		3
#define ADC_CONF_MODE_POWERDOWN2		4
#define ADC_CONF_MODE_SHUNT_CONTINUE	5
#define	ADC_CONF_MODE_BUS_CONTINUE		6
#define	ADC_CONF_MODE_ALL_CONTINUE		7 // default


//--- comm
#define SYNC1		0x5a
#define SYNC2		0xa5

#define LCD_COMM_N	0
#define LCD_COMM_MODE	1
#define LCD_COMM_ADDR_H	2
#define LCD_COMM_ADDR_L	3
#define LCD_COMM_DATA_N	4
#define LCD_COMM_DATA_H	5
#define LCD_COMM_DATA_L	6

//---- LCD VP address
#define VP_CH1_0	0x00
#define VP_CH1_1	0x01
#define VP_CH1_2	0x02
#define VP_CH1_3	0x03
#define VP_CH2_0	0x10
#define VP_CH2_1	0x11
#define VP_CH2_2	0x12
#define VP_CH2_3	0x13
#define VP_CH3_0	0x20
#define VP_CH3_1	0x21
#define VP_CH3_2	0x22
#define VP_CH3_3	0x23
#define VP_CH4_0	0x30
#define VP_CH4_1	0x31
#define VP_CH4_2	0x32
#define VP_CH4_3	0x33

#define VP_KEY_CONF	0x001e

#define VP_MIN_IIN	0x0100
#define VP_MIN_226	0x0101
#define VP_MIN_8	0x0102
#define VP_MIN_5	0x0103
#define VP_MAX_IIN	0x0110
#define VP_MAX_226	0x0111
#define VP_MAX_8	0x0112
#define VP_MAX_5	0x0113
#define VP_DELAY	0x0120

#define VP_KEY_IIN_MIN	0x0130
#define VP_KEY_226_MIN	0x0131
#define VP_KEY_8_MIN	0x0132
#define VP_KEY_5_MIN	0x0133
#define VP_KEY_DELAY	0x0134
#define VP_KEY_IIN_MAX	0x0135
#define VP_KEY_226_MAX	0x0136
#define VP_KEY_8_MAX	0x0137
#define VP_KEY_5_MAX	0x0138
#define VP_KEY_QUIT	0x0139

#define VP_LED_IIN	0x0041
#define VP_LED_V24	0x0042
#define VP_LED_V5	0x0043
#define VP_LED_V33	0x0044


#define KEY_UP		1
#define KEY_DN		0

#define KEY_CANCEL	0
#define KEY_SAVE	1

//---- LCD SP address
#define SP_CH1_3	0x5010
#define SP_CH1_0	0x5020
#define SP_CH1_1	0x5030
#define SP_CH2_0	0x5040

#define SP_OFFSET_COLOR	3

#define RED_H	0xf8
#define RED_L	0x00
#define GREEN_H		0x07
#define GREEN_L		0xe0


#define LCD_RD	0x83
#define LCD_WR	0x82

#define RD_OFFSET_H	5
#define RD_OFFSET_L	6

//-------------
#define LIMIT_IIN	0
#define LIMIT_226	1
#define LIMIT_8		2
#define LIMIT_5		3

#define EEP_IIN_MIN	0
#define EEP_226_MIN	2
#define EEP_8_MIN	4
#define EEP_5_MIN	6
#define EEP_IIN_MAX	8
#define EEP_226_MAX	10
#define EEP_8_MAX	12
#define EEP_5_MAX	14
#define EEP_CAL00	16
#define EEP_CAL01	18
#define EEP_CAL02	20
#define EEP_CAL03	22
#define EEP_CAL20	24
#define EEP_DELAY	100

unsigned char adc_read_time; /*  detect zero_cross */
unsigned char adc_updated; /*  */
unsigned char in_i2c_comm;
unsigned char in_uart_comm;
unsigned char uart_updated;
unsigned char event_on; /*  */
unsigned char wait_reply; /*  */

volatile unsigned char curr_dimm;

volatile unsigned char tick;
volatile unsigned char state_cnt;
volatile unsigned char comm_tick;
volatile unsigned char delay_cnt;



unsigned char back_tick, sec_cnt, sec, todo;
union {
	signed int c_int;
	unsigned char c_byte[2];
} conv;

union {
	signed long c_long;
	unsigned char c_byte[4];
} conv_l;

//----------------
unsigned char default_v_conf, adc_ch_cnt, adc_buf_cnt;
unsigned char i2c_comm_data[4];
signed int adc_buf1[4][8];
signed int adc_buf2[4][8];
signed int adc_buf_i[128];
signed int adc_buf_v[8];

signed int cal_buf1[4], cal_buf2[4], cal_buf0[2];

signed int adc_res[4], adc_v_scale, adc_i_gain[4];
signed int adc_rs[4], adc_vref;
unsigned int adc_result[3][4];
//-----------------------------
unsigned char t_ch10, t_ch11;
unsigned char lcd_buf[64], temp_buf[64], lcd_cnt, lcd_length;
unsigned char send_buf[64];
signed char temp1, temp2;
unsigned char sw_cnt, jig_status, load_status;
unsigned char mode;

signed int limit_min[4], limit_max[4];
unsigned char delay;




unsigned char ver[] = VER;
unsigned char *ver_ptr ; 





//#pragma vector = TIMER0_COMPA_vect
//__interrupt void TIMER0_COMPA_interrupt(void)
ISR(TIMER0_COMPA_vect)
{
	tick++;
//	state_cnt++;
//	if(state_cnt > 7) state_cnt = 0;
	event_on = 1;
	if(comm_tick < 0xff) comm_tick++;
	if(delay_cnt < 0xff) delay_cnt++;
}

//#pragma vector = TIMER1_CAPT_vect
//__interrupt void TIM1_CAPT_interrupt(void)
ISR(TIMER1_CAPT_vect)
{
}

//#pragma vector = TIMER1_COMPA_vect
//__interrupt void TIMER1_COMPA_interrupt(void)
ISR(TIMER1_COMPA_vect)
{
}

//#pragma vector = TIMER1_COMPB_vect
//__interrupt void TIMER1_COMPB_interrupt(void)
ISR(TIMER1_COMPB_vect)
{
}

//#pragma vector = TIMER2_COMPA_vect
//__interrupt void TIMER2_COMPA_interrupt(void)
ISR(TIMER2_COMPA_vect)
{
}

//#pragma vector = INT0_vect
//__interrupt void INT0_interrupt(void)	
ISR(INT0_vect)
{
}

//#pragma vector = INT1_vect
//__interrupt void INT1_interrupt(void)
ISR(INT1_vect)
{
}

//#pragma vector = ADC_vect
//__interrupt void ADC_interrupt(void)
ISR(ADC_vect)
{
}

unsigned char read_eeprom(unsigned int add)
{
/*	while (tbi(EECR, EEPE));		// Delay for eeprom action end
	EEAR = add;
	sbi(EECR, EERE);				// Read start
	return EEDR;
*/
	return eeprom_read_byte((const uint8_t *)add);
}

void write_eeprom(unsigned int add, unsigned char data)
{
/*	while (tbi(EECR, EEPE));		// Delay for eeprom action end
	EEAR = add;
	EEDR = data;
	cli();
	EECR |= 0x04;				// Set Master wirte enable
	EECR |= 0x02;				// Write start
	sei();
*/
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t *)add, (uint8_t)data);	
}

void save_ref_val(void)
{
	unsigned char i;

	conv.c_int = limit_min[LIMIT_IIN];
	write_eeprom(EEP_IIN_MIN, conv.c_byte[0]);
	write_eeprom(EEP_IIN_MIN + 1, conv.c_byte[1]);
	wdt_reset();
	conv.c_int = limit_min[LIMIT_226];
	write_eeprom(EEP_226_MIN, conv.c_byte[0]);
	write_eeprom(EEP_226_MIN + 1, conv.c_byte[1]);
	wdt_reset();
	conv.c_int = limit_min[LIMIT_8];
	write_eeprom(EEP_8_MIN, conv.c_byte[0]);
	write_eeprom(EEP_8_MIN + 1, conv.c_byte[1]);
	wdt_reset();
	conv.c_int = limit_min[LIMIT_5];
	write_eeprom(EEP_5_MIN, conv.c_byte[0]);
	write_eeprom(EEP_5_MIN + 1, conv.c_byte[1]);
	wdt_reset();
	conv.c_int = limit_max[LIMIT_IIN];
	write_eeprom(EEP_IIN_MAX, conv.c_byte[0]);
	write_eeprom(EEP_IIN_MAX + 1, conv.c_byte[1]);
	wdt_reset();
	conv.c_int = limit_max[LIMIT_226];
	write_eeprom(EEP_226_MAX, conv.c_byte[0]);
	write_eeprom(EEP_226_MAX + 1, conv.c_byte[1]);
	wdt_reset();
	conv.c_int = limit_max[LIMIT_8];
	write_eeprom(EEP_8_MAX, conv.c_byte[0]);
	write_eeprom(EEP_8_MAX + 1, conv.c_byte[1]);
	wdt_reset();
	conv.c_int = limit_max[LIMIT_5];
	write_eeprom(EEP_5_MAX, conv.c_byte[0]);
	write_eeprom(EEP_5_MAX + 1, conv.c_byte[1]);

	write_eeprom(EEP_DELAY, delay);
	wdt_reset();
}

void load_ref_val(void)
{
	unsigned char i;

	conv.c_byte[0] = read_eeprom(EEP_IIN_MIN);
	conv.c_byte[1] = read_eeprom(EEP_IIN_MIN + 1);
	if(conv.c_int == -1){
		limit_min[LIMIT_IIN] = I_MIN;
		limit_min[LIMIT_226] = VIN_MIN;
		limit_min[LIMIT_8] = V8_5_MIN;
		limit_min[LIMIT_5] = V5_3_MIN;
		limit_max[LIMIT_IIN] = I_MAX;
		limit_max[LIMIT_226] = VIN_MAX;
		limit_max[LIMIT_8] = V8_5_MAX;
		limit_max[LIMIT_5] = V5_3_MAX;
		delay = 50;
		wdt_reset();

		save_ref_val();

		return;
	}
	limit_min[LIMIT_IIN] = conv.c_int;
	wdt_reset();

	conv.c_byte[0] = read_eeprom(EEP_226_MIN);
	conv.c_byte[1] = read_eeprom(EEP_226_MIN + 1);
	limit_min[LIMIT_226] = conv.c_int;
	wdt_reset();
	conv.c_byte[0] = read_eeprom(EEP_8_MIN);
	conv.c_byte[1] = read_eeprom(EEP_8_MIN + 1);
	limit_min[LIMIT_8] = conv.c_int;
	wdt_reset();
	conv.c_byte[0] = read_eeprom(EEP_5_MIN);
	conv.c_byte[1] = read_eeprom(EEP_5_MIN + 1);
	limit_min[LIMIT_5] = conv.c_int;
	wdt_reset();
	conv.c_byte[0] = read_eeprom(EEP_IIN_MAX);
	conv.c_byte[1] = read_eeprom(EEP_IIN_MAX + 1);
	limit_max[LIMIT_IIN] = conv.c_int;
	wdt_reset();

	conv.c_byte[0] = read_eeprom(EEP_226_MAX);
	conv.c_byte[1] = read_eeprom(EEP_226_MAX + 1);
	limit_max[LIMIT_226] = conv.c_int;
	wdt_reset();
	conv.c_byte[0] = read_eeprom(EEP_8_MAX);
	conv.c_byte[1] = read_eeprom(EEP_8_MAX + 1);
	limit_max[LIMIT_8] = conv.c_int;
	wdt_reset();
	conv.c_byte[0] = read_eeprom(EEP_5_MAX);
	conv.c_byte[1] = read_eeprom(EEP_5_MAX + 1);
	limit_max[LIMIT_5] = conv.c_int;
	wdt_reset();

	delay = read_eeprom(EEP_DELAY);
	
//	adc_v_scale = ADC_V_SCALE;
//	adc_vref = ADC_VREF;

//	cal_buf1[0] = 1451;		// 22.6V
//	cal_buf1[1] = 900;		// 8V
//	cal_buf1[2] = 1072;		// 24V
//	cal_buf1[3] = 251;		// I in
//	cal_buf2[0] = 1337;		// 5V

	cal_buf1[0] = 1024;		// 22.6V
	cal_buf1[1] = 1024;		// 8V
	cal_buf1[2] = 1024;		// 5V

	cal_buf0[0] = 5000;		// I in
	cal_buf0[1] = 125;		// V in
}



int __low_level_init(void)
{
	MCUCR = INIT_MCUCR;	

	MCUSR &= ~(1<<WDRF);
	WDTCSR |= ((1<<WDCE)|(1<<WDE));
	WDTCSR = (1<<WDE) | (1<<WDP2) | (1<<WDP0);    // 64K = 0.5

	DDRB = INIT_DDRB;
	PORTB = INIT_PORTB;
	DDRC = INIT_DDRC;
	PORTC = INIT_PORTC;
	DDRD = INIT_DDRD;
	PORTD = INIT_PORTD;

	ACSR = INIT_ACSR;	
	ADCSRA = INIT_ADCSRA;
	
	TCCR0A = INIT_TCCR0A;
	TCCR0B = INIT_TCCR0B;
	OCR0A = INIT_OCR0A;
	OCR0B = INIT_OCR0B;
	TIMSK0 = INIT_TIMSK0;
	
	TCCR1A = INIT_TCCR1A;
	TCCR1B = INIT_TCCR1B;
	TCCR1C = INIT_TCCR1C;
	OCR1A = INIT_OCR1A;
	OCR1B = INIT_OCR1B;
	ICR1 = INIT_ICR1;
	TIMSK1 = INIT_TIMSK1;

	GTCCR = INIT_GTCCR;

	ASSR = INIT_ASSR;
	TCCR2A = INIT_TCCR2A;
	TCCR2B = INIT_TCCR2B;
	OCR2A = INIT_OCR2A;
	OCR2B = INIT_OCR2B;
	TCNT2 = INIT_TCNT2;
	TIMSK2 = INIT_TIMSK2;

	UCSR0A = INIT_UCSR0A;
	

	EICRA = INIT_EICRA;
	EIMSK = INIT_EIMSK;

	PCICR = INIT_PCICR;
	PCMSK0 = INIT_PCMSK0;
	PCMSK1 = INIT_PCMSK1;
	PCMSK2 = INIT_PCMSK2;
	
	ADMUX = INIT_ADMUX;
	ADCSRA = INIT_ADCSRA;
	ADCSRB = INIT_ADCSRB;

        return 1;
}



void to_do(void)
{
	unsigned char temp, i, t;
	unsigned int temp_i;

	
	switch(todo){
		default:
				break;
	}
	todo = 0xff;
	
	if(back_tick != tick){	// every 10mS, normally spend 10uS
//		sbi(PORTD, PD4);
		back_tick = tick;
		sec_cnt++;
		if((sec_cnt) %10 == 0)
		{
			adc_read_time = 1;
		}

		if(sec_cnt > 99)
		{
			sec++;
			sec_cnt=0;
		} 
		else
		{
		}
//		cbi(PORTD, PD4);
	}	

	
	
}

void do_comm(void)
{
}

unsigned char check_data(void)
{
}

unsigned char check_comm(void)
{
    unsigned char temp, j;
    unsigned int temp_i;
	return 0;		
}

void init_adc(void)
{
	default_v_conf = (START_CONVERSION | CH1 | CONV_MODE_CONT | SPS_60_14_BIT | PGA_X1);

	/* Send every other packet with reversed data */
	i2c_comm_data[0] = default_v_conf;

	i2c_transmit(ADC_ADDR1, i2c_comm_data, 1);
	i2c_transmit(ADC_ADDR2, i2c_comm_data, 1);
	
	conv.c_int = (ADC_CONF_AVG_1<<9) | (ADC_CONF_VBUSCT_4<<6) | (ADC_CONF_VSHSCT_4<<3) | (ADC_CONF_MODE_ALL_CONTINUE);
	i2c_comm_data[0] = ADC_CONFIG_REG;
	i2c_comm_data[1] = conv.c_byte[1];
	i2c_comm_data[2] = conv.c_byte[0];
	i2c_transmit(ADC_ADDR0, i2c_comm_data, 3);
	
}

void get_adc_data(void)
{
//	sbi(PORTD, PD4);
	i2c_receive(ADC_ADDR1, i2c_comm_data, 3);
	conv.c_byte[0] = i2c_comm_data[1];
	conv.c_byte[1] = i2c_comm_data[0];
	adc_buf1[adc_ch_cnt][(adc_buf_cnt >> 2)] = conv.c_int;	// make MSB to sign bit 
	
	i2c_receive(ADC_ADDR2, i2c_comm_data, 3);
	conv.c_byte[0] = i2c_comm_data[1];
	conv.c_byte[1] = i2c_comm_data[0];
	adc_buf2[adc_ch_cnt][(adc_buf_cnt >> 2)] = conv.c_int;	// make MSB to sign bit 
	
//	if(adc_ch_cnt < 3)
//	{
//		adc_ch_cnt ++;
//	}
//	else 
//	{
		i2c_comm_data[0] = ADC_SHUNT_VOLTAGE_REG;
		i2c_transmit(ADC_ADDR0, i2c_comm_data, 1);
		i2c_receive(ADC_ADDR0, i2c_comm_data, 2);
		conv.c_byte[0] = i2c_comm_data[1];
		conv.c_byte[1] = i2c_comm_data[0];
		adc_buf_i[adc_buf_cnt] = conv.c_int;	// make MSB to sign bit 

		i2c_comm_data[0] = ADC_BUS_VOLTAGE_REG;
		i2c_transmit(ADC_ADDR0, i2c_comm_data, 1);
		i2c_receive(ADC_ADDR0, i2c_comm_data, 2);
		conv.c_byte[0] = i2c_comm_data[1];
		conv.c_byte[1] = i2c_comm_data[0];
		adc_buf_v[(adc_buf_cnt >> 2)] = conv.c_int;	// make MSB to sign bit 

//		adc_ch_cnt = 0;
		adc_updated = 1;
		if(adc_buf_cnt < 31)
			adc_buf_cnt++;
		else
			adc_buf_cnt = 0;	
		adc_ch_cnt = adc_buf_cnt & 0x03;
//	}

	i2c_comm_data[0] = default_v_conf + ((adc_ch_cnt & 0x07) * 0x20);
	i2c_transmit(ADC_ADDR1, i2c_comm_data, 1);
	i2c_transmit(ADC_ADDR2, i2c_comm_data, 1);
}

void conv_adc_data(void)
{
	unsigned char i, j, min, max;
	signed int *data_ptr, temp_i;
	signed long temp;
	

	sbi(PORTD, PD4);
	for(i = 0 ; i < 4 ; i++)
	{
		data_ptr = &adc_buf1[i][0];
		for(j = 0, temp = 0 ; j < 8 ; j++)
		{
			temp += (signed int)*(data_ptr+j);
		}
		temp = temp * cal_buf1[i];
//		adc_result[0][i] = temp  / 2048;
		adc_result[1][i] = (temp  / 20480);
	}
	
	for(i = 0 ; i < 4 ; i++)
	{
		data_ptr = &adc_buf2[i][0];
		for(j = 0, temp = 0 ; j < 8 ; j++)
		{
			temp += (signed int)*(data_ptr+j);
		}
		temp = temp * cal_buf2[i];
		adc_result[2][i] = (temp  / 20480);
	}

	
	for(j = 0, temp = 0; j < 32 ; j++)
	{
			temp_i = adc_buf_i[j];
			if(temp_i < 0) temp_i = 0;
			
			temp += temp_i;
	}
	temp = (temp/32) * cal_buf0[0];
/*
	temp = adc_buf0[0][0] + adc_buf0[0][1];
	temp = temp / 2;
	adc_buf0[0][1] = (signed int)temp;	
	temp = temp * cal_buf0[0];
*/	
	adc_result[0][0] = (temp / 220);
	if(adc_result[0][0] > I_OFFSET)
		adc_result[0][0] -= I_OFFSET;
	else
		adc_result[0][0] = 0;

	for(j = 0, temp = 0 ; j < 8 ; j++)
	{
		temp += adc_buf_v[j];
	}
	
	temp = temp * cal_buf0[1];
	adc_result[0][1] = (temp / 8000);
	
	cbi(PORTD, PD4);
}

unsigned char check_adc_range(void)
{
	unsigned char temp, i;
	signed long temp_long;

	temp_long = adc_result[0][0];		// Iin
	if((temp_long > limit_max[LIMIT_IIN]) || (temp_long < limit_min[LIMIT_IIN]))
		temp = 1;
	else 
		temp = 0;

	
	send_buf[0] = SYNC1;
	send_buf[1] = SYNC2;
	send_buf[2] = 5;		// 3+ 18data
	send_buf[3] = LCD_WR;
	send_buf[4] = (unsigned char)((SP_CH1_3 + SP_OFFSET_COLOR) >> 8);
	send_buf[5] = (unsigned char)(SP_CH1_3 + SP_OFFSET_COLOR); // start_addr
	if(temp)
	{
		send_buf[6] = RED_H;
		send_buf[7] = RED_L;
	}
	else
	{
		send_buf[6] = GREEN_H;
		send_buf[7] = GREEN_L;
	}
	for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
	send_buf[2] = 5;		// 3+ 18data
	send_buf[3] = LCD_WR;
	send_buf[4] = 0;
	send_buf[5] = VP_LED_IIN; // start_addr
	send_buf[6] = 0;
	if(temp)
	{
		send_buf[7] = 0;
	}
	else
	{
		send_buf[7] = 1;
	}
	for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
	
	
	temp_long = adc_result[1][0];		//22.6
	if((temp_long > limit_max[LIMIT_226]) || (temp_long < limit_min[LIMIT_226]))
		temp = 1;
	else 
		temp = 0;

	
	send_buf[0] = SYNC1;
	send_buf[1] = SYNC2;
	send_buf[2] = 5;		// 3+ 18data
	send_buf[3] = LCD_WR;
	send_buf[4] = (unsigned char)((SP_CH1_0 + SP_OFFSET_COLOR) >> 8);
	send_buf[5] = (unsigned char)(SP_CH1_0 + SP_OFFSET_COLOR); // start_addr
	if(temp)
	{
		send_buf[6] = RED_H;
		send_buf[7] = RED_L;
	}
	else
	{
		send_buf[6] = GREEN_H;
		send_buf[7] = GREEN_L;
	}
	for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
	send_buf[2] = 5;		// 3+ 18data
	send_buf[3] = LCD_WR;
	send_buf[4] = 0;
	send_buf[5] = VP_LED_V24; // start_addr
	send_buf[6] = 0;
	if(temp)
	{
		send_buf[7] = 0;
	}
	else
	{
		send_buf[7] = 1;
	}
	for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
	
	temp_long = adc_result[1][1];		//8
	if((temp_long > limit_max[LIMIT_8]) || (temp_long < limit_min[LIMIT_8]))
		temp = 1;
	else 
		temp = 0;

	
	send_buf[0] = SYNC1;
	send_buf[1] = SYNC2;
	send_buf[2] = 5;		// 3+ 18data
	send_buf[3] = LCD_WR;
	send_buf[4] = (unsigned char)((SP_CH1_1 + SP_OFFSET_COLOR) >> 8);
	send_buf[5] = (unsigned char)(SP_CH1_1 + SP_OFFSET_COLOR); // start_addr
	if(temp)
	{
		send_buf[6] = RED_H;
		send_buf[7] = RED_L;
	}
	else
	{
		send_buf[6] = GREEN_H;
		send_buf[7] = GREEN_L;
	}
	for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
	send_buf[2] = 5;		// 3+ 18data
	send_buf[3] = LCD_WR;
	send_buf[4] = 0;
	send_buf[5] = VP_LED_V5; // start_addr
	send_buf[6] = 0;
	if(temp)
	{
		send_buf[7] = 0;
	}
	else
	{
		send_buf[7] = 1;
	}
	for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);

	temp_long = adc_result[1][2];	//5
	if((temp_long >limit_max[LIMIT_5]) || (temp_long < limit_min[LIMIT_5]))
		temp = 1;
	else 
		temp = 0;


	send_buf[0] = SYNC1;
	send_buf[1] = SYNC2;
	send_buf[2] = 5;		// 3+ 18data
	send_buf[3] = LCD_WR;
	send_buf[4] = (unsigned char)((SP_CH2_0 + SP_OFFSET_COLOR) >> 8);
	send_buf[5] = (unsigned char)(SP_CH2_0 + SP_OFFSET_COLOR); // start_addr
	if(temp)
	{
		send_buf[6] = RED_H;
		send_buf[7] = RED_L;
	}
	else
	{
		send_buf[6] = GREEN_H;
		send_buf[7] = GREEN_L;
	}
	for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
	send_buf[2] = 5;		// 3+ 18data
	send_buf[3] = LCD_WR;
	send_buf[4] = 0;
	send_buf[5] = VP_LED_V33; // start_addr
	send_buf[6] = 0;
	if(temp)
	{
		send_buf[7] = 0;
	}
	else
	{
		send_buf[7] = 1;
	}
	for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);

	
}

void send_limit_data(void)
{
	unsigned char i;

	
	send_buf[0] = SYNC1;
	send_buf[1] = SYNC2;
	send_buf[2] = 11;		// 3+ 18data
	send_buf[3] = LCD_WR;
	send_buf[4] = (unsigned char)((VP_MIN_IIN) >> 8);
	send_buf[5] = (unsigned char)(VP_MIN_IIN); // start_addr
	conv.c_int = (signed long)limit_min[LIMIT_IIN];
	send_buf[6] = conv.c_byte[1];
	send_buf[7] = conv.c_byte[0];
	conv.c_int = (signed long)limit_min[LIMIT_226];
	send_buf[8] = conv.c_byte[1];
	send_buf[9] = conv.c_byte[0];
	conv.c_int = (signed long)limit_min[LIMIT_8];
	send_buf[10] = conv.c_byte[1];
	send_buf[11] = conv.c_byte[0];
	conv.c_int = (signed long)limit_min[LIMIT_5];
	send_buf[12] = conv.c_byte[1];
	send_buf[13] = conv.c_byte[0];
	
	wdt_reset();
	for(i = 0 ; i < 14 ; i++) PutChar(send_buf[i]);

	send_buf[0] = SYNC1;
	send_buf[1] = SYNC2;
	send_buf[2] = 11;		// 3+ 18data
	send_buf[3] = LCD_WR;
	send_buf[4] = (unsigned char)((VP_MAX_IIN) >> 8);
	send_buf[5] = (unsigned char)(VP_MAX_IIN); // start_addr
	conv.c_int = limit_max[LIMIT_IIN];
	send_buf[6] = conv.c_byte[1];
	send_buf[7] = conv.c_byte[0];
	conv.c_int = limit_max[LIMIT_226];
	send_buf[8] = conv.c_byte[1];
	send_buf[9] = conv.c_byte[0];
	conv.c_int = limit_max[LIMIT_8];
	send_buf[10] = conv.c_byte[1];
	send_buf[11] = conv.c_byte[0];
	conv.c_int = limit_max[LIMIT_5];
	send_buf[12] = conv.c_byte[1];
	send_buf[13] = conv.c_byte[0];
	for(i = 0 ; i < 14 ; i++) PutChar(send_buf[i]);

	wdt_reset();
	
	send_buf[0] = SYNC1;
	send_buf[1] = SYNC2;
	send_buf[2] = 5;		// 3+ 18data
	send_buf[3] = LCD_WR;
	send_buf[4] = (unsigned char)((VP_DELAY) >> 8);
	send_buf[5] = (unsigned char)(VP_DELAY); // start_addr
	conv.c_int = (signed long)delay;
	send_buf[6] = 0;
	send_buf[7] = delay;
	for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
}

unsigned char check_lcd_comm(void)
{
	unsigned char dummy, i;

	dummy = GetChar((unsigned char *)&t_ch10);
	if(dummy){
		if(in_uart_comm == 0){
			if((t_ch10 == SYNC2)&&(t_ch11 == SYNC1)){
				in_uart_comm = 1;
				lcd_length = 6;
				comm_tick = 0;
			} else {
				t_ch11 = t_ch10;
			}
		} else {
			temp_buf[lcd_cnt] = t_ch10;
			if(lcd_cnt == LCD_COMM_DATA_N){
				lcd_length = (t_ch10 << 1)+4;
//			if(com1_cnt == 0){
//				com1_length = (t_ch10 + 1);
			} else{
				if(lcd_cnt == lcd_length){
					lcd_cnt = 0;
					in_uart_comm = 0;
					for(i = 0 ; i <= lcd_length ; i++){
						lcd_buf[i] = temp_buf[i];
					}
					return 1;
				}
			}
			lcd_cnt++;
			if(comm_tick > 10){	// time out
				lcd_cnt = 0;
				in_uart_comm = 0;
			}
		}
	}
	return 0;		
}

void send_adc_data(void)
{
	unsigned char i;
//	LED2 = 1;	


	send_buf[0] = SYNC1;
	send_buf[1] = SYNC2;
	send_buf[2] = 11;		// 3+ 18data
	send_buf[3] = LCD_WR;
	send_buf[4] = 0;
	send_buf[5] = VP_CH1_0;	// start_addr
	conv.c_int = adc_result[0][1];	// Vin
	send_buf[6] = conv.c_byte[1];
	send_buf[7] = conv.c_byte[0];
	conv.c_int = adc_result[0][0];	//Iin
	send_buf[8] = conv.c_byte[1];
	send_buf[9] = conv.c_byte[0];
	conv.c_int = adc_result[1][0];	//22.6
	send_buf[10] = conv.c_byte[1];
	send_buf[11] = conv.c_byte[0];
	conv.c_int = adc_result[1][1];	//8
	send_buf[12] = conv.c_byte[1];
	send_buf[13] = conv.c_byte[0];
	for(i = 0 ; i < 14 ; i++)
	{
		 PutChar(send_buf[i]);
		 wdt_reset();
	}

	send_buf[5] = VP_CH2_0; // start_addr
	conv.c_int = adc_result[1][2];	//5
	send_buf[6] = conv.c_byte[1];
	send_buf[7] = conv.c_byte[0];
	conv.c_int = adc_result[2][0];
	send_buf[8] = conv.c_byte[1];
	send_buf[9] = conv.c_byte[0];
	conv.c_int = adc_result[2][1];
	send_buf[10] = conv.c_byte[1];
	send_buf[11] = conv.c_byte[0];
	conv.c_int = adc_result[2][2];
	send_buf[12] = conv.c_byte[1];
	send_buf[13] = conv.c_byte[0];
	for(i = 0 ; i < 14 ; i++) 
	{
		 PutChar(send_buf[i]);
		 wdt_reset();
	}
	
/*

send_buf[0] = SYNC1;
send_buf[1] = SYNC2;
send_buf[2] = 18;		// 3+ 18data
send_buf[3] = LCD_WR;
send_buf[4] = 0;
send_buf[5] = VP_CH1_0; // start_addr
conv.c_int = adc_result[0][0];
send_buf[6] = conv.c_byte[3];
send_buf[7] = conv.c_byte[2];
send_buf[8] = conv.c_byte[1];
send_buf[9] = conv.c_byte[0];
conv.c_int = adc_result[0][1];
send_buf[10] = conv.c_byte[3];
send_buf[11] = conv.c_byte[2];
send_buf[12] = conv.c_byte[1];
send_buf[13] = conv.c_byte[0];
conv.c_int = adc_result[0][2];
send_buf[14] = conv.c_byte[3];
send_buf[15] = conv.c_byte[2];
send_buf[16] = conv.c_byte[1];
send_buf[17] = conv.c_byte[0];
conv.c_int = adc_result[0][3];
send_buf[18] = conv.c_byte[3];
send_buf[19] = conv.c_byte[2];
send_buf[20] = conv.c_byte[1];
send_buf[21] = conv.c_byte[0];
for(i = 0 ; i < 21 ; i++) PutChar(send_buf[i]);
*/
	
}

void pharse_lcd_comm(void)
{
	unsigned int temp_com;
	unsigned char i;

	if(lcd_buf[LCD_COMM_MODE] == LCD_RD){
		if(wait_reply == YES){
//			get_ref_data();
			wait_reply = NO;
			return;
		}
		
		if(lcd_buf[LCD_COMM_N] != 6) return;
		conv.c_byte[0] = lcd_buf[LCD_COMM_ADDR_L];
		conv.c_byte[1] = lcd_buf[LCD_COMM_ADDR_H];
		temp_com = conv.c_int;
		
		if(temp_com == VP_KEY_CONF){				// into configuration
			if(lcd_buf[LCD_COMM_DATA_L] == ON){	// On
				mode = MODE_CONFIG;
				send_limit_data();
			}		
		} else if(temp_com == VP_KEY_QUIT){				// Humidifier control
			if(lcd_buf[LCD_COMM_DATA_L] == KEY_SAVE){	// On
				save_ref_val();
			} else {							// Off
			}		
			mode = MODE_RUN;
			send_adc_data();
		} else if(temp_com == VP_KEY_IIN_MIN){				// Humidifier control
			if(lcd_buf[LCD_COMM_DATA_L] == KEY_DN){	// Down
				if(limit_min[LIMIT_IIN] > 10 ) limit_min[LIMIT_IIN] = limit_min[LIMIT_IIN] - 10 ;
			} else {							// Dn
				if(limit_min[LIMIT_IIN] < 29990 ) limit_min[LIMIT_IIN] = limit_min[LIMIT_IIN] + 10 ;
			}		
			send_buf[0] = SYNC1;
			send_buf[1] = SYNC2;
			send_buf[2] = 5;		// 3+ 18data
			send_buf[3] = LCD_WR;
			send_buf[4] = (unsigned char)((VP_MIN_IIN) >> 8);
			send_buf[5] = (unsigned char)(VP_MIN_IIN); // start_addr
			conv.c_int = limit_min[LIMIT_IIN];
			send_buf[6] = conv.c_byte[1];
			send_buf[7] = conv.c_byte[0];
			wdt_reset();
			for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
		} else if(temp_com == VP_KEY_226_MIN){				// Humidifier control
			if(lcd_buf[LCD_COMM_DATA_L] == KEY_DN){	// Down
				if(limit_min[LIMIT_226] > 10 ) limit_min[LIMIT_226] = limit_min[LIMIT_226] - 10 ;
			} else {							// Dn
				if(limit_min[LIMIT_226] < 29990 ) limit_min[LIMIT_226] = limit_min[LIMIT_226] + 10 ;
			}		
			send_buf[0] = SYNC1;
			send_buf[1] = SYNC2;
			send_buf[2] = 5;		// 3+ 18data
			send_buf[3] = LCD_WR;
			send_buf[4] = (unsigned char)((VP_MIN_226) >> 8);
			send_buf[5] = (unsigned char)(VP_MIN_226); // start_addr
			conv.c_int = limit_min[LIMIT_226];
			send_buf[6] = conv.c_byte[1];
			send_buf[7] = conv.c_byte[0];
			wdt_reset();
			for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
		}else if(temp_com == VP_KEY_8_MIN){				// Humidifier control
			if(lcd_buf[LCD_COMM_DATA_L] == KEY_DN){	// Down
				if(limit_min[LIMIT_8] > 10 ) limit_min[LIMIT_8] = limit_min[LIMIT_8] - 10 ;
			} else {							// Dn
				if(limit_min[LIMIT_8] < 29990 ) limit_min[LIMIT_8] = limit_min[LIMIT_8] + 10 ;
			}		
			send_buf[0] = SYNC1;
			send_buf[1] = SYNC2;
			send_buf[2] = 5;		// 3+ 18data
			send_buf[3] = LCD_WR;
			send_buf[4] = (unsigned char)((VP_MIN_8) >> 8);
			send_buf[5] = (unsigned char)(VP_MIN_8); // start_addr
			conv.c_int = limit_min[LIMIT_8];
			send_buf[6] = conv.c_byte[1];
			send_buf[7] = conv.c_byte[0];
			wdt_reset();
			for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
		}else if(temp_com == VP_KEY_5_MIN){				// Humidifier control
			if(lcd_buf[LCD_COMM_DATA_L] == KEY_DN){	// Down
				if(limit_min[LIMIT_5] > 10 ) limit_min[LIMIT_5] = limit_min[LIMIT_5] - 10 ;
			} else {							// Dn
				if(limit_min[LIMIT_5] < 29990 ) limit_min[LIMIT_5] = limit_min[LIMIT_5] + 10 ;
			}		
			send_buf[0] = SYNC1;
			send_buf[1] = SYNC2;
			send_buf[2] = 5;		// 3+ 18data
			send_buf[3] = LCD_WR;
			send_buf[4] = (unsigned char)((VP_MIN_5) >> 8);
			send_buf[5] = (unsigned char)(VP_MIN_5); // start_addr
			conv.c_int = limit_min[LIMIT_5];
			send_buf[6] = conv.c_byte[1];
			send_buf[7] = conv.c_byte[0];
			wdt_reset();
			for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
		} else if(temp_com == VP_KEY_IIN_MAX){				// Humidifier control
			if(lcd_buf[LCD_COMM_DATA_L] == KEY_DN){	// Down
				if(limit_max[LIMIT_IIN] > 10 ) limit_max[LIMIT_IIN] = limit_max[LIMIT_IIN] - 10 ;
			} else {							// Dn
				if(limit_max[LIMIT_IIN] < 29990 ) limit_max[LIMIT_IIN] = limit_max[LIMIT_IIN] + 10 ;
			}		
			send_buf[0] = SYNC1;
			send_buf[1] = SYNC2;
			send_buf[2] = 5;		// 3+ 18data
			send_buf[3] = LCD_WR;
			send_buf[4] = (unsigned char)((VP_MAX_IIN) >> 8);
			send_buf[5] = (unsigned char)(VP_MAX_IIN); // start_addr
			conv.c_int = limit_max[LIMIT_IIN];
			send_buf[6] = conv.c_byte[1];
			send_buf[7] = conv.c_byte[0];
			wdt_reset();
			for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
		} else if(temp_com == VP_KEY_226_MAX){				// Humidifier control
			if(lcd_buf[LCD_COMM_DATA_L] == KEY_DN){	// Down
				if(limit_max[LIMIT_226] > 10 ) limit_max[LIMIT_226] = limit_max[LIMIT_226] - 10 ;
			} else {							// Dn
				if(limit_max[LIMIT_226] < 29990 ) limit_max[LIMIT_226] = limit_max[LIMIT_226] + 10 ;
			}		
			send_buf[0] = SYNC1;
			send_buf[1] = SYNC2;
			send_buf[2] = 5;		// 3+ 18data
			send_buf[3] = LCD_WR;
			send_buf[4] = (unsigned char)((VP_MAX_226) >> 8);
			send_buf[5] = (unsigned char)(VP_MAX_226); // start_addr
			conv.c_int = limit_max[LIMIT_226];
			send_buf[6] = conv.c_byte[1];
			send_buf[7] = conv.c_byte[0];
			wdt_reset();
			for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
		}else if(temp_com == VP_KEY_8_MAX){				// Humidifier control
			if(lcd_buf[LCD_COMM_DATA_L] == KEY_DN){	// Down
				if(limit_max[LIMIT_8] > 10 ) limit_max[LIMIT_8] = limit_max[LIMIT_8] - 10 ;
			} else {							// Dn
				if(limit_max[LIMIT_8] < 29990 ) limit_max[LIMIT_8] = limit_max[LIMIT_8] + 10 ;
			}		
			send_buf[0] = SYNC1;
			send_buf[1] = SYNC2;
			send_buf[2] = 5;		// 3+ 18data
			send_buf[3] = LCD_WR;
			send_buf[4] = (unsigned char)((VP_MAX_8) >> 8);
			send_buf[5] = (unsigned char)(VP_MAX_8); // start_addr
			conv.c_int = limit_max[LIMIT_8];
			send_buf[6] = conv.c_byte[1];
			send_buf[7] = conv.c_byte[0];
			wdt_reset();
			for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
		}else if(temp_com == VP_KEY_5_MAX){				// Humidifier control
			if(lcd_buf[LCD_COMM_DATA_L] == KEY_DN){	// Down
				if(limit_max[LIMIT_5] > 10 ) limit_max[LIMIT_5] = limit_max[LIMIT_5] - 10 ;
			} else {							// Dn
				if(limit_max[LIMIT_5] < 29990 ) limit_max[LIMIT_5] = limit_max[LIMIT_5] + 10 ;
			}		
			send_buf[0] = SYNC1;
			send_buf[1] = SYNC2;
			send_buf[2] = 5;		// 3+ 18data
			send_buf[3] = LCD_WR;
			send_buf[4] = (unsigned char)((VP_MAX_5) >> 8);
			send_buf[5] = (unsigned char)(VP_MAX_5); // start_addr
			conv.c_int = limit_max[LIMIT_5];
			send_buf[6] = conv.c_byte[1];
			send_buf[7] = conv.c_byte[0];
			wdt_reset();
			for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
		}else if(temp_com == VP_KEY_DELAY){				// Humidifier control
			if(lcd_buf[LCD_COMM_DATA_L] == KEY_DN){	// Down
				if(delay > 20 ) delay = delay - 10 ;
			} else {							// Dn
				if(delay < 240 ) delay = delay + 10 ;
			}		
			send_buf[0] = SYNC1;
			send_buf[1] = SYNC2;
			send_buf[2] = 5;		// 3+ 18data
			send_buf[3] = LCD_WR;
			send_buf[4] = (unsigned char)((VP_DELAY) >> 8);
			send_buf[5] = (unsigned char)(VP_DELAY); // start_addr
			send_buf[6] = 0;
			send_buf[7] = delay;
			wdt_reset();
			for(i = 0 ; i < 8 ; i++) PutChar(send_buf[i]);
		}
		
	}
}

void init_param(void)
{
 	back_tick = 0;
	tick = 0;
	ver_ptr = ver;

	todo = 0xff;

	adc_ch_cnt = adc_buf_cnt = 0;

	adc_read_time = 0;
	in_i2c_comm = 0;
	in_uart_comm = 0;
	uart_updated = 0;
	event_on = 0;
	wait_reply = 0;

	sw_cnt = 0;
	jig_status = load_status = 0;

	mode = MODE_RUN;

	load_ref_val();
}

int main(void)
{
	unsigned char temp, led_cnt;
	
	__low_level_init();
	wdt_reset();
	init_uart();
	init_i2c();
	init_param();
	
	sei();
	delay_cnt = 0;
	do{
		if(delay_cnt > 100) break;	//500mS delay
		wdt_reset();
	}while(1);

	init_adc();
	
	led_cnt = 0;

	do{
		wdt_reset();
		//to_do();

		temp = check_lcd_comm();
		if(temp){
			pharse_lcd_comm();
		}
		
		if((event_on == 1) && (mode != MODE_CONFIG))	// 12.5Hz
		{
			event_on = 0;
			
			if(++led_cnt >= 100)
			{
				led_cnt = 0;
				sbi(LED1_PORT, LED1);
			}
			else if(led_cnt == 50)
			{
				cbi(LED1_PORT, LED1);
			}
			if(tbi(CHK_SW_PORT, CHK_SW) == 0)	// JIG DN
			{
				if(sw_cnt < CHK_SW_CNT)sw_cnt++;
			}
			else 			//JIG_UP
			{
				if(sw_cnt > 0)sw_cnt--;
			}
			
			switch(state_cnt)	
			{
				case 0:
					get_adc_data();
					break;
				case 1:
					conv_adc_data();
					break;
				case 2:
					send_adc_data();
					break;
				case 3:
					check_adc_range();
					if((jig_status == 0) && (sw_cnt >= CHK_SW_CNT))
					{
						jig_status = 1;
						delay_cnt = 0;
						load_status = 1;
					}
					else if((jig_status == 1) && (sw_cnt<= (CHK_SW_CNT - 3)))
					{
						jig_status = 0;
						load_status = 0;
					}

					if(load_status == 1)
						cbi(LOAD_CTRL_PORT, LOAD_CTRL);
					else
					{
						if(delay_cnt > delay)
							sbi(LOAD_CTRL_PORT, LOAD_CTRL);
					}
					break;
				case 4:
					get_adc_data();
					break;
				case 5:
					conv_adc_data();
					break;
				case 6:
					send_adc_data();
					break;
				case 7:
					check_adc_range();
					if((jig_status == 0) && (sw_cnt >= CHK_SW_CNT))
					{
						jig_status = 1;
						load_status = 1;
						delay_cnt = 0;
					}
					else if((jig_status == 1) && (sw_cnt <= (CHK_SW_CNT - 3)))
					{
						jig_status = 0;
						load_status = 0;
					}

					if(load_status == 1)
						cbi(LOAD_CTRL_PORT, LOAD_CTRL);
					else 
					{
						if(delay_cnt > delay)
							sbi(LOAD_CTRL_PORT, LOAD_CTRL);
					}
					break;
				default:
					break;
			}
			state_cnt++;
			if(state_cnt > 7) state_cnt = 0;
		}
/*
		if(adc_read_time == 1)
		{
			adc_read_time = 0;
			get_adc_data();
		}
		if(adc_updated == 1)
		{
			adc_updated = 0;
			conv_adc_data();
		}
*/
	}while(1);
}
