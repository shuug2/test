#include	<avr/io.h>
#include	<avr/pgmspace.h>
#include	<avr/interrupt.h>
#include	<avr/wdt.h>

#include "macros.h"
#include  "uart.h"
#include  "comm.h"

static const unsigned char newline[] = {"\n\r"};
int	CommData_Flag = 0;


void Display_Prompt(void)
{
	PutStrn("LCD>",4);
}

void send_CR(void)
{
	PutChar((unsigned char)0x0d);
	PutChar((unsigned char)0x0a);
}

void Send_strn485(unsigned char * buf, unsigned char len)
{
	sbi(COMM_PORT, COMM_DIR);	//Tx mede
	PutStrn(buf, len);
	cbi(COMM_PORT, COMM_DIR);	//Rx mede
	
}

int Get_String(unsigned char *get_buf,int max_ch_cnt)
{
	int	cnt = 0;
	int	temp;

	do
	{
		temp = GetChar((unsigned char *)&get_buf[cnt]);

		if(temp > 0)													// Data Input
		{
			if((cnt != 0)&&(get_buf[cnt] == '\r'))							// Input Complete
			{
//				get_buf[cnt] = '\0';										// String_Compare에서 비교시 에러남
				send_CR();										// 줄 바꿈

				return cnt;
			}
			else
				cnt++;

			if(cnt >= max_ch_cnt)
				return cnt;
		}
	} while(1);
}

int String_Compare_n(unsigned char *src,unsigned char *dest, unsigned char no)
{
	int	i = 0;

	for(i = 0; i < no ; i++)
	{
		if(src[i] != dest[i])
			return 0;

	}

	return 1;
}

unsigned char  String_Compare(const unsigned char *src,unsigned char *dest)
{
	int	i = 0;

	while(src[i] != '\r' || dest[i] != '\r')
	{
		if(src[i] != dest[i])
			return 0;

		i++;
	}

	return 1;
}

signed char String2Decimal(unsigned char * str, char size)
{
	unsigned char	temp;

	switch(size)
	{
		case 1:
		{
			if((str[0] >= '0')&&(str[0] <= '9'))
				temp = str[0] - '0';
			else
				return -1;

			break;
		}
		case 2:
		{
			if((str[0] >= '0') && (str[0] <= '9'))
				temp = (str[0] - '0') * 10;
			else
				return -1;

			if((str[1] >= '0') && (str[1] <= '9'))
				temp += (str[1] - '0');
			else
				return -1;

			break;
		}
		case 3:
		{
			if((str[0] >= '0') && (str[0] <= '9'))
				temp = (str[0] - '0') * 100;
			else
				return -1;

			if((str[1] >= '0') && (str[1] <= '9'))
				temp += (str[1] - '0') * 10;
			else
				return -1;

			if((str[2] >= '0') && (str[2] <= '9'))
				temp += (str[2] - '0');
			else
				return -1;

			break;
		}
		case 4:
		{
			if((str[0] >= '0') && (str[0] <= '9'))
				temp = (str[0] - '0') * 1000;
			else
				return -1;

			if((str[1] >= '0') && (str[1] <= '9'))
				temp += (str[1] - '0') * 100;
			else
				return -1;

			if((str[2] >= '0') && (str[2] <= '9'))
				temp += (str[2] - '0') * 10;
			else
				return -1;

			if((str[3] >= '0') && (str[3] <= '9'))
				temp += (str[3] - '0');
			else
				return -1;

			break;
		}
		default:
			break;
	}

    return temp;
}

signed char String2Hex(unsigned char * str)
{
	unsigned char	temp;

	if((str[0] >= '0')&&(str[0] <= '9'))
		temp = str[0] - '0';
	else if((str[0] >= 'A')&&(str[0] <= 'F'))
		temp = str[0] - 'A' + 10;
	else return 0xff;

	temp *= 0x10; 

	if((str[1] >= '0') && (str[1] <= '9'))
		temp += (str[1] - '0');
	else if((str[1] >= 'A')&&(str[1] <= 'F'))
		temp += str[1] - 'A' + 10;
	else return 0xff;

    return temp;
}


void Decimal2String(unsigned int dec, unsigned char * str)
{
	str[0] = (char)(dec / 1000) + 0x30;
	dec %= 1000;

	str[1] = (char)(dec / 100) + 0x30;
	dec %= 100;

	str[2] = (char)(dec / 10) + 0x30;
	dec %= 10;

	str[3] = (char)dec + 0x30;
}

void Hex2String(unsigned char src, unsigned char *dest_str)
{
	unsigned char nibble;
	nibble = (src >> 4) & 0x0f;
	*(dest_str)= (nibble > 9) ? (nibble-10 + 'A') : (nibble+'0');
	nibble = src & 0x0F;
	*(dest_str + 1) = (nibble > 9) ? (nibble-10 + 'A') : (nibble +'0');
	
}

void Make_Upper(unsigned char * str_buffer)
{
	int	i = 0;

	while(1)
	{
		if((str_buffer[i] >= 'a') && (str_buffer[i] <= 'z')){
			str_buffer[i] += ('A' - 'a');
		}else if(str_buffer[i] == STOP_CHAR) break;

		if(++i > 100) break;;
	}

}
