#include "bsp_485.h"

uint16_t  uart_p = 1;
uint8_t   uart_buff[UART_BUFF_SIZE];

/***************** 发送一个字符  **********************/
//使用单字节数据发送前要使能发送引脚，发送后要使能接收引脚。
void _485_SendByte(  uint8_t ch )
{
	/* 发送一个字节数据到USART1 */
    HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);	
}
/*****************  发送指定长度的字符串 **********************/
void _485_SendStr_length( uint8_t *str ,uint32_t strlen )
{
	unsigned int k=0;

	_485_TX_EN()	;//	使能发送数据	
    do 
    {
        _485_SendByte( *(str + k) );
        k++;
    } while(k < strlen);
		
//	/*加短暂延时，保证485发送数据完毕*/
//	Delay(0xFFF);
		my_delay_us(25);
		
	_485_RX_EN()	;//	使能接收数据
}


/*****************  发送字符串 **********************/
void _485_SendString(  uint8_t *str)
{
	unsigned int k=0;
	
	_485_TX_EN()	;//	使能发送数据
	
    do 
    {
        _485_SendByte(  *(str + k) );
        k++;
    } while(*(str + k)!='\0');
	
//	/*加短暂延时，保证485发送数据完毕*/
//	Delay(0xFFF);
		
	_485_RX_EN()	;//	使能接收数据
}

//获取接收到的数据和长度
char *get_rebuff(uint16_t *len) 
{
    *len = uart_p;
    return (char *)&uart_buff;
}

//清空缓冲区
void clean_rebuff(void) 
{

    uint16_t i=UART_BUFF_SIZE+1;
    uart_p = 0;	//计数归零
	while(i)
		uart_buff[--i]=0;
}

//static void Delay(__IO uint32_t nCount)	 //简单的延时函数
//{
//	for(; nCount != 0; nCount--);
//}

//void float2char(float value/*需要转换的值*/,
//								char* cSendBuff/*结果存储的数组*/, 
//								int Decimals/*小数位的长度*/)
//{
//	int i = 1, k = 0;
//	int integer = abs(value);//整数部分
//	int decimal = (abs(value) - integer)*pow(10, Decimals);//小数部分
//	int temp = integer;
//	if (value < 0)cSendBuff[k++] = '-';//如果小于0，加个负号
//	while (temp /= 10)
//	{
//		i*=10;
//	}
//	while (integer) {
//		cSendBuff[k++] = integer / i + '0';
//		integer %= i;
//		i /= 10;
//	}
//	if (Decimals == 0) {	//如果没有小数位，直接返回
//		cSendBuff[k++] = '\0';
//		return;
//	}
//	cSendBuff[k++] = '.';    //加小数点
//	temp = decimal;
//	i = 1;
//	while (temp /= 10)
//	{
//		i *= 10;
//	}
//	while (decimal) {
//		cSendBuff[k++] = decimal / i + '0';
//		decimal %= i;
//		i /= 10;
//	}
//	cSendBuff[k++] = '\0';
//}

uint8_t * FloatToUint8(uint8_t * char_array,float data)
{
    for(uint8_t i=0;i<4;i++)
    {
        char_array[i] = ((uint8_t*)(&data))[i];
    }
		return (uint8_t *)char_array;
}

float Uint8ToFloat(uint8_t * char_array)
{
		float data;
//		uint8_t* a = char_array;
    for(uint8_t i=0;i<4;i++)
    {
       ((uint8_t*)(&data))[i] = char_array[i];
    }
		return data;
}


