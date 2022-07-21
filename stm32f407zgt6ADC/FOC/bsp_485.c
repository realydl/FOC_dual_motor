#include "bsp_485.h"

uint16_t  uart_p = 1;
uint8_t   uart_buff[UART_BUFF_SIZE];

/***************** ����һ���ַ�  **********************/
//ʹ�õ��ֽ����ݷ���ǰҪʹ�ܷ������ţ����ͺ�Ҫʹ�ܽ������š�
void _485_SendByte(  uint8_t ch )
{
	/* ����һ���ֽ����ݵ�USART1 */
    HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);	
}
/*****************  ����ָ�����ȵ��ַ��� **********************/
void _485_SendStr_length( uint8_t *str ,uint32_t strlen )
{
	unsigned int k=0;

	_485_TX_EN()	;//	ʹ�ܷ�������	
    do 
    {
        _485_SendByte( *(str + k) );
        k++;
    } while(k < strlen);
		
//	/*�Ӷ�����ʱ����֤485�����������*/
//	Delay(0xFFF);
		my_delay_us(25);
		
	_485_RX_EN()	;//	ʹ�ܽ�������
}


/*****************  �����ַ��� **********************/
void _485_SendString(  uint8_t *str)
{
	unsigned int k=0;
	
	_485_TX_EN()	;//	ʹ�ܷ�������
	
    do 
    {
        _485_SendByte(  *(str + k) );
        k++;
    } while(*(str + k)!='\0');
	
//	/*�Ӷ�����ʱ����֤485�����������*/
//	Delay(0xFFF);
		
	_485_RX_EN()	;//	ʹ�ܽ�������
}

//��ȡ���յ������ݺͳ���
char *get_rebuff(uint16_t *len) 
{
    *len = uart_p;
    return (char *)&uart_buff;
}

//��ջ�����
void clean_rebuff(void) 
{

    uint16_t i=UART_BUFF_SIZE+1;
    uart_p = 0;	//��������
	while(i)
		uart_buff[--i]=0;
}

//static void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
//{
//	for(; nCount != 0; nCount--);
//}

//void float2char(float value/*��Ҫת����ֵ*/,
//								char* cSendBuff/*����洢������*/, 
//								int Decimals/*С��λ�ĳ���*/)
//{
//	int i = 1, k = 0;
//	int integer = abs(value);//��������
//	int decimal = (abs(value) - integer)*pow(10, Decimals);//С������
//	int temp = integer;
//	if (value < 0)cSendBuff[k++] = '-';//���С��0���Ӹ�����
//	while (temp /= 10)
//	{
//		i*=10;
//	}
//	while (integer) {
//		cSendBuff[k++] = integer / i + '0';
//		integer %= i;
//		i /= 10;
//	}
//	if (Decimals == 0) {	//���û��С��λ��ֱ�ӷ���
//		cSendBuff[k++] = '\0';
//		return;
//	}
//	cSendBuff[k++] = '.';    //��С����
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


