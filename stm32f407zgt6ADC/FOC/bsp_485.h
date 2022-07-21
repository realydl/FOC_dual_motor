#ifndef __485_H
#define	__485_H

#include <stdio.h>
//#include "stm32f4xx_hal.h"
//#include "usart.h"
#include "main.h"

//中断缓存串口数据
#define UART_BUFF_SIZE      15
extern uint16_t  uart_p;
extern uint8_t   uart_buff[UART_BUFF_SIZE];


// 不精确的延时
static void _485_delay(__IO uint32_t nCount)
{
	for(; nCount != 0; nCount--);
}

/*控制收发引脚*/
//进入接收模式,必须要有延时等待485处理完数据
#define _485_RX_EN()			_485_delay(1000);\
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_9,GPIO_PIN_SET); _485_delay(1000);
//进入发送模式,必须要有延时等待485处理完数据
#define _485_TX_EN()			_485_delay(1000);\
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_9,GPIO_PIN_RESET); _485_delay(1000);


/*debug*/

#define _485_DEBUG_ON         1
#define _485_DEBUG_ARRAY_ON   1
#define _485_DEBUG_FUNC_ON    1
   
   
// Log define
#define _485_INFO(fmt,arg...)           printf("<<-_485-INFO->> "fmt"\n",##arg)
#define _485_ERROR(fmt,arg...)          printf("<<-_485-ERROR->> "fmt"\n",##arg)
#define _485_DEBUG(fmt,arg...)          do{\
																					 if(_485_DEBUG_ON)\
																					 printf("<<-_485-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
																				 }while(0)

#define _485_DEBUG_ARRAY(array, num)    do{\
                                         int32_t i;\
                                         uint8_t* a = array;\
                                         if(_485_DEBUG_ARRAY_ON)\
                                         {\
                                            printf("<<-_485-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printf("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printf("\n");\
                                                }\
                                            }\
                                            printf("\n");\
                                        }\
                                       }while(0)

#define _485_DEBUG_FUNC()               do{\
                                         if(_485_DEBUG_FUNC_ON)\
                                         printf("<<-_485-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)

void _485_SendByte( uint8_t ch );
void _485_SendStr_length( uint8_t *str,uint32_t strlen );
void _485_SendString( uint8_t *str);

char *get_rebuff(uint16_t *len);
void clean_rebuff(void);
uint8_t * FloatToUint8(uint8_t * char_array,float data);			
float Uint8ToFloat(uint8_t * char_array);																			 
#endif
																			 
																			 




																			 