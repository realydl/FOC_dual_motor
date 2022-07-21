#ifndef MAGNETICSENSOR2AS5600_LIB_H
#define MAGNETICSENSOR2AS5600_LIB_H



#include "arm_math.h"
#include "mydelay.h"

//#define  AS5600_Address  0x36
//#define  RAW_Angle_Hi    0x0C
//#define  RAW_Angle_Lo    0x0D

//#define I2C1_Direction_Transmitter 		0x00
//#define I2C1_Direction_Receiver   	  0x01
//#define _stat 												0x0b
//#define _raw_ang_hi										0x0c
//#define _raw_ang_lo 									0x0d
//#define _ang_hi												0x0e
//#define _ams5600_Address              0x36

//#define AS5600_CPR      4096

//#define AS5600_RAW_ANGLE_REGISTER  0x0C
//#define AS5600_RAW_ADDR    0x36
//#define AS5600_ADDR        (AS5600_RAW_ADDR << 1)
//#define AS5600_WRITE_ADDR  (AS5600_RAW_ADDR << 1)
//#define AS5600_READ_ADDR   ((AS5600_RAW_ADDR << 1) | 1)

//#define I2C_TIME_OUT_BASE   10
//#define I2C_TIME_OUT_BYTE   1

//#define abs(x) ((x)>0?(x):-(x))

//uint32_t I2C_getRawCount(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

//float bsp_as5600GetAngle(void);
//uint16_t bsp_as5600GetRawAngle(void);
//static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count);
//static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count);
//void bsp_as5600Init(void);



////PD3->SDA PH7->SCL
//#define SDA_IN()		{GPIOD->MODER &= 0xffffff3f;}
//#define SDA_OUT()		{GPIOD->MODER &= 0xffffff3f; GPIOD->MODER |= (uint32_t)(1 << 3*2);}
//#define READ_SDA  HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3)
//#define IIC_SCL_1  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET)
//#define IIC_SCL_0  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET)
//#define IIC_SDA_1  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_7,GPIO_PIN_SET)
//#define IIC_SDA_0  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_7,GPIO_PIN_RESET)

//PD14->SCL PD15->SDA
#define SDA2_IN()		{GPIOD->MODER &= 0x3fffffff;}
#define SDA2_OUT()		{GPIOD->MODER &= 0x3fffffff; GPIOD->MODER |= (uint32_t)(1 << 15*2);}

#define READ_SDA2  	HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15)

#define IIC2_SCL_1  	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET)
#define IIC2_SCL_0  	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET)
#define IIC2_SDA_1  	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET)
#define IIC2_SDA_0  	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET)

//extern float angle_prev;
//extern long cpr;

//IIC所有操作函数
void IIC2_Init(void);                //初始化IIC的IO口				 
void IIC2_Start(void);				//发送IIC开始信号
void IIC2_Stop(void);	  			//发送IIC停止信号
void IIC2_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC2_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC2_Wait_Ack(void); 				//IIC等待ACK信号
void IIC2_Ack(void);					//IIC发送ACK信号
void IIC2_NAck(void);				//IIC不发送ACK信号

void IIC2_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC2_Read_One_Byte(uint8_t daddr,uint8_t addr);

uint8_t Sim_I2C2_Read8(uint8_t moni_dev_addr, uint8_t moni_reg_addr, uint8_t moni_i2c_len, uint8_t *moni_i2c_data_buf);
int8_t Sim_I2C2_Write8(uint8_t moni_dev_addr, uint8_t moni_reg_addr, uint8_t moni_i2c_len, uint8_t *moni_i2c_data_buf);

uint8_t readOneByte2(uint8_t in_adr);
uint16_t readTwoBytes2(uint8_t in_adr_hi, uint8_t in_adr_lo);

float getAngle2(void);
float AS5600_ReadRawAngle2(void);
float getVelocity2(void);

//uint16_t readTwoBytes(uint8_t in_adr_hi, uint8_t in_adr_lo);

//uint8_t detectMagnet(void);

//void Programe_Run(void);

//float AS5600_ReadRawAngle(void);
//float getAngle(void);
//float getVelocity(void);
#endif
