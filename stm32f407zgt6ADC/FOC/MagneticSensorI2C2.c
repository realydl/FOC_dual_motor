#include "MagneticSensorI2C2.h"

long  angle_data_prev2;
//long  cpr;
float full_rotation_offset2;
//float angle_prev;
//unsigned long velocity_calc_timestamp;
unsigned long velocity_calc_timestamp2;
float angle_prev2;

//初始化IIC
//PD14->SCL PD15->SDA
void IIC2_Init(void)
{					     
//	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;
	   
	GPIO_InitStructure.Pin = GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;   //推挽输出
//	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;   //开漏输出
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM ;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14| GPIO_PIN_15, GPIO_PIN_SET);//PD0,PD1 输出高
}

//产生IIC起始信号
void IIC2_Start(void)
{
	SDA2_OUT();     //sda线输出
	IIC2_SDA_1;	  	  
	IIC2_SCL_1;
	my_delay_us(4);
 	IIC2_SDA_0;//START:when CLK is high,DATA change form high to low 
	my_delay_us(4);
	IIC2_SCL_0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC2_Stop(void)
{
	SDA2_OUT();//sda线输出
	IIC2_SCL_0;
	IIC2_SDA_0;//STOP:when CLK is high DATA change form low to high
 	my_delay_us(4);
	IIC2_SCL_1; 
	IIC2_SDA_1;//发送I2C总线结束信号
	my_delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC2_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA2_IN();      //SDA设置为输入  
	IIC2_SDA_1;
	my_delay_us(1);	   
	IIC2_SCL_1;
	my_delay_us(1);	 
	while(READ_SDA2)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC2_Stop();
			return 1;
		}
	}
	IIC2_SCL_0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC2_Ack(void)
{
	IIC2_SCL_0;
	SDA2_OUT();
	IIC2_SDA_0;
	my_delay_us(2);
	IIC2_SCL_1;
	my_delay_us(2);
	IIC2_SCL_0;
}
//不产生ACK应答		    
void IIC2_NAck(void)
{
	IIC2_SCL_0;
	SDA2_OUT();
	IIC2_SDA_1;
	my_delay_us(2);
	IIC2_SCL_1;
	my_delay_us(2);
	IIC2_SCL_0;
}					 				     

void IIC2_Send_Byte(uint8_t txd)
{
	uint32_t i;
	SDA2_OUT(); 
	IIC2_SCL_0;
	for(i=0;i<8;i++)
	{
		if((txd&0x80)!=0)
			IIC2_SDA_1;
		else
			IIC2_SDA_0;
		txd<<=1;
		my_delay_us(2);
		IIC2_SCL_1;
		my_delay_us(2);
		IIC2_SCL_0;
		my_delay_us(2);
	}
}


//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC2_Read_Byte(unsigned char ack)
{
	uint8_t receive = 0;
	unsigned char i;
	SDA2_IN();//SDA设置为输入
	for(i=0;i<8;i++)
	{
		IIC2_SCL_0; 
		my_delay_us(2);
//		my_delay_us(3);
		IIC2_SCL_1;
		receive<<=1;
		if(READ_SDA2)receive++;  
		my_delay_us(1); 
//		my_delay_us(2);
	}					 
	if (!ack)
			IIC2_NAck();//发送nACK
	else
			IIC2_Ack(); //发送ACK   
	return receive;
}

// 模拟IIC的多字节读
uint8_t Sim_I2C2_Read8(uint8_t moni_dev_addr, uint8_t moni_reg_addr, uint8_t moni_i2c_len, uint8_t *moni_i2c_data_buf)
{
	IIC2_Start();
	IIC2_Send_Byte(moni_dev_addr << 1 | I2C1_Direction_Transmitter);
	IIC2_Wait_Ack();
	IIC2_Send_Byte(moni_reg_addr);
	IIC2_Wait_Ack();
	
	IIC2_Start();
	IIC2_Send_Byte(moni_dev_addr << 1 | I2C1_Direction_Receiver);
	IIC2_Wait_Ack();
	while (moni_i2c_len)
	{
		if (moni_i2c_len==1)
			*moni_i2c_data_buf = IIC2_Read_Byte(0);
		else
			*moni_i2c_data_buf = IIC2_Read_Byte(1);
		moni_i2c_data_buf++;
		moni_i2c_len--;
	}
	IIC2_Stop();
	return 0x00;
}

// 模拟IIC的多字节写
int8_t Sim_I2C2_Write8(uint8_t moni_dev_addr, uint8_t moni_reg_addr, uint8_t moni_i2c_len, uint8_t *moni_i2c_data_buf)
{
	uint8_t i;
	IIC2_Start();
	IIC2_Send_Byte(moni_dev_addr << 1 | I2C1_Direction_Transmitter);
	IIC2_Wait_Ack();
	IIC2_Send_Byte(moni_reg_addr);
	IIC2_Wait_Ack();
	
	for (i=0; i<moni_i2c_len; i++)
	{
		IIC2_Send_Byte(moni_i2c_data_buf[i]);
		IIC2_Wait_Ack();
	}
	IIC2_Stop();
	return 0;
}

//给定地址读一个字节
uint8_t readOneByte2(uint8_t in_adr)
{
  uint8_t retVal = 0;
	
	Sim_I2C2_Read8(_ams5600_Address,in_adr,1,&retVal);
	my_delay_ms(4);
  return retVal;
}

//给定两个地址 各自读出一个字节
uint16_t readTwoBytes2(uint8_t in_adr_hi, uint8_t in_adr_lo)
{
	uint16_t retVal = 0;
	uint8_t low=0,high=0;
		
	/* Read Low Byte */
	low = readOneByte2(in_adr_lo);
	/* Read High Byte */  
	high = readOneByte2(in_adr_hi);
	//将两个八位 合成一个16位
	retVal = high << 8;
	retVal = (retVal &0x0fff)| low;
		
	return retVal;
}

////判断有无磁铁
//uint8_t detectMagnet(void)
//{
//  uint8_t magStatus;
//  uint8_t retVal = 0;
//  magStatus = readOneByte(_stat);
//  
//  if(magStatus & 0x20)
//    retVal = 1; 
//  
//  return retVal;
//}


float AS5600_ReadRawAngle2(void)
{
	uint16_t rawangle = 0;
//	float realangle = 0;
	uint8_t dh,dl;		  	    																 
	IIC2_Start();
	IIC2_Send_Byte(_ams5600_Address <<1);
	IIC2_Wait_Ack();
	IIC2_Send_Byte(_raw_ang_hi);
	IIC2_Wait_Ack();
	
	IIC2_Start();
	IIC2_Send_Byte((_ams5600_Address <<1)+1);
	IIC2_Wait_Ack();
	dh=IIC2_Read_Byte(1);   //1-ack for next byte
	dl=IIC2_Read_Byte(0);   //0-end trans
	IIC2_Stop();
	
	rawangle = (dh<<8)+dl;
//	realangle = (float)rawangle/4096*360;//角度
//	realangle = (float)rawangle/4096*_2PI;
	return rawangle;
}

float getAngle2(void)//AS5600
{
	cpr = AS5600_CPR;
	long angle_data,d_angle;
	
	angle_data = AS5600_ReadRawAngle2();
	
	// tracking the number of rotations 
	// in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data - angle_data_prev2;
	// if overflow happened track it as full rotation
	if(fabs(d_angle) > (0.8*cpr) ) 
		full_rotation_offset2 += (d_angle > 0) ? -_2PI : _2PI; 
	// save the current angle value for the next steps
	// in order to know if overflow happened
	angle_data_prev2 = angle_data;
	// return the full angle 
	// (number of full rotations)*2PI + current sensor angle 
	return (full_rotation_offset2 + ( (float)angle_data / cpr) * _2PI);
}


// Shaft velocity calculation
float getVelocity2(void)
{
	float Ts_V;
	unsigned long now_us;
	float vel;
	float angle_now2;
	// calculate sample time
//	now_us = SysTick->VAL; //_micros();
//	if(now_us < velocity_calc_timestamp)
//		Ts = (float)(velocity_calc_timestamp - now_us)/6*1e-9f;
//	else
//		Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp)/6*1e-9f;
//	// quick fix for strange cases (micros overflow)
//	if(Ts == 0 || Ts > 0.5f) Ts = 1e-3f;

	now_us = __HAL_TIM_GetCounter(&htim4);
	if(now_us < velocity_calc_timestamp2)
		Ts_V = (float)(velocity_calc_timestamp2 - now_us)*1e-6f;
	else
		Ts_V = (float)(0x0000FFFF - now_us + velocity_calc_timestamp2)*1e-6f;
	if(Ts_V == 0 || Ts_V > 0.5f) Ts_V = 1e-3f;
	
	// current angle
	angle_now2 = getAngle2();
	// velocity calculation
	vel = (angle_now2 - angle_prev2)/Ts_V;

	// save variables for future pass
	angle_prev2 = angle_now2;
	velocity_calc_timestamp2 = now_us;
	return vel;
}


//// Shaft velocity calculation
//float getVelocity(void)
//{
//	float Ts_V;
//	unsigned long now_us;
//	float vel;
//	float angle_now;
//	// calculate sample time
////	now_us = SysTick->VAL; //_micros();
////	if(now_us < velocity_calc_timestamp)
////		Ts = (float)(velocity_calc_timestamp - now_us)/6*1e-9f;
////	else
////		Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp)/6*1e-9f;
////	// quick fix for strange cases (micros overflow)
////	if(Ts == 0 || Ts > 0.5f) Ts = 1e-3f;

//	now_us = __HAL_TIM_GetCounter(&htim4);
//	if(now_us < velocity_calc_timestamp)
//		Ts_V = (float)(velocity_calc_timestamp - now_us)*1e-6f;
//	else
//		Ts_V = (float)(0x0000FFFF - now_us + velocity_calc_timestamp)*1e-6f;
//	if(Ts_V == 0 || Ts_V > 0.5f) Ts_V = 1e-3f;
//	
//	// current angle
//	angle_now = getAngle();
//	// velocity calculation
//	vel = (angle_now - angle_prev)/Ts_V;

//	// save variables for future pass
//	angle_prev = angle_now;
//	velocity_calc_timestamp = now_us;
//	return vel;
//}



