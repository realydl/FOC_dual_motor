#include "MagneticSensorI2C.h"

long  angle_data_prev;
long  cpr;
float full_rotation_offset;
float angle_prev;

unsigned long velocity_calc_timestamp;



//��ʼ��IIC
//PD0->SCL PD1->SDA
void IIC_Init(void)
{					     
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;
	   
	GPIO_InitStructure.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;   //�������
//	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;   //��©���
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM ;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0| GPIO_PIN_1, GPIO_PIN_SET);//PD0,PD1 �����
}


////PD3->SDA PH7->SCL
//void IIC_Init(void)
//{					     
//	__HAL_RCC_GPIOD_CLK_ENABLE();
//	GPIO_InitTypeDef GPIO_InitStructure;
//	   
//	GPIO_InitStructure.Pin = GPIO_PIN_3;
////	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;   //�������
//	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;   //��©���
//	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM ;
//	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.Pin = GPIO_PIN_7;
////	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;   //�������
//	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;   //��©���
//	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_MEDIUM ;
//	HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
//	
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);//PD3 �����
//	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_SET);//PH7 �����
//}



//void SDA_OUT(void) 
//{ 
//	GPIO_InitTypeDef GPIO_InitStruct;
//	GPIO_InitStruct.Pin = GPIO_PIN_1;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
////GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//}
//void SDA_IN(void) 
//{  
//	GPIO_InitTypeDef GPIO_InitStruct;
//	GPIO_InitStruct.Pin = GPIO_PIN_1;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
////	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
//	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//}

//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA_1;	  	  
	IIC_SCL_1;
	my_delay_us(4);
 	IIC_SDA_0;//START:when CLK is high,DATA change form high to low 
	my_delay_us(4);
	IIC_SCL_0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL_0;
	IIC_SDA_0;//STOP:when CLK is high DATA change form low to high
 	my_delay_us(4);
	IIC_SCL_1; 
	IIC_SDA_1;//����I2C���߽����ź�
	my_delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA_1;
	my_delay_us(1);	   
	IIC_SCL_1;
	my_delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL_0;
	SDA_OUT();
	IIC_SDA_0;
	my_delay_us(2);
	IIC_SCL_1;
	my_delay_us(2);
	IIC_SCL_0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL_0;
	SDA_OUT();
	IIC_SDA_1;
	my_delay_us(2);
	IIC_SCL_1;
	my_delay_us(2);
	IIC_SCL_0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
//void IIC_Send_Byte(uint8_t txd)
//{                        
//  uint8_t t;   
//	SDA_OUT(); 	    
//	IIC_SCL_0;//����ʱ�ӿ�ʼ���ݴ���
//	for(t=0;t<8;t++)
//	{              
//		IIC_SDA =(txd&0x80)>>7;
//		txd<<=1; 	  
//		my_delay_us(2);   //��TEA5767��������ʱ���Ǳ����
//		IIC_SCL_1;
//		my_delay_us(2); 
//		IIC_SCL_0;	
//		my_delay_us(2);
//	}	 
//}
void IIC_Send_Byte(uint8_t txd)
{
	uint32_t i;
	SDA_OUT(); 
	IIC_SCL_0;
	for(i=0;i<8;i++)
	{
		if((txd&0x80)!=0)
			IIC_SDA_1;
		else
			IIC_SDA_0;
		txd<<=1;
		my_delay_us(2);
		IIC_SCL_1;
		my_delay_us(2);
		IIC_SCL_0;
		my_delay_us(2);
	}
}
uint8_t receive = 0;
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i;
	SDA_IN();//SDA����Ϊ����
	for(i=0;i<8;i++)
	{
		IIC_SCL_0; 
		my_delay_us(2);
//		my_delay_us(3);
		IIC_SCL_1;
		receive<<=1;
		if(READ_SDA)receive++;   
		my_delay_us(1); 
//		my_delay_us(2);
	}					 
	if (!ack)
			IIC_NAck();//����nACK
	else
			IIC_Ack(); //����ACK   
	return receive;
}
//u8 IIC_Read_Byte(u8 ack)
//{
//	u8 i,rcv=0;
//	
//	SDA_IN();
//	for(i=0;i<8;i++)
//	{
//		IIC_SCL_0; 
//		delay_s(20);
//		IIC_SCL_1;
//		rcv<<=1;
//		if(READ_SDA!=0)rcv++;
//		delay_s(10);
//	}
//	SDA_OUT();
//	if(!ack)IIC_NAck();
//	else
//		IIC_Ack();
//	return rcv;
//}

// ģ��IIC�Ķ��ֽڶ�
uint8_t Sim_I2C_Read8(uint8_t moni_dev_addr, uint8_t moni_reg_addr, uint8_t moni_i2c_len, uint8_t *moni_i2c_data_buf)
{
	IIC_Start();
	IIC_Send_Byte(moni_dev_addr << 1 | I2C1_Direction_Transmitter);
	IIC_Wait_Ack();
	IIC_Send_Byte(moni_reg_addr);
	IIC_Wait_Ack();
	
	IIC_Start();
	IIC_Send_Byte(moni_dev_addr << 1 | I2C1_Direction_Receiver);
	IIC_Wait_Ack();
	while (moni_i2c_len)
	{
		if (moni_i2c_len==1)
			*moni_i2c_data_buf = IIC_Read_Byte(0);
		else
			*moni_i2c_data_buf = IIC_Read_Byte(1);
		moni_i2c_data_buf++;
		moni_i2c_len--;
	}
	IIC_Stop();
	return 0x00;
}

// ģ��IIC�Ķ��ֽ�д
int8_t Sim_I2C1_Write8(uint8_t moni_dev_addr, uint8_t moni_reg_addr, uint8_t moni_i2c_len, uint8_t *moni_i2c_data_buf)
{
	uint8_t i;
	IIC_Start();
	IIC_Send_Byte(moni_dev_addr << 1 | I2C1_Direction_Transmitter);
	IIC_Wait_Ack();
	IIC_Send_Byte(moni_reg_addr);
	IIC_Wait_Ack();
	
	for (i=0; i<moni_i2c_len; i++)
	{
		IIC_Send_Byte(moni_i2c_data_buf[i]);
		IIC_Wait_Ack();
	}
	IIC_Stop();
	return 0;
}

//������ַ��һ���ֽ�
uint8_t readOneByte(uint8_t in_adr)
{
  uint8_t retVal = 0;
	
	Sim_I2C_Read8(_ams5600_Address,in_adr,1,&retVal);
	my_delay_ms(4);
  return retVal;
}

//����������ַ ���Զ���һ���ֽ�
uint16_t readTwoBytes(uint8_t in_adr_hi, uint8_t in_adr_lo)
{
	uint16_t retVal = 0;
	uint8_t low=0,high=0;
		
	/* Read Low Byte */
	low = readOneByte(in_adr_lo);
	/* Read High Byte */  
	high = readOneByte(in_adr_hi);
	//��������λ �ϳ�һ��16λ
	retVal = high << 8;
	retVal = (retVal &0x0fff)| low;
		
	return retVal;
}

//�ж����޴���
uint8_t detectMagnet(void)
{
  uint8_t magStatus;
  uint8_t retVal = 0;
  magStatus = readOneByte(_stat);
  
  if(magStatus & 0x20)
    retVal = 1; 
  
  return retVal;
}


float AS5600_ReadRawAngle(void)
{
	uint16_t rawangle = 0;
//	float realangle = 0;
	uint8_t dh,dl;		  	    																 
	IIC_Start();
	IIC_Send_Byte(_ams5600_Address <<1);
	IIC_Wait_Ack();
	IIC_Send_Byte(_raw_ang_hi);
	IIC_Wait_Ack();
	
	IIC_Start();
	IIC_Send_Byte((_ams5600_Address <<1)+1);
	IIC_Wait_Ack();
	dh=IIC_Read_Byte(1);   //1-ack for next byte
	dl=IIC_Read_Byte(0);   //0-end trans
	IIC_Stop();
	
	rawangle = (dh<<8)+dl;
//	realangle = (float)rawangle/4096*360;//�Ƕ�
//	realangle = (float)rawangle/4096*_2PI;
	return rawangle;
}

//��bug
float getAngle(void)//AS5600
{
	cpr = AS5600_CPR;
	long angle_data,d_angle;
	
	angle_data = AS5600_ReadRawAngle();//ԭ�Ƕ�
	
	// tracking the number of rotations 
	// in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data - angle_data_prev;
	// if overflow happened track it as full rotation
	if(fabs(d_angle) >= 0.8*cpr ) 
		full_rotation_offset += (d_angle > 0) ? -_2PI : _2PI; 

	// save the current angle value for the next steps
	// in order to know if overflow happened
	angle_data_prev = angle_data;
	// return the full angle 
	
	// (number of full rotations)*2PI + current sensor angle 
	return (full_rotation_offset + ( (float)angle_data / cpr) * _2PI);
}

//float getAngle(void)//AS5600
//{
//	cpr = AS5600_CPR;
//	long angle_data,d_angle;
//	
//	angle_data = AS5600_ReadRawAngle();//ԭ�Ƕ�

//	d_angle = angle_data - angle_data_prev;
//	
//	// if overflow happened track it as full rotation
//	if(fabs(d_angle) >= 0 ){
//		if((angle_data_prev + fabs(d_angle)) > _2PI)
//			
//	}else{
//		if((angle_data_prev - fabs(d_angle)) < _2PI)
//	}
//			

//	angle_data_prev = angle_data;
//	// return the full angle 
//	
//	// (number of full rotations)*2PI + current sensor angle 
//	return (full_rotation_offset + ( (float)angle_data / cpr) * _2PI);
//}

// Shaft velocity calculation
float getVelocity(void)
{
	float Ts_V;
	unsigned long now_us;
	float vel;
	float angle_now;
	// calculate sample time
//	now_us = SysTick->VAL; //_micros();
//	if(now_us < velocity_calc_timestamp)
//		Ts = (float)(velocity_calc_timestamp - now_us)/6*1e-9f;
//	else
//		Ts = (float)(0xFFFFFF - now_us + velocity_calc_timestamp)/6*1e-9f;
//	// quick fix for strange cases (micros overflow)
//	if(Ts == 0 || Ts > 0.5f) Ts = 1e-3f;

	now_us = __HAL_TIM_GetCounter(&htim4);
	if(now_us < velocity_calc_timestamp)
		Ts_V = (float)(velocity_calc_timestamp - now_us)*1e-6f;
	else
		Ts_V = (float)(0x0000FFFF - now_us + velocity_calc_timestamp)*1e-6f;
	if(Ts_V == 0 || Ts_V > 0.5f) Ts_V = 1e-3f;
	
	// current angle
	angle_now = getAngle();
	// velocity calculation
	vel = (angle_now - angle_prev)/Ts_V;

	// save variables for future pass
	angle_prev = angle_now;
	velocity_calc_timestamp = now_us;
	return vel;
}
//void Programe_Run(void)
//{
//	dect = detectMagnet();   
//	rawdata = readTwoBytes(_raw_ang_hi, _raw_ang_lo);
//	/* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */ 
//	degress = rawdata*360/4095;
//}


/****************************************************************************/

//static float angle_data_prev; //�ϴ�λ��
//static float full_rotation_offset; //ת������Ȧ��
//uint8_t buffer[2] = {0};

//void bsp_as5600Init(void) {
//  /* init i2c interface */
//  
//  /* init var */
//  full_rotation_offset = 0;
//  angle_data_prev = bsp_as5600GetRawAngle();
//}

//static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
//  int status;
//  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
//  
//  status = HAL_I2C_Master_Transmit(&hi2c3, dev_addr, pData, count, i2c_time_out);
//  return status;
//}

//static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
//  int status;
//  int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
//  
//  status = HAL_I2C_Master_Receive(&hi2c3, (dev_addr | 1), pData, count, i2c_time_out);
//  return status;
//}

//uint16_t bsp_as5600GetRawAngle(void) {
//  uint16_t raw_angle;
//  
//  uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;
//  
//  i2cWrite(AS5600_ADDR, &raw_angle_register, 1000);//���ܵȴ�ʱ�����
//  i2cRead(AS5600_ADDR, buffer, 1000);//���ܵȴ�ʱ�����
//  raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
//  return raw_angle;
//}

//float bsp_as5600GetAngle(void) {
//  float angle_data = bsp_as5600GetRawAngle();
//  
//  float d_angle = angle_data - angle_data_prev;
//  if(abs(d_angle) > (0.8 * AS5600_CPR)) {
//    full_rotation_offset += (d_angle > 0 ? -_2PI : _2PI);
//  }
//  angle_data_prev = angle_data;
//  
//  return (full_rotation_offset + (angle_data / (float)AS5600_CPR)*_2PI);
//}


