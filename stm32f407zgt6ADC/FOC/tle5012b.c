#include "tle5012b.h"
									
uint16_t SPIx_ReadWriteByte(uint16_t byte)
{

	uint16_t read_value = 0;
	
	HAL_SPI_Transmit( &hspi1, (uint8_t *)(&byte), sizeof(byte)/sizeof(uint16_t), 0xff );
	
	HAL_SPI_Transmit( &hspi1, (uint8_t *)(&read_value), sizeof(read_value)/sizeof(uint16_t), 0xff );
	return read_value;
}
 

double ReadAngle(void)
{
	return ( ReadValue(READ_ANGLE_VALUE) * 360.0 / 0x10000 );
}
 

uint16_t ReadSpeed(void)
{
	return ReadValue(READ_SPEED_VALUE);
}
 
uint16_t u16Data;
uint16_t ReadValue(uint16_t u16RegValue)
{
	
 
	SPI_CS_ENABLE;
	
	HAL_SPI_Transmit( &hspi1, (uint8_t *)(&u16RegValue), sizeof(u16RegValue)/sizeof(uint16_t), 0xff );
	HAL_SPI_Receive( &hspi1,(uint8_t *)(&u16Data), sizeof(u16Data)/sizeof(uint16_t), 0xff );
	
	SPI_CS_DISABLE;

	
	return((u16Data & 0x7FFF )<<1);
}


