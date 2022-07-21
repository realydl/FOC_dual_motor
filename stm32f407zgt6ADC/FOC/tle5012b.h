#ifndef __TLE5012B_H_
#define __TLE5012B_H_

//#include "cpu_common.h"
//#include "main.h"
//#include "spi.h"

extern SPI_HandleTypeDef hspi1;
 
#define GPIO_CS_Pin_Name  GPIO_PIN_15
#define GPIO_CS_Pin_Type  GPIOA


#define SPI_CS_ENABLE  HAL_GPIO_WritePin( GPIO_CS_Pin_Type, GPIO_CS_Pin_Name, GPIO_PIN_RESET )
#define SPI_CS_DISABLE HAL_GPIO_WritePin( GPIO_CS_Pin_Type, GPIO_CS_Pin_Name, GPIO_PIN_SET )

//#define SPI_CS_ENABLE  HAL_GPIO_WritePin( GPIO_CS_Pin_Type, GPIO_CS_Pin_Name, GPIO_PIN_SET )
//#define SPI_CS_DISABLE HAL_GPIO_WritePin( GPIO_CS_Pin_Type, GPIO_CS_Pin_Name, GPIO_PIN_RESET )

 
/* SPI command for TLE5012 */
#define READ_STATUS					0x8001			//8000
//#define READ_ANGLE_VALUE		0x8021			//8020
#define READ_ANGLE_VALUE		0x8020			//8020
#define READ_SPEED_VALUE		0x8031			//8030
 
#define WRITE_MOD1_VALUE		0x5060							//0_1010_0_000110_0001
#define MOD1_VALUE	0x0001
 
#define WRITE_MOD2_VALUE		0x5080							//0_1010_0_001000_0001
#define MOD2_VALUE	0x0801
 
#define WRITE_MOD3_VALUE		0x5091							//0_1010_0_001001_0001
#define MOD3_VALUE	0x0000
 
#define WRITE_MOD4_VALUE		0x50E0							//0_1010_0_001110_0001
#define MOD4_VALUE	0x0098				//9bit 512
 
#define WRITE_IFAB_VALUE		0x50B1
#define IFAB_VALUE 0x000D
/* Functionality mode */
#define REFERESH_ANGLE		0
 
void SPI_SendData16(uint16_t SendData);
uint16_t SPI_ReadData16(void);
double ReadAngle(void);
uint16_t ReadSpeed(void);
uint16_t ReadValue(uint16_t u16Value);
uint16_t SPIx_ReadWriteByte(uint16_t byte);
uint16_t TlE5012W_Reg(uint16_t Reg_CMD, uint16_t Reg_Data);


#endif


