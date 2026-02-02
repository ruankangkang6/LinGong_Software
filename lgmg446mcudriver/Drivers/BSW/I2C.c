#include "main.h"
#include "Global_Variables.h"
//#include "Global_Functions.h"
#define I2C_SLAVE_ADDR_WR    0xA0
#define I2C_SLAVE_ADDR_RD    0xA1

uint16_t I2CReadInt32WaitMode(uint16_t Address, uint32_t* u32Data)
{
	uint16_t	rc = 0;
	uint32_t TimeOut;
	uint8_t ReadBuff[4];
	Address = Address << 2;
	TimeOut = 0;
	while ((HAL_I2C_Mem_Read(&hi2c1,I2C_SLAVE_ADDR_RD,Address,I2C_MEMADD_SIZE_16BIT,ReadBuff,4,0xFFFF) != HAL_OK)&&(TimeOut < 0xFFFFF))
	{
		TimeOut++;
		/* code */
	}
	
	//HAL_I2C_Mem_Read(&hi2c1,I2C_SLAVE_ADDR_RD,Address,I2C_MEMADD_SIZE_16BIT,ReadBuff,4,0xFFFFF);
//	HAL_I2C_Master_Transmit(&hi2c1,I2C_SLAVE_ADDR,data,2,10);
//
//	delay_us(1500);
//	HAL_I2C_Master_Receive(&hi2c1,I2C_SLAVE_ADDR,ReadBuff,4,20);


    *u32Data = (uint32_t)ReadBuff[0]<<24;
    *u32Data += (uint32_t)ReadBuff[1]<<16;
    *u32Data += (uint32_t)ReadBuff[2]<<8;
    *u32Data += ReadBuff[3];


    rc = 1;
	return rc;
}

uint16_t I2CWriteInt32WaitMode(uint16_t Address, uint32_t u32Data)
{
	uint16_t rc = 0;
	uint32_t TimeOut = 0;
	uint8_t data[6];
	Address = Address << 2;
	data[0] = Address>>8;
	data[1] = Address - data[0];
	data[0] = ((u32Data&0xFF000000)>>24);
	data[1] = ((u32Data&0x00FF0000)>>16);
	data[2] = ((u32Data&0x0000FF00)>>8);
	data[3] = (u32Data&0x000000FF);
//	if(READ_BIT(hi2c1.Instance->CR2,I2C_SR2_BUSY)||(READ_BIT(hi2c1.Instance->CR1,I2C_CR1_PE))==0)
//	{
//		SET_BIT(hi2c1.Instance->CR1,I2C_CR1_SWRST);
//		CLEAR_BIT(hi2c1.Instance->CR1,I2C_CR1_SWRST);
//		HAL_I2C_MspInit(&hi2c1);
//		HAL_I2C_Init(&hi2c1);
//	}
//	HAL_I2C_Master_Transmit(&hi2c1,I2C_SLAVE_ADDR,data,6,10);
	while ((HAL_I2C_Mem_Write(&hi2c1,I2C_SLAVE_ADDR_WR,Address,I2C_MEMADD_SIZE_16BIT,data,4,0xFFFF) != HAL_OK)&&(TimeOut < 0xFFFFF))
	{
		TimeOut++;
		/* code */
	}
	//HAL_I2C_Mem_Write(&hi2c1,I2C_SLAVE_ADDR_WR,Address,I2C_MEMADD_SIZE_16BIT,data,4,0xFFFFF);
	//delay_us(1);
	//asm(" RPT #1800 || NOP");
	rc = 1;

	return rc;


}
