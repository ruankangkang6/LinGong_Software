#include "main.h"
#include "Global_Variables.h"
//#include "Global_Functions.h"

void MS5910_Init(void)
{
//	uint8_t tempcfg;
	RDC_RESET_H;
	RDC_SAMPLE_H;
	RDC_CS_H;
	RDC_WR_H;
	RDC_RESET_H;
	RDC_RESET_L;
	delay_ms(1);
	RDC_RESET_H;
	delay_ms(20);
	//after Reset delay : 16bit:60ms	14bit:25ms	12bit:20ms	10bit:10ms
	MS5910_ClrFaultReg();
	RDC_CS_L;
	(void)MS5910_READ(REG_LOSTH);
	(void)MS5910_READ(REG_DOSOVERTH);
	(void)MS5910_READ(REG_DOSMISTH);
	(void)MS5910_READ(REG_DOSRSTMAXTH);
	(void)MS5910_READ(REG_DOSRSTMINTH);
	(void)MS5910_READ(REG_LOTHIGHTH);
	(void)MS5910_READ(REG_LOTLOWTH);
	(void)MS5910_READ(REG_EXCFREQ);
	MS5910_WRITE(REG_CTRL,0x7A);
	(void) MS5910_READ(REG_CTRL);
//	tempcfg = MS5910_READ(REG_LOSTH);
//	tempcfg = MS5910_READ(REG_DOSOVERTH);
//	tempcfg = MS5910_READ(REG_DOSMISTH);
//	tempcfg = MS5910_READ(REG_DOSRSTMAXTH);
//	tempcfg = MS5910_READ(REG_DOSRSTMINTH);
//	tempcfg = MS5910_READ(REG_LOTHIGHTH);
//	tempcfg = MS5910_READ(REG_LOTLOWTH);
//	tempcfg = MS5910_READ(REG_EXCFREQ);
//	MS5910_WRITE(REG_CTRL,0x7A);
//	tempcfg = MS5910_READ(REG_CTRL);
	RDC_CS_H;
}
void MS5910_ClrFaultReg(void)
{
	uint8_t val;
	RDC_SAMPLE_H;
	RDC_SAMPLE_L;
	delay_us(1);
	RDC_SAMPLE_H;

	MS5910_RDFLT(&val);

	RDC_SAMPLE_L;
	delay_us(1);
	RDC_SAMPLE_H;
}

void MS5910_RDPos(uint16_t *val)
{
	uint8_t data;
	uint16_t temp16 = 0;
	RDC_CS_L;
	data = MS5910_READ(REG_POSMSB);
	temp16 =(uint16_t)( data<<8);
	data = MS5910_READ(REG_POSLSB);
	temp16 += data;
	*val = ((temp16&0xFFF0));
	RDC_CS_H;

}

void MS5910_RDFLT(uint8_t *val)
{
	RDC_CS_L;
	*val = MS5910_READ(REG_FAULT);
	RDC_CS_H;
}

void MS5910_RESET(void)
{
	RDC_SAMPLE_H;
	RDC_SAMPLE_L;
	RDC_RESET_L;
	delay_ms(3);
	RDC_RESET_H;
	delay_ms(1);
}

void MS5910_REFAULT(void)
{
	RDC_SAMPLE_H;
	delay_us(100);
	RDC_SAMPLE_L;
	delay_us(100);
	MS5910_READ(REG_FAULT);
	RDC_SAMPLE_H;
	delay_us(200);
	RDC_SAMPLE_L;
	delay_us(600);
}

uint8_t MS5910_READ(uint8_t addr)
{
	uint8_t temp = addr;
	uint8_t buff = 0;
//	uint8_t temp2 = 0;
	RDC_WR_L;
//	delay_us(100);
	HAL_SPI_Transmit(&hspi3,&temp,1,1);
	RDC_WR_H;
//	delay_us(200);

	buff=0;
	RDC_WR_L;
//	temp2 = HAL_SPI_TransmitReceive(&hspi3,&temp,&buff,1,1);
	(void)HAL_SPI_TransmitReceive(&hspi3,&temp,&buff,1,1);
	RDC_WR_H;
//	delay_us(200);

	return buff;
}

void MS5910_WRITE(uint8_t addr,uint8_t data)
{
	uint8_t temp = addr;
	uint8_t rData = 0;

	RDC_WR_L;
	delay_us(100);
	HAL_SPI_TransmitReceive(&hspi3,&temp,&rData,1,0xFFFF);
	RDC_WR_H;
	delay_us(600);
	RDC_WR_L;
	temp = data;
	HAL_SPI_TransmitReceive(&hspi3,&temp,&rData,1,0xFFFF);
	RDC_WR_H;
	delay_ms(1);
}

void delay_us(uint32_t us)
{
	 uint32_t ticks;
	 uint32_t told, tnow, tcnt = 0;

	 ticks = us * (180000000 / 1000000);
	 told = SysTick->VAL;
	 while (1)
	 {
		 tnow = SysTick->VAL;
		 if (tnow != told)
		 {
			 if (tnow < told)
			 {
				 tcnt += told - tnow;
			 }
			 else
			 {
				 tcnt += SysTick->LOAD - tnow + told;
			 }
			 told = tnow;
			 if (tcnt >= ticks)
			 {
				 break;
			 }
		 }
	 }
}

void delay_ms(uint16_t ms)
{
	delay_us(ms*1000);
}
