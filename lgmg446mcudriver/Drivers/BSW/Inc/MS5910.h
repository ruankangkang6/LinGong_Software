/*
 * PWM.h
 *
 *  Created on: 2025 Jan 22
 *      Author:
 */

#ifndef INCLUDE_MS5910_H_
#define INCLUDE_MS5910_H_

#define RDC_RESET_H 	HAL_GPIO_WritePin(RDC1_RST_GPIO_Port,RDC1_RST_Pin,1)
#define RDC_RESET_L 	HAL_GPIO_WritePin(RDC1_RST_GPIO_Port,RDC1_RST_Pin,0)
#define RDC_SAMPLE_H 	HAL_GPIO_WritePin(RDC1_SAMPLE_GPIO_Port,RDC1_SAMPLE_Pin,1)
#define RDC_SAMPLE_L 	HAL_GPIO_WritePin(RDC1_SAMPLE_GPIO_Port,RDC1_SAMPLE_Pin,0)
#define RDC_CS_H		HAL_GPIO_WritePin(RDC1_CS_GPIO_Port,RDC1_CS_Pin,1)
#define RDC_CS_L		HAL_GPIO_WritePin(RDC1_CS_GPIO_Port,RDC1_CS_Pin,0)
#define RDC_WR_H		HAL_GPIO_WritePin(RDC1_WR_GPIO_Port,RDC1_WR_Pin,1)
#define RDC_WR_L		HAL_GPIO_WritePin(RDC1_WR_GPIO_Port,RDC1_WR_Pin,0)

//Register Map
#define	REG_POSMSB		  (char)0x80
#define	REG_POSLSB		  (char)0x81
#define	REG_VELMSB		  (char)0x82
#define	REG_VELLSB		  (char)0x83
#define	REG_LOSTH		    (char)0x88
#define	REG_DOSOVERTH	  (char)0x89
#define	REG_DOSMISTH	  (char)0x8A
#define	REG_DOSRSTMAXTH	(char)0x8B
#define	REG_DOSRSTMINTH	(char)0x8C
#define	REG_LOTHIGHTH	  (char)0x8D
#define	REG_LOTLOWTH	  (char)0x8E
#define	REG_EXCFREQ	    (char)0x91
#define	REG_CTRL		    (char)0x92
#define	REG_SOFTRST		  (char)0xF0
#define	REG_FAULT		    (char)0xFF

void MS5910_Init(void);
void MS5910_ClrFaultReg(void);
void MS5910_RDReg(uint8_t addr,uint8_t *val);
void MS5910_RDPos(uint16_t *val);
void MS5910_RDFLT(uint8_t *val);
uint8_t MS5910_READ(uint8_t addr);
void MS5910_RESET(void);
void MS5910_REFAULT(void);
void MS5910_WRITE(uint8_t addr,uint8_t data);
void delay_ms(uint16_t ms);
void delay_us(uint32_t us);
#endif /* INCLUDE_PWM_H_ */
