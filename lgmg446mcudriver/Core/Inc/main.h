/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "PWM.h"
#include "Interrupt.h"
#include "MS5910.h"
#include "Global_Variables.h"
#include "math.h"
#include "can_cfg.h"
#include "can2_cfg.h"
#include "MonitorPrm.h"
#include "Monitor.h"
#include "TempCommnn.h"
#include "PrmMgr.h"
#include "f_public.h"
#include "type_define.h"
#include "VehicleCtrl.h"
#include "SDO_COB.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void PWM_enable(void);
extern void PWM_disable(void);
extern void GetVehicleInformation(void);
extern void pwm_freq_set(float freq);
extern void TIM_SetChannalCompare(TIM_TypeDef* TIMx, uint16_t channel, uint16_t Compare);
extern  void AsynPWMAngleCal(uint32_t FcApply);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DI_1_MCU_Pin GPIO_PIN_13
#define DI_1_MCU_GPIO_Port GPIOC
#define DI_3_MCU_Pin GPIO_PIN_14
#define DI_3_MCU_GPIO_Port GPIOC
#define DI_5_MCU_Pin GPIO_PIN_15
#define DI_5_MCU_GPIO_Port GPIOC
#define AC_OCP1_Pin GPIO_PIN_0
#define AC_OCP1_GPIO_Port GPIOC
#define AC_OCP1_EXTI_IRQn EXTI0_IRQn
#define DRIVER1_Current_Pin GPIO_PIN_1
#define DRIVER1_Current_GPIO_Port GPIOC
#define MCU_RST1_Pin GPIO_PIN_2
#define MCU_RST1_GPIO_Port GPIOC
#define ENC1_A_MCU_Pin GPIO_PIN_0
#define ENC1_A_MCU_GPIO_Port GPIOA
#define ENC1_B_MCU_Pin GPIO_PIN_1
#define ENC1_B_MCU_GPIO_Port GPIOA
#define M1_W_Current_Pin GPIO_PIN_2
#define M1_W_Current_GPIO_Port GPIOA
#define M1_V_Current_Pin GPIO_PIN_3
#define M1_V_Current_GPIO_Port GPIOA
#define M1_TEM_MTR_Pin GPIO_PIN_4
#define M1_TEM_MTR_GPIO_Port GPIOA
#define M1_TEM_MODULE_Pin GPIO_PIN_5
#define M1_TEM_MODULE_GPIO_Port GPIOA
#define UDC_AD_Pin GPIO_PIN_6
#define UDC_AD_GPIO_Port GPIOA
#define AI_1_MCU_Pin GPIO_PIN_7
#define AI_1_MCU_GPIO_Port GPIOA
#define CAN_ID_Pin GPIO_PIN_4
#define CAN_ID_GPIO_Port GPIOC
#define RDC1_WR_Pin GPIO_PIN_5
#define RDC1_WR_GPIO_Port GPIOC
#define AI_4_MCU_Pin GPIO_PIN_0
#define AI_4_MCU_GPIO_Port GPIOB
#define KSI_AD_Pin GPIO_PIN_1
#define KSI_AD_GPIO_Port GPIOB
#define Charge_CTR_Pin GPIO_PIN_2
#define Charge_CTR_GPIO_Port GPIOB
#define M1_PWM_EN_Pin GPIO_PIN_10
#define M1_PWM_EN_GPIO_Port GPIOB
#define RDC1_CS_Pin GPIO_PIN_12
#define RDC1_CS_GPIO_Port GPIOB
#define PWM_UL1_A_Pin GPIO_PIN_13
#define PWM_UL1_A_GPIO_Port GPIOB
#define PWM_VL1_A_Pin GPIO_PIN_14
#define PWM_VL1_A_GPIO_Port GPIOB
#define PWM_WL1_A_Pin GPIO_PIN_15
#define PWM_WL1_A_GPIO_Port GPIOB
#define DRIVER5_CTR_Pin GPIO_PIN_8
#define DRIVER5_CTR_GPIO_Port GPIOC
#define DRIVER6_CTR_Pin GPIO_PIN_9
#define DRIVER6_CTR_GPIO_Port GPIOC
#define PWM_UH1_A_Pin GPIO_PIN_8
#define PWM_UH1_A_GPIO_Port GPIOA
#define PWM_VH1_A_Pin GPIO_PIN_9
#define PWM_VH1_A_GPIO_Port GPIOA
#define PWM_WH1_A_Pin GPIO_PIN_10
#define PWM_WH1_A_GPIO_Port GPIOA
#define CANA1_RXD_Pin GPIO_PIN_11
#define CANA1_RXD_GPIO_Port GPIOA
#define CANA1_TXD_Pin GPIO_PIN_12
#define CANA1_TXD_GPIO_Port GPIOA
#define DI_11_MCU_Pin GPIO_PIN_15
#define DI_11_MCU_GPIO_Port GPIOA
#define RCD1_SCLK_Pin GPIO_PIN_10
#define RCD1_SCLK_GPIO_Port GPIOC
#define RDC1_SAMPLE_Pin GPIO_PIN_11
#define RDC1_SAMPLE_GPIO_Port GPIOC
#define RDC1_SDI_Pin GPIO_PIN_12
#define RDC1_SDI_GPIO_Port GPIOC
#define DI_7_MCU_Pin GPIO_PIN_2
#define DI_7_MCU_GPIO_Port GPIOD
#define DI_13_MCU_Pin GPIO_PIN_3
#define DI_13_MCU_GPIO_Port GPIOB
#define RDC1_SDO_Pin GPIO_PIN_4
#define RDC1_SDO_GPIO_Port GPIOB
#define CANB1_RXD_Pin GPIO_PIN_5
#define CANB1_RXD_GPIO_Port GPIOB
#define CANB1_TXD_Pin GPIO_PIN_6
#define CANB1_TXD_GPIO_Port GPIOB
#define RDC1_RST_Pin GPIO_PIN_7
#define RDC1_RST_GPIO_Port GPIOB
#define SCL1_Pin GPIO_PIN_8
#define SCL1_GPIO_Port GPIOB
#define PB9_I2C1_SDA_Pin GPIO_PIN_9
#define PB9_I2C1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
