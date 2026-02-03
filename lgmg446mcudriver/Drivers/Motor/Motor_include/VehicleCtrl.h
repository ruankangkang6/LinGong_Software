/*
 * VehicleCtrl.h
 *
 *  Created on: Mar 5, 2025
 *      Author: Administrator
 */

#ifndef MOTOR_MOTOR_INCLUDE_VEHICLECTRL_H_
#define MOTOR_MOTOR_INCLUDE_VEHICLECTRL_H_

#include "main.h"

#define MAIN_CONTACTOR_ON 		//(HAL_GPIO_WritePin(DRIVER1_CTR_GPIO_Port,DRIVER1_CTR_Pin,GPIO_PIN_SET)) //主接触器
#define MAIN_CONTACTOR_OFF 		//(HAL_GPIO_WritePin(DRIVER1_CTR_GPIO_Port,DRIVER1_CTR_Pin,GPIO_PIN_RESET))

#define PreCharge_On 		    (HAL_GPIO_WritePin(Charge_CTR_GPIO_Port,Charge_CTR_Pin,GPIO_PIN_SET))  //预充开始
#define PreCharge_Off 		    (HAL_GPIO_WritePin(Charge_CTR_GPIO_Port,Charge_CTR_Pin,GPIO_PIN_RESET))

#define Open_DownValve 		    {TIM3->CCR2 = 1800;}   //下降电池阀占空比 13% 1k
#define Close_DownValve 		{TIM3->CCR2 = 0;}

#define SeatSignal			(HAL_GPIO_ReadPin(DI_1_MCU_GPIO_Port,DI_1_MCU_Pin))             //座椅信号
#define ParkHold			(!HAL_GPIO_ReadPin(DI_3_MCU_GPIO_Port,DI_3_MCU_Pin))            //手刹信号
#define BrakeSignalTest1	(HAL_GPIO_ReadPin(DI_5_MCU_GPIO_Port,DI_5_MCU_Pin))             //脚刹信号
#define SeatBelt			1 //(HAL_GPIO_ReadPin(DI_7_MCU_GPIO_Port,DI_7_MCU_Pin))             //安全带信号
#define GearD_Enable		0 //(HAL_GPIO_ReadPin(DI_9_MCU_GPIO_Port,DI_9_MCU_Pin))             //D档信号
#define ACC_enable			(HAL_GPIO_ReadPin(DI_11_MCU_GPIO_Port,DI_11_MCU_Pin))           //油门数字量信号
#define GearR_Enable		(HAL_GPIO_ReadPin(DI_13_MCU_GPIO_Port,DI_13_MCU_Pin))           //R档信号

#define RLY_PWM_PERIOD 100      // 100 step =  100*1us = 1ms (tim6 interrupt gennerate pwm)

typedef struct VEHICLE_CANCTRL
{
	uint16_t TractionSpeed;     //牵引最大转速
	float PSpeedpercentage;     //P模式速度百分比 单位%
	float ESpeedpercentage;     //E模式速度百分比 单位%
	float SSpeedpercentage;     //S模式速度百分比 单位%

	uint16_t PAccTime;             //P模式加速时间 单位0.01s
	uint16_t PDecTime;             //P模式减速时间 单位0.01s
	uint16_t EAccTime;             //E模式加速时间 单位0.01s
	uint16_t EDecTime;             //E模式减速时间 单位0.01s
	uint16_t SAccTime;             //S模式加速时间 单位0.01s
	uint16_t SDecTime;             //S模式减速时间 单位0.01s
	uint16_t HSpeedAccTime;        //高速时踩下刹车减速时间 单位0.01s
	uint16_t LSpeedAccTime;        //低速时踩下刹车减速时间 单位0.01s
	uint16_t CrawDecTime;          //蠕行时踩下刹车减速时间单位0.01s
	uint16_t HalfDecTime;          //半松油门减速时间 单位0.01s
	uint16_t ReverseDecTime;       //反向制动减速时间 单位0.01s
	uint16_t CrawAccAndDec;        //蠕行加减速时间 单位0.01s

	uint16_t UnSeatDelayTime;      //座椅开关延时 单位0.1s
	uint16_t UnSeatbeltDlTime;     //安全带开关延时 单位0.1s
	uint16_t AntiDelayTime;        //最大驻坡时间 单位0.1s

	float ThrottleGain;         //油门模拟量增益 单位%
	float ThrottleOffset;       //油门模拟量偏置 单位%
	float TyreAngleGain;        //转角模拟量增益 单位%
	float TyreAngleOffset;      //转角模拟量偏置 单位%

	uint16_t SeatbeltLogic;     //澳标安全带逻辑
	uint16_t SeatbeltCheck;     //牵引安全带检测功能
	uint16_t PanelLockCheck;    //牵引仪表刷卡功能

	float Backpercentage;       //倒车速度百分比 单位%
	uint16_t SCurveTime1;          //S曲线时间1S 单位0.01s
	uint16_t SCurveTime2;          //S曲线时间1S 单位0.01s
}VehicleStru;
extern VehicleStru gVclCtrl;

extern float32 BMS_Voltage;
extern float32 BMS_Current;
extern Uint16 BMS_SOC;
extern Uint16 BMS_Ca;
extern Uint16 BMS_Time;
extern Uint16 BMS_OVP;
extern Uint16 BMS_overdischarge;
extern Uint16 BMS_CommFault;
extern Uint16 BMS_singleuv;
extern Uint16 BMS_OCP;
extern Uint16 BMS_OTP;
extern Uint16 BMS_OTW;
extern Uint16 BMS_ChrgStatus;
extern Uint16 Drive_Mode;
extern Uint16 Faultcode,Faultlevel;
extern Uint8 testbit;
extern Uint8 LiftMCU_FaultCode;
extern Uint8 LiftMCU_FaultLevel;
extern Uint8 Tbox_msg_index;
extern int8  Steering_Angle;
extern Uint8 Remote_Lock_Status;
extern Uint8 BMS_ChrgMode;
extern float32 BMS_MaxChgCurrAllowed;
extern float32 BMS_MaxDisChgCurrAllowed;
extern Uint8 Lift_Lock;
extern double vehicle_spd;
extern Uint16 LiftMCU_Speed;
extern Uint8 Meter_msg_index;
extern Uint16 I_phase;
extern Uint8 Meter_access;
extern Uint8  Lock_command;

extern Uint8 PreChargeStatus;
extern Uint8 PreCharOnFlag;
extern Uint8 LiftDownSignal;  //举升中位信号(下降开关)
extern Uint8 MainContactorStatus;

extern void VehicleCtrlInit(void);
extern void VehicleDataDeal(void);
extern void ForkLiftRunDeal(void);
extern void ForkLiftHourCnt(void);

void VehicleCtrlTboxMsg_Updata​(uint8_t* Msg_data);
void VehicleCtrlTboxBind​(void);

uint8_t VehicleCtrlGetLockSt​(void);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Main_Contactor_On(void)
void Main_Contactor_Off(void)


#endif /* MOTOR_MOTOR_INCLUDE_VEHICLECTRL_H_ */
