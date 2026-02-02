/*
 * Global_Variables.h
 *
 *  Created on: 2025 Jan 22
 *      Author:
 */

#ifndef INCLUDE_GLOBAL_VARIABLES_H_
#define INCLUDE_GLOBAL_VARIABLES_H_
#include "type_define.h"


extern double PI_Error;
extern double PI_Error_Z1;
extern double KP_Result;
extern double KI_Result;
extern int count1;
extern int count1_limit;
extern int KI_Reset_Flag1;
extern int Flag_Fast2Zero;
extern int Flag_Fast2Zero_Spd;
extern int Flag_De,Flag_In;
extern int AntiCnt, AntiCnt1;
extern int Flag_Anti;
extern int slopecnt,slope_stopcnt;
extern int SlopOut_cnt;
extern int slopecnt1,slope_stopcnt1;
extern int SlopOut_cnt1;
extern int Flag_Behind,Flag_Front;
extern int BrakeSignalTest_old,FlagRest,Flag_Trigger;
extern double Pre_LoopOut,PowerLimit;
extern double ChgUp_Rate;
extern double ChgDown_Rate;
extern double ChgUp_Rate_Spd;
extern double ChgDown_Rate_Spd;
extern double ChgDown_Rate_Spd1;
extern double ChgUp_Rate_Spd1;
extern double Brk_DownRate;
extern double ErrLmt_Calibrate;
extern float MaxRegen_Torq;
extern float gMainCmd_FreqSet_old,gMainCmd_FreqSet_Pre;
extern int Cnt_clear;
extern int KeyEnable_old,Flag_SpdReduce,Flag_Stop,Flag_SpdReduce1;
extern int ACC_Enable,Gear_information;
extern int ACC_Release_flag;

extern Uint32 Powerminute_cnt,Powerhour,Workminute_cnt,workhour,Failureminute_cnt,Failurehour,Pumphour,Pumpminute_cnt;
extern uint8_t HourClearFlag;


extern int16_t MotorTemp;			  
extern int16_t IGBTTemp;
extern float ACC_Steering;
extern int BMS_Faultlevel;
extern int Seat_enable;

extern int16_t BrakeSignalTest;
extern float32 m_SpeedAim,m_SpeedAim_Pre;


//extern uint16_t Input_SeatSignal,Input_ParkHold,Input_BrakeSignal,Input_SeatBelt,Input_GearD_Enable,Input_Acc_enable,Input_GearR_Enable;
extern uint16_t ADCresult[12];
extern unsigned int ms_cnt;
extern double DSP_temperature;
extern uint16_t u16AdcResult0_iw,u16AdcResult1_iv,u16AdcResult2_tempMotor,u16AdcResult3_tempIGBT,u16AdcResult4_KSI,u16AdcResult5_Accpedal,u16AdcResult6_SteeringAngle,u16AdcResult7_MainRelay_Cur,u16AdcResult8_Drv3_Cur,u16AdcResult9_VDC;
extern uint16_t uhADCxConvertedValue[12];
extern float ACC_AD,AD_Ctrl_Vin,Ctrl_Vin;
extern uint32_t systick_cnt;
extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern int Count100ms;
extern int Count50ms;
extern int Count20ms;
extern int Count10ms;
extern int Count2ms;
extern uint16_t MotorType,MotorPower,MotorVotage,MotorCurrent,MotorFrequency,MotorRpm,MotorMaxCurrent,RotorTransPoles,MotorLd,MotorLq,MotorRs,MotorBemf,MotorZeroPosCompVal,MotorZeroPosition,AsrKpH,AsrKiH,AsrKpL,AsrKiL,AsrChgFrqL,AsrChgFrqH,SpeedFilter,AcrKp,AcrKi,MaxFeedbackCurr,AccTorTime,DecTorTime,MaxFrq,UpperFrq,LowerFrq,AccFrqTime,DecFrqTime,CarrierFrq,InputVoltOn,InputVoltOff,InputVoltOvr,SpeedPLimit,SpeedNLimit,OcpInstLimit,OcpFilterLimit,AMBTempDerateBeg,AMBTempDerateEnd,IGBTTempLimit,MotorTempLimit,IGBTTempDerateBeg,IGBTTempDerateEnd,MotorTempDerateBeg,MotorTempDerateEnd,SafetyVolt,TimeoutSec,FunEnFlag;
extern uint32_t parameter[100];
extern Uint16 BDI_LimLevel;
extern uint16_t Poweron_ErrReport_Cnt;
extern int KeyOn_Enable,PowerOn_Logic_Fault_Drive;
extern uint16_t I2CWriteInt32WaitMode(uint16_t Address, uint32_t u32Data);
extern uint16_t I2CReadInt32WaitMode(uint16_t Address, uint32_t* u32Data);



#endif /* INCLUDE_GLOBAL_VARIABLES_H_ */
