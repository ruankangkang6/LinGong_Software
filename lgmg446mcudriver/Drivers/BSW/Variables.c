#include "main.h"

//uint16_t Input_SeatSignal=0,Input_ParkHold=0,Input_BrakeSignal=0,Input_SeatBelt=0,Input_GearD_Enable=0,Input_Acc_enable=0,Input_GearR_Enable=0;
uint16_t u16AdcResult0_iw=0;
uint16_t u16AdcResult1_iv=0;
uint16_t u16AdcResult2_tempMotor=0;
uint16_t u16AdcResult3_tempIGBT=0;
uint16_t u16AdcResult4_KSI=0;
uint16_t u16AdcResult5_Accpedal=0;
uint16_t u16AdcResult6_SteeringAngle=0;
uint16_t u16AdcResult7_MainRelay_Cur=0;
uint16_t u16AdcResult8_Drv3_Cur=0;
uint16_t u16AdcResult9_VDC=0;

int Count100ms=0;
int Count50ms=0;
int Count20ms=0;
int Count10ms=0;
int Count2ms=0;
uint16_t MotorType,MotorPower,MotorVotage,MotorCurrent,MotorFrequency,MotorRpm,MotorMaxCurrent,RotorTransPoles,MotorLd,MotorLq,MotorRs,MotorBemf,MotorZeroPosCompVal,MotorZeroPosition,AsrKpH,AsrKiH,AsrKpL,AsrKiL,AsrChgFrqL,AsrChgFrqH,SpeedFilter,AcrKp,AcrKi,MaxFeedbackCurr,AccTorTime,DecTorTime,MaxFrq,UpperFrq,LowerFrq,AccFrqTime,DecFrqTime,CarrierFrq,InputVoltOn,InputVoltOff,InputVoltOvr,SpeedPLimit,SpeedNLimit,OcpInstLimit,OcpFilterLimit,AMBTempDerateBeg,AMBTempDerateEnd,IGBTTempLimit,MotorTempLimit,IGBTTempDerateBeg,IGBTTempDerateEnd,MotorTempDerateBeg,MotorTempDerateEnd,SafetyVolt,TimeoutSec,FunEnFlag;
uint32_t parameter[100];
