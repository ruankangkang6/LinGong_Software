/*
 * Pubulic.h
 *
 *  Created on: 2024年11月29日
 *      Author: pc
 */

#ifndef PUBULIC_H_
#define PUBULIC_H_

#include "main.h"
#include "TempCommnn.h"
#include "f_carProtocolJR-SYNC.h"
#include "MotorInclude.h"
#include "f_runCaseDeal.h"
#include "f_disCharge.h"
#include "f_torqueDeal.h"
#include "f_freqDeal.h"
#include "f_asrControl.h"
#include "f_eeprom.h"
#include "f_paramEst.h"
#include "f_fluxWeak.h"
#include "f_deadBandComp.h"
#include "f_torqueComp.h"
#include "f_torModSpdlimit.h"
#include "f_tempCheck.h"
#include "f_fanControl.h"
#include "f_faultProtect.h"

#define RUN_CTRL_PERIOD         2       // 处理周期，_ms

//---------------------CAN接收数据结构体----------------------------------
typedef struct CANRE_STRUCT_DEF
{
    int16_t EnableReq;               //1 使能信号
    int16_t ModeReq;                 //2 控制模式
    int16_t Direction;               //3 运行方向
    float FreqCmd;               //4 目标速度
    float TorqueCmd;             //5 目标转矩
    float MaxTroqLimit;          //6 最大正扭矩限值
    float MinTroqLimit;          //7 最大负扭矩限值
    float FwdSpdLimit;           //8 正转速度上限设定，绝对值
    float RevSpdLimit;           //9 正转速度上限设定，绝对值
}CanReDmdStruct;
extern CanReDmdStruct gDmdCanRe;

//---------------------CAN外发数据结构体----------------------------------
typedef struct CANSED_STRUCT_DEF
{
    int16_t EnableFlag;              //1 使能信号
    int16_t McuMode;                 //2 控制模式
    int16_t Direction;               //3 运行方向
    float FreqSet;               //4 速度给定
    float TorqueSet;             //5 转矩给定
}CanSedDmdStruct;
extern CanSedDmdStruct gDmdCanSed;

//---------------------电机基本信息----------------------------------
typedef struct MOTOR_STRUCT_DEF
{
    uint16_t  Number;                 //电机编号
    uint16_t  Type;                   //电机类型
    uint16_t  Power;                  //电机额定功率
    uint16_t  PeakPower;              //电机峰值功率
    float Rpm;                    //电机额定转速  单位1rpm
    float MaxRpm;                 //电机峰值转速
    float Frequency;              //电机额定频率
    uint16_t  Poles;                  //电机极对数
    uint16_t  RTPoles;                //旋变极对数
    uint16_t  Voltage;                //电机额定电压
    float Current;                //电机额定电流
    float MaxCurrent;             //电机最大电流
    float MaxTorque;              //电机最大扭矩

    float LD;                     //电机D轴电感
    float LQ;                     //电机q轴电感
    float PmRs;                   //电机定子电阻
    float VsCoe;                  //电机反电势常数
}MOTOR_STRUCT;//电机基本信息结构
extern MOTOR_STRUCT         gMotorInfo;     //电机信息


typedef struct INV_STRUCT_DEF
{
	uint16_t  InvPower;               //控制器额定功率
	uint16_t  InvVotage;              //控制器额定电压
    float InvCurrent;             //控制器额定电流
    float InvMaxCurrent;          //控制器峰值电流
    uint16_t  UdcCalCoef;             //母线电压采样校准系数 新增
    uint16_t  PhaseCurrCalCoef;       //相电流采样校准系数  新增
    uint16_t  CtrlVolCalCoef;         //控制电压采样校准系数 新增
}INV_STRUCT;//变频器硬件信息结构
extern INV_STRUCT gInvInfo;

//---------------------加减速计算结构体----------------------------------
typedef struct
{
    void (*calc)(void *);         // 函数指针

    float maxValue;             // 最大值
    float aimValue;             // 目标值
    float curValue;             // 当前值

    uint32_t tickerAll;             // 从0到最大值的ticker
} LINE_CHANGE_STRUCT;

#define LINE_CHANGE_STRTUCT_DEFALUTS       \
{                                          \
    (void (*)(void *))LineChangeCalc       \
}

extern LINE_CHANGE_STRUCT frqLine;
extern void LineChangeCalc(LINE_CHANGE_STRUCT *p);
extern void RunStModeDeal(CanReDmdStruct DmdRe);

extern void Saturatefloat(float* Var, float Max,float Min);
extern void Saturateuint16_t(uint16_t* Var, uint16_t Max, uint16_t Min);
extern void Saturateint16_t(int16_t* Var, int16_t Max, int16_t Min);
extern void PIRegulator( PIReg* pReg );
extern void PIRegulatorEx( PIReg* pReg );

#endif /* PUBULIC_H_ */
