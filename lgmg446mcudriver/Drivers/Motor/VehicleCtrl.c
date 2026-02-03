/*
 * Vehicle_Ctrl.c
 *
 *  Created on: Mar 5, 2025
 *      Author: Administrator
 */

#include "VehicleCtrl.h"
#include "f_public.h"
#include <string.h>

#define CRAWSPEED  70.0f                     //蠕动速度判断阈值 单位hz

extern Uint8 Meter_access;
extern uint16_t FaultcodeFlag[FAULT_CODE_NMB];

extern int    BmsCanLiftCnt;
extern int    BmsCanErrFlag;
extern int    LiftCanLiftCnt;
extern int    CanBusssoffCnt;
extern int    LiftCanErrFlag;
extern int    MeterCanLiftCnt;
extern int    MeterCanErrFlag;
extern int    TboxCanLiftCnt;
extern int    TboxCanErrFlag;

extern uint16_t    TboxA2CanErrFlag;
extern uint16_t    TboxA2CanLiftCnt;
extern uint16_t    TboxA2CanFirst_VFlag;

extern int    TboxA1CanLiftCnt;
extern int    TboxA1CanErrFlag;

extern Uint16 	g_u16canerr_jr;

extern Uint16 	g_PumpRunStatus;

//ACC信号处理
uint16_t ACC_Enable_cnt=0;//
int ACC_Enable_Rsvd=0;//
uint16_t Poweron_ErrReport_Cnt = 0;


Uint32 Powerminute_cnt = 0,Powerhour = 0,Workminute_cnt = 0,workhour = 0,Failureminute_cnt = 0,Failurehour = 0,Pumphour=0,Pumpminute_cnt=0;
Uint8 HourClearFlag = 0;

Uint8 LiftDownSignal = 0;  //举升中位信号(下降开关)
Uint8 PreChargeStatus = 0;
Uint8 PreCharOnFlag = 0;
Uint8 MainContactorStatus = 0;

VehicleStru gVclCtrl =   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0, 0, 0, 0};


// 加速激活高
// 加速激活低
// 加速错误高
// 加速错误低
// 模拟量激活高
// 模拟量激活低
// 模拟量错误高
// 模拟量错误低
// 模拟量死区高
// 模拟量死区低
// 限速开始角�?
// 限速结束角�?
// 转向模拟量使�?
// 座椅离开时间
// 溜坡状态行走电机转�?
 Uint16 Drive_Pot_ActiveHigh = 1450;
 Uint16 Drive_Pot_ActiveLow = 160;
 Uint16 Drive_Pot_ErrorHigh = 4090;
 Uint16 Drive_Pot_ErrorLow = 0;
 Uint16 SteerPotActiveHigh = 3731;
 Uint16 SteerPotActiveLow = 415;
 Uint16 SteerPotErrorHigh = 3900;
 Uint16 SteerPotErrorLow = 200;
 Uint16 SteerPotNeauHigh = 4000;
 Uint16 SteerPotNeauLow = 1000;
 Uint16 StartReduceAngle = 8;
 Uint16 EndReduceAngle = 70;
 Uint16 StreerAnalogEnable = 0;
 Uint16 SeatLeaveTime = 1500;
 Uint16 SlideSlopeSpd = 100;
 Uint16 AntiTime = 1500;


float32 ACC_Percent = 0.0;


int KeyOn_Enable = 0;
int Drive_enable = 0;
int Seat_enable = 0;
int PumpSeat_enable = 0;
int Last_Seat_enable = 0;

Uint16 PMode_MaxSpeed=0,EMode_MaxSpeed=0,SMode_MaxSpeed=0;

int16_t PumpFlag_Seat3s = 1;
int16_t BrakeCnt = 0,BrakeSignalTest= 0,Flag_Seat3s = 1,Flag_SeatCheck = 0,SeatCnt = 0,SeatSignal_old = 0,Flag_SeatCheck_old = 0,MaxSpd = 0;
int PowerOn_Logic_Fault_Drive = 0,FaultLogic_GearD = 0,FaultLogic_GearR = 0,FaultLogic_ACC = 0;
int FaultSeat_GearD = 0, FaultSeat_GearR = 0, FaultSeat_ACC = 0;
int FaultSeat_Pump = 0;

int BMS_Faultlevel=0;
float32 m_SpeedAim = 0,m_SpeedAim_Pre = 0,m_SpeedAim_Pre_Rsvd =0,m_ACCStartSpeed=0,PMode_AccRate=0,EMode_AccRate=0,SMode_AccRate=0,PMode_DecRate=0,EMode_DecRate=0,SMode_DecRate=0,Brake_Rate=0;
float32 HSBrake_Rate=0,LSBrake_Rate=0,CSBrake_Rate=0;
float32 Half_AccRate=0,Rev_AccRate=0,Craw_ADRate=0;
float32 realSpeed=0;
Uint16 BDI_LimLevel=20,RollBackActiveTime=30;
int16_t LastBrakeSignal= 0;

Uint16 Poweron_cnt = 0;
Uint16 Poweroff_cnt = 0;

Uint16 PreChargeon_cnt = 0;
Uint16 PreChargeoff_cnt = 0;
Uint16 LKSIDisable = 0;

float32 BMS_Voltage=0.0f;
float32 BMS_Current= 0.0f;
Uint16 BMS_SOC = 0;
Uint16 BMS_Cap = 0;
Uint16 BMS_Time = 0;
Uint16 BMS_OVP=0;
Uint16 BMS_overdischarge=0;
Uint16 BMS_CommFault=0;
Uint16 BMS_singleuv=0;
Uint16 BMS_OCP = 0;
Uint16 BMS_OTP = 0;
Uint16 BMS_OTW = 0;
Uint16 BMS_ChrgStatus = 0;
Uint16 Drive_Mode = 0;
Uint16 Faultcode=0,Faultlevel=0;
Uint8 testbit=0;
Uint8 LiftMCU_FaultCode=0;
Uint8 LiftMCU_FaultLevel=0;
Uint8 Tbox_msg_index=0;
int8  Steering_Angle=0;
Uint8 Remote_Lock_Status=0;
Uint8 BMS_ChrgMode=0;
float32 BMS_MaxChgCurrAllowed=0;
float32 BMS_MaxDisChgCurrAllowed=0;
Uint8 Lift_Lock=0;
double vehicle_spd;
Uint16 LiftMCU_Speed=0;
Uint8 Meter_msg_index=0;
Uint16 I_phase = 0;
Uint8 Meter_access=0;
Uint8  Lock_command=0;

Uint8 fault_soc_cnt=0;
Uint16 GearD_EnableBackFlag = 0;
Uint16 GearR_EnableBackFlag = 0;

Uint16 AntiACCFlag = 0;
Uint16 LastSeatSignal2=0;

Uint16 GearN_Disable=1;
Uint16 GearNCnt=0;

IL_Tbox_DRV_MCU_0x6B68A1_U Tbox_MCU_CMD;

IL_Tbox_DRV_MCU_0x6B68A2_U Tbox_MCU_GPSID;

enum
 {
    TboxUnBind = 0,
    TboxBind
 };

 enum
 {
    Ignore = 0,
    Action
 };

 enum
 {
    UnLock = 0,
    LockedLevel1,
    LockedLevel2
 };

 #define LockedMissGPS 5

enum
 {
    UnBind = 0,
    StandardBind,
    AdvancedBind,
 };

 enum
 {
    CommPass = 0,
    CommFailed,
 };

  enum
 {
    Unlock_Lock_Failed = 0,
    Unlock_Lock_Success,
 };

 enum
 {
    MatchFailed = 0,
    Matched,
    Matching,
 };

  enum
 {
    DMCU_UnLock = 0,
    DMCU_Lock
 };

 enum
 {
    DMCULock_Not = 0,
    DMCULock_Level1,
    DMCULock_Level2,
 };

union{ 
    uint32_t EE_GPS[2]; // 
    uint8_t U8t[8]; //     
}GPSID_DATA,GPSID_DATA_Get;

union{ 
    uint32_t EE_MCU[2]; // 
    uint8_t U8t[8]; //     
}MCUID_DATA;


#define TBOX_MCUID_EE_ADDR_START   500
#define TBOX_GPSID_EE_ADDR_START   510
#define TBOX_Lock_Level_EE_ADDR   520
#define TBOX_Bind_Level_EE_ADDR   530


uint32_t TboxLock_Level_st = 0;
uint32_t TboxLock_Level_St_Stor = 0;
uint8_t TboxLock_Level_Action_st = 0;

uint8_t TboxBind_Standard_Stor = 0;
uint8_t TboxBind_Advanced_Stor = 0;

uint8_t TboxBind_Standard_st = 0;
uint8_t TboxBind_Advanced_st = 0;

uint8_t Tbox_Lock_Unlock_st = 0;
uint8_t TboxCommErr_st = 0;

uint8_t TboxBind_st_Rflag = 0;
uint8_t TboxBind_GPSID_Rflag = 0;
uint32_t TboxBind_st = 0;
uint32_t TboxBind_st_Stor = 0;

//******Main contactor PWM control parameter******//
static uint8_t RLY_pwm_cnt = 0;
uint8_t RLY_pwm_duty = 0;    // 0~20 → 0~100%

union{ 
    struct
    {
    uint8_t GPS_Adv_st:1; // 
    uint8_t Tbox_Lock_st:2; // 
    uint8_t Tbox_Lock_Resp_st:1; // 
    uint8_t resv1:1; //
    uint8_t Tbox_Bind_Comm_st:1; // 
    uint8_t GPS_Sta_st:1; // 
    uint8_t resv2:1; //
    }st;
    uint8_t MsgData; //
}Tbox_DMCU_Bind_Lock_st;

void VehicleCtrlInit(void)
{

	Uint32 Readdata;
	//Uint32 m_DataErrFlag;
   
	Uint8 i = 0;
#if 0
	m_DataErrFlag = 0;
	
	for(i = 0; i < 10; i++)
	{
		I2CReadInt32WaitMode(5000+i,&Readdata);
		
		if(Readdata == 0xFFFFFFFF)
		{
			if(i == 0)
			{
				Powerhour = 0;
			}
		}
		else
		{
			if(Readdata > Powerhour)
			{
				Powerhour = Readdata;
			}
		}

	}

	for(i = 0; i < 10; i++)
	{
		I2CReadInt32WaitMode(5010+i,&Readdata);
		workhour = Readdata;
		
		if(Readdata == 0xFFFFFFFF)
		{
			if(i == 0)
			{
				workhour = 0;
			}
		}
		else
		{
			if(Readdata > workhour)
			{
				workhour = Readdata;
			}
		}

	}

	
	I2CReadInt32WaitMode(5030,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	PMode_MaxSpeed = Readdata;
	
	I2CReadInt32WaitMode(5031,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	EMode_MaxSpeed = Readdata;
	
	I2CReadInt32WaitMode(5032,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	SMode_MaxSpeed = Readdata;
	
	I2CReadInt32WaitMode(5033,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	BDI_LimLevel = Readdata*0.1;
	
	I2CReadInt32WaitMode(5034,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	RollBackActiveTime = Readdata*0.1;
	
	I2CReadInt32WaitMode(5035,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	PMode_AccRate = Readdata*0.1;
	
	I2CReadInt32WaitMode(5036,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	EMode_AccRate = Readdata*0.1;
	
	I2CReadInt32WaitMode(5037,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	SMode_AccRate = Readdata*0.1;
	
	I2CReadInt32WaitMode(5038,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	PMode_DecRate = Readdata*0.1;
	
	I2CReadInt32WaitMode(5039,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	EMode_DecRate = Readdata*0.1;
	
	I2CReadInt32WaitMode(5040,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	SMode_DecRate = Readdata*0.1;
	
	I2CReadInt32WaitMode(5041,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	Brake_Rate = Readdata*0.1;
	
	I2CReadInt32WaitMode(5042,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	ChgDown_Rate_Spd = Readdata*0.1;
	
	I2CReadInt32WaitMode(5043,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	ChgUp_Rate_Spd = Readdata*0.1;


	I2CReadInt32WaitMode(5044,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	Drive_Pot_ActiveHigh = Readdata;

	I2CReadInt32WaitMode(5045,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	Drive_Pot_ActiveLow = Readdata;

	I2CReadInt32WaitMode(5046,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	Drive_Pot_ErrorHigh = Readdata;

	I2CReadInt32WaitMode(5047,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	Drive_Pot_ErrorLow = Readdata;

	I2CReadInt32WaitMode(5048,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	SteerPotActiveHigh = Readdata;

	I2CReadInt32WaitMode(5049,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	SteerPotActiveLow = Readdata;

	I2CReadInt32WaitMode(5050,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	SteerPotErrorHigh = Readdata;

	I2CReadInt32WaitMode(5051,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	SteerPotErrorLow = Readdata;

	I2CReadInt32WaitMode(5052,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	SteerPotNeauHigh = Readdata;

	I2CReadInt32WaitMode(5053,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	SteerPotNeauLow = Readdata;

	I2CReadInt32WaitMode(5054,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	StartReduceAngle = Readdata;

	I2CReadInt32WaitMode(5055,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	EndReduceAngle = Readdata;

	I2CReadInt32WaitMode(5056,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	StreerAnalogEnable = Readdata;

	I2CReadInt32WaitMode(5057,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	SeatLeaveTime = Readdata;

	I2CReadInt32WaitMode(5058,&Readdata);
	if(Readdata == 0xFFFFFFFF)
	{
		m_DataErrFlag = 1;
	}
	SlideSlopeSpd = Readdata;



#endif

	//for(i = 0; i < 2; i++)
	//{
		//I2CReadInt32WaitMode(TBOX_MCUID_EE_ADDR_START+i,&Readdata);
        MCUID_DATA.EE_MCU[0] = HAL_GetUIDw0(); 
        MCUID_DATA.EE_MCU[1] = HAL_GetUIDw1();
        MCUID_DATA.EE_MCU[1] += HAL_GetUIDw2();

	//}
    for(i = 0; i < 2; i++)
    {
        I2CReadInt32WaitMode(TBOX_GPSID_EE_ADDR_START+i,&Readdata);
        GPSID_DATA.EE_GPS[i] = Readdata;
        GPSID_DATA_Get.EE_GPS[i] = Readdata;
    }
    I2CReadInt32WaitMode(TBOX_Lock_Level_EE_ADDR,&Readdata);
    //Readdata = 0xFF;
    if((Readdata&0xFF) == 0xFF)
    {
        Readdata = 0;
        I2CWriteInt32WaitMode(TBOX_Lock_Level_EE_ADDR,Readdata);
    }
    TboxLock_Level_St_Stor = Readdata;
    TboxLock_Level_st = Readdata;
    TboxLock_Level_Action_st = TboxLock_Level_st&0x03;
    Tbox_Lock_Unlock_st = Unlock_Lock_Success;
    
    I2CReadInt32WaitMode(TBOX_Bind_Level_EE_ADDR,&Readdata);
    if((Readdata&0xFF) == 0xFF)
    {
        Readdata = 0;
        I2CWriteInt32WaitMode(TBOX_Bind_Level_EE_ADDR,Readdata);
    }
    TboxBind_st_Stor = Readdata;
    TboxBind_st = Readdata;
    if(TboxBind_st_Stor == StandardBind)
    {
        TboxBind_Standard_Stor = 1;
        TboxBind_Advanced_Stor = 0;
        TboxBind_Standard_st = 1;
        TboxBind_Advanced_st = 0;
    }
    else if(TboxBind_st_Stor == AdvancedBind)
    {
        TboxBind_Standard_Stor = 0;
        TboxBind_Standard_st = 0;
        TboxBind_Advanced_Stor = 1;
        TboxBind_Advanced_st = 1;
    }
    else
    {
        TboxBind_Standard_Stor = 0;
        TboxBind_Standard_st = 0;
        TboxBind_Advanced_Stor = 0;
        TboxBind_Advanced_st = 0;
    }

}

void VehicleDataDeal(void)
{
	//仪表初始化
	gVclCtrl.TractionSpeed = SDO_SetData_GET(T_M_Spd_max_e);        //牵引最大转速
	gVclCtrl.PSpeedpercentage = (float)SDO_SetData_GET(Pmode_spd_percentage_e)*0.01f;     //P模式速度百分比
	gVclCtrl.ESpeedpercentage = (float)SDO_SetData_GET(Emode_spd_percentage_e)*0.01f;     //E模式速度百分比
	gVclCtrl.SSpeedpercentage = (float)SDO_SetData_GET(Smode_spd_percentage_e)*0.01f;     //S模式速度百分比

	gVclCtrl.PAccTime = SDO_SetData_GET(Tr_Pmode_acc_time_e)*10;             //P模式加速时间 单位ms
	gVclCtrl.PDecTime = SDO_SetData_GET(Tr_Pmode_dec_time_e)*10;             //P模式减速时间 单位ms
	gVclCtrl.EAccTime = SDO_SetData_GET(Tr_Emode_acc_time_e)*10;             //E模式加速时间 单位ms
	gVclCtrl.EDecTime = SDO_SetData_GET(Tr_Emode_dec_time_e)*10;             //E模式减速时间 单位ms
	gVclCtrl.SAccTime = SDO_SetData_GET(Tr_Smode_acc_time_e)*10;             //S模式加速时间 单位ms
	gVclCtrl.SDecTime = SDO_SetData_GET(Tr_Smode_dec_time_e)*10;             //S模式减速时间 单位ms
	gVclCtrl.HSpeedAccTime = SDO_SetData_GET(H_spd_brake_mode_dec_time_e)*10;        //高速时踩下刹车减速时间 单位ms
	gVclCtrl.LSpeedAccTime = SDO_SetData_GET(L_spd_brake_mode_dec_time_e)*10;        //低速时踩下刹车减速时间 单位ms
	gVclCtrl.CrawDecTime = SDO_SetData_GET(Crawl_brake_mode_dec_time_e)*10;          //蠕行时踩下刹车减速时间单位 单位ms
	gVclCtrl.HalfDecTime = SDO_SetData_GET(Half_Throttle_dec_time_e)*10;          //半松油门减速时间 单位ms
	gVclCtrl.ReverseDecTime = SDO_SetData_GET(Reverse_dec_time_e)*10;       //反向制动减速时间 单位ms
	gVclCtrl.CrawAccAndDec = SDO_SetData_GET(Crawl_acc_dec_time_e)*10;        //蠕行加减速时间 单位ms

	gVclCtrl.UnSeatDelayTime = SDO_SetData_GET(Unseat_delay_time_e)*100;      //座椅开关延时  单位ms
	gVclCtrl.UnSeatbeltDlTime = SDO_SetData_GET(Unseatbelt_delay_time_e)*100;     //安全带开关延时 单位ms
	gVclCtrl.AntiDelayTime = SDO_SetData_GET(PSlope_time_MAX_e)*50;        //最大驻坡时间 单位2ms

	gVclCtrl.ThrottleGain =  (float)((int16)SDO_SetData_GET(Throttle_analog_input_gain_e)-400)*0.01f;         //油门模拟量增益 单位%
	if (gVclCtrl.ThrottleGain <=0)
	{
		gVclCtrl.ThrottleGain = 0;
	}
	gVclCtrl.ThrottleOffset = (float)((int16)SDO_SetData_GET(Throttle_analog_input_offset_e)-200)*0.01f;       //油门模拟量偏置 单位%
	if (gVclCtrl.ThrottleOffset <=0)
	{
		gVclCtrl.ThrottleOffset = 0;
	}
	gVclCtrl.TyreAngleGain = (float)((int16)SDO_SetData_GET(Tyre_angle_analog_input_gain_e)-400)*0.01f;        //转角模拟量增益 单位%
	if (gVclCtrl.TyreAngleGain <=0)
	{
		gVclCtrl.TyreAngleGain = 0;
	}
	gVclCtrl.TyreAngleOffset = (float)((int16)SDO_SetData_GET(Tyre_angle_analog_input_offset_e)-200)*0.01f;      //转角模拟量偏置 单位%
	if (gVclCtrl.TyreAngleOffset <=0)
	{
		gVclCtrl.TyreAngleOffset = 0;
	}

	gVclCtrl.SeatbeltLogic = SDO_SetData_GET(Australian_seatbelt_logic_e);     //澳标安全带逻辑
	gVclCtrl.SeatbeltCheck = SDO_SetData_GET(Tracti_seatbelt_check_e);     //牵引安全带检测功能
	gVclCtrl.PanelLockCheck = SDO_SetData_GET(TractioPanelLoc_check_e);    //牵引仪表刷卡功能

	gVclCtrl.Backpercentage = (float)SDO_SetData_GET(Back_off_mod_sp_percentage_e)*0.01f;       //倒车速度百分比 单位%
	gVclCtrl.SCurveTime1 = SDO_SetData_GET(S_curve_mode_time1_e)*10;          //S曲线时间1S 单位ms
	gVclCtrl.SCurveTime2 = SDO_SetData_GET(S_curve_mode_time2_e)*10;          //S曲线时间1S 单位ms
	
	
	//参数转换
	if (ACC_Percent == 0 && fabs(gRotorSpeed.SpeedApply) < 1 && m_SpeedAim == 0) //车辆停止时生效
	{
		PMode_MaxSpeed = (uint16_t)(gVclCtrl.TractionSpeed*gVclCtrl.PSpeedpercentage);//4570;//5212;
		EMode_MaxSpeed = (uint16_t)(gVclCtrl.TractionSpeed*gVclCtrl.ESpeedpercentage);//4570;//5212;
		SMode_MaxSpeed = (uint16_t)(gVclCtrl.TractionSpeed*gVclCtrl.SSpeedpercentage);//2077;

		PMode_AccRate = (float)gVclCtrl.TractionSpeed/gVclCtrl.PAccTime;  //单位rpm/ms
		EMode_AccRate = (float)gVclCtrl.TractionSpeed/gVclCtrl.EAccTime;
		SMode_AccRate = (float)gVclCtrl.TractionSpeed/gVclCtrl.SAccTime;
		PMode_DecRate = (float)gVclCtrl.TractionSpeed/gVclCtrl.PDecTime;
		EMode_DecRate = (float)gVclCtrl.TractionSpeed/gVclCtrl.EDecTime;
		SMode_DecRate = (float)gVclCtrl.TractionSpeed/gVclCtrl.SDecTime;

		Half_AccRate = (float)gVclCtrl.TractionSpeed/gVclCtrl.HalfDecTime;
		Rev_AccRate = (float)gVclCtrl.TractionSpeed/gVclCtrl.ReverseDecTime;
		Craw_ADRate = (float)gVclCtrl.TractionSpeed/gVclCtrl.CrawAccAndDec;

		HSBrake_Rate = (float)gVclCtrl.TractionSpeed/gVclCtrl.HSpeedAccTime;
		LSBrake_Rate = (float)gVclCtrl.TractionSpeed/gVclCtrl.LSpeedAccTime;
		CSBrake_Rate = (float)gVclCtrl.TractionSpeed/gVclCtrl.CrawDecTime;


		SeatLeaveTime = gVclCtrl.UnSeatDelayTime;  //单位ms
		AntiTime = gVclCtrl.AntiDelayTime;            //单位2ms


		Brake_Rate = HSBrake_Rate; //1.3

		BDI_LimLevel = 10;
		RollBackActiveTime = 30;
		ChgDown_Rate_Spd = 17;
		ChgUp_Rate_Spd = 5;
		
		Drive_Pot_ActiveHigh = (uint16_t)(1265 * gVclCtrl.ThrottleGain) ; //1450;
		Drive_Pot_ActiveLow = (uint16_t)(1265 * gVclCtrl.ThrottleOffset) ; //20;  //50
		Drive_Pot_ErrorHigh = 3200;
		Drive_Pot_ErrorLow = 0;
		SteerPotActiveHigh = 3731;
		SteerPotActiveLow = 415;
		SteerPotErrorHigh = 3900;
		SteerPotErrorLow = 200;
		SteerPotNeauHigh = 4000;
		SteerPotNeauLow = 1000;
		StartReduceAngle = 8;
		EndReduceAngle = 70;
		StreerAnalogEnable = 1;

		SlideSlopeSpd = 100;
	}
}


void GetVehicleInformation(void)
{
    float AccRate = 0.0,DecRate = 0.0;

    static Uint16 MainContactorFaultCnt = 0;
    static Uint16 MainContactorStatus_Old = 0;
    static int16 MainContactorOnNum = 0;
    static Uint16 MainConDalayCnt = 0;

    /*---------------预充判断--------------*/
    if (Ctrl_Vin > 50)  //KSI电压
    {
        PreChargeon_cnt++;
        PreChargeoff_cnt = 0;
    }
    else
    {
        PreChargeon_cnt = 0;
        PreChargeoff_cnt++;
    }


    if (PreChargeon_cnt > 100)
    {
    	LKSIDisable = 0;
        PreChargeon_cnt = 150;

        //预充未完成、主解除器未吸合情况下，开始预充
        if ((PreCharOnFlag == 0) && (MainContactorStatus == 0) && (gFault3rd.bit.PreChargeFault == 0))
        {
			PreCharge_On;
			PreCharOnFlag = 1;
        }

        //预充完成、主解除器已吸合情况，断开预充
        if ((PreCharOnFlag == 1) && (MainContactorStatus == 1))
		{
			PreCharge_Off;
			PreCharOnFlag = 0;
		}


        /*---------------主接触器吸合判断--------------*/
        if (gFault3rd.bit.HwOverCurrent == 1 || gUDC.uDCBigFilter < 50)
        {
            Main_Contactor_Off();//MAIN_CONTACTOR_OFF;
            MainContactorStatus = 0;
        }
        else
        {
        	if (PreCharOnFlag == 1)
        	{
                if (gUDC.uDCBigFilter >58 && fabs(Ctrl_Vin - gUDC.uDCBigFilter)<8 )
                {
                    Poweron_cnt++;
                    Poweroff_cnt = 0;
                }
                else
                {
                    Poweron_cnt = 0;
                    Poweroff_cnt++;
                }

                if (Poweron_cnt > 300)
                {
                    KeyOn_Enable = 1;
                    Poweron_cnt = 350;

    				Main_Contactor_On(); //MAIN_CONTACTOR_ON;  //主接触器吸合
    				MainContactorStatus = 1;
                }

                if (Poweroff_cnt > 3000)
                {
                    Poweron_cnt = 3100;

        			PreCharge_Off;
        			PreCharOnFlag = 0;

                    gFault3rd.bit.PreChargeFault = 1;//3s未满足吸合条件，报预充失败
                }
        	}
        }
    }
    else
    {
    	if (PreChargeoff_cnt > 5)
    	{
    		PreCharge_Off;
    		PreCharOnFlag = 0;

        	Poweron_cnt = 0;
        	Poweroff_cnt = 0;

    		LKSIDisable = 1;
    		PreChargeoff_cnt = 10;

    		//关使能后，断开主接触器
    		if(STATUS_RUNNING != g_RunStatus && STATUS_RUNNING != g_PumpRunStatus)
    		{
    			if(gFault3rd.bit.SwOverUdc == 0)
    			{
    				Main_Contactor_Off();//MAIN_CONTACTOR_OFF;
    				MainContactorStatus = 0;
    			}

    		}
    	}
    }


	//主接触器未吸合提前动作报故障
	if (MainContactorStatus == 0)
	{
		if (GearD_Enable == 1)
		{
			FaultLogic_GearD = 1;
		}
		if (GearR_Enable == 1)
		{
			FaultLogic_GearR = 1;
		}
		if (ACC_enable == 1)
		{
			FaultLogic_ACC = 1;
		}
	}
	else
	{
        //提前动作恢复
		if (GearD_Enable == 0)
		{
			FaultLogic_GearD = 0;
		}
		if (GearR_Enable == 0)
		{
			FaultLogic_GearR = 0;
		}
		if (ACC_enable == 0)
		{
			FaultLogic_ACC = 0;
		}
	}
	PowerOn_Logic_Fault_Drive = (FaultLogic_GearD || FaultLogic_GearR || FaultLogic_ACC); //提前动作

    //主接触发粘连
    if ( fabs(Ctrl_Vin - gUDC.uDCBigFilter)<4 && MainContactorStatus == 0 && PreCharOnFlag == 0 && gUDC.uDCBigFilter >20) //主接触器断开、预充断开，KSI电压与母线电压偏差小，判断粘连
    {
        MainContactorFaultCnt++;

        if (MainContactorFaultCnt >= 3000)
        {
            MainContactorFaultCnt = 3000;
            gFault3rd.bit.MainContactorFault = 1;   //粘连3s报故障
        }
    }
    else
    {
    	MainContactorFaultCnt = 0;
    }

    //预充失败
    if ( MainContactorStatus == 0 && MainContactorStatus_Old == 1)
    {
    	MainConDalayCnt = 0;
    	MainContactorOnNum++;

        if (MainContactorOnNum >= 3)
        {
        	MainContactorOnNum = 3;

			PreCharge_Off;
			PreCharOnFlag = 0;

            gFault3rd.bit.PreChargeFault = 1;   //短时间连续3次接触器吸合断开，报预充失败故障，不再预充
        }
    }
    else
    {
        MainConDalayCnt++;
        if (MainConDalayCnt >= 30000) //每30s清除接触器吸合计数
        {
        	MainConDalayCnt = 30000;

        	MainContactorOnNum--;
        	if (MainContactorOnNum<0)
        	{
        		MainContactorOnNum = 0;
        	}
        }
    }
    MainContactorStatus_Old = MainContactorStatus;
	

	if (gVclCtrl.PanelLockCheck == 0)
	{
		Meter_access = 0;  //不检测刷卡
	}


    
    //电流采集开后50us后 判断是否过流 过流则置高RLY
    static uint16_t u16AdcResult7_MainRelay_Cur_old = 0;
    static uint32_t MRLY_TimeBak = 0; // 保存上一次延时计时
    uint32_t MRLY_Time;
    uint32_t MRLY_Deta_Time;
    MRLY_Time = SysTick->VAL;
    if(u16AdcResult7_MainRelay_Cur_old < CUR_TH && u16AdcResult7_MainRelay_Cur >= CUR_TH)
    {
        if (MRLY_TimeBak != 0)
        {
            if (MRLY_TimeBak > MRLY_Time)
            {
                MRLY_Deta_Time = MRLY_TimeBak - MRLY_Time;
            }
            else
            {
                MRLY_Deta_Time = SysTick->LOAD - MRLY_Time + MRLY_TimeBak;
            }

            if (MRLY_Deta_Time >= 9000)   // 50us @180MHz
            {
                MRLY_TimeBak = MRLY_Time;

                //Relay coil overcurrent. Switch on high relay.
                if(u16AdcResult7_MainRelay_Cur >= 3723)
                {
                  Main_Contactor_On();
                }
                
            }
        }
        else
        {
            MRLY_TimeBak = MRLY_Time;   // 第一次初始化
        }

    }
    u16AdcResult7_MainRelay_Cur_old = u16AdcResult7_MainRelay_Cur;



#if(0)
	//下降电池阀控制 (油泵)
	if (1)//(KeyOn_Enable == 1) && (Meter_access == 0))
	{
		if(1)//PumpSeat_enable == 1)
		{
			if(Ctrl_Vin>=80)	//40%
			{
				TIM3->CCR1 = 4000;
			}
			else if(Ctrl_Vin>=70)	//30%
			{
				TIM3->CCR1 = 3000;
			}
			else if(Ctrl_Vin>=60)	//20%
			{
				TIM3->CCR1 = 2000;
			}
			else if(Ctrl_Vin>=50)	//10%
			{
				TIM3->CCR1 = 1000;
			}
			else
			{
				TIM3->CCR1 = 0;
			}
		}
		else
		{
			TIM3->CCR1 = 0;
		}
	}
	else
	{
		TIM3->CCR1 = 0;
	}

	if((LastSeatSignal2 == 0) && (LiftDownSignal == 1) && (PumpSeat_enable == 1))
	{
		Close_DownValve;
	}
	else
	{
		LastSeatSignal2 = PumpSeat_enable;
	}
#endif


	//加速踏板百分比计算
	ACC_Percent = (float)(ACC_AD - (float)Drive_Pot_ActiveLow)/(float)((float)Drive_Pot_ActiveHigh-(float)Drive_Pot_ActiveLow);

	if (ACC_Percent > 1)
	{
		ACC_Percent = 1;
	}
	else if (ACC_Percent < 0)
	{
		ACC_Percent = 0;
	}
	else
	{
		;
	}

    //PES档，最大转速限制
    if (Drive_Mode == 3) //P
    {
        MaxSpd = PMode_MaxSpeed;//5212;
    }
    else if (Drive_Mode == 2)//E
    {
        MaxSpd = EMode_MaxSpeed;//5212;
    }
    else if (Drive_Mode == 1)//S
    {
        MaxSpd = SMode_MaxSpeed;//2077;
    }
    else
    {
        ;
    }

    //PES挡位，目标转速加速步进值处理
    if (Drive_Mode == 3) //P
    {
		AccRate = PMode_AccRate; //0.72

		if((ACC_Percent < (CRAWSPEED*15/PMode_MaxSpeed))&&(fabs(gRotorSpeed.SpeedApplyFilter)<CRAWSPEED))
		{
			AccRate = Craw_ADRate; //0，12
		}


    }
    else if (Drive_Mode == 2)//E
    {
        AccRate = EMode_AccRate;

		if((ACC_Percent < (CRAWSPEED*15/EMode_MaxSpeed))&&(fabs(gRotorSpeed.SpeedApplyFilter)<CRAWSPEED))
		{
			AccRate = Craw_ADRate;
		}
    }
    else if (Drive_Mode == 1)//S
    {
        AccRate = SMode_AccRate;

		if((ACC_Percent < (CRAWSPEED*15/SMode_MaxSpeed))&&(fabs(gRotorSpeed.SpeedApplyFilter)<CRAWSPEED))
		{
			AccRate = Craw_ADRate;
		}
    }

    //PES挡位，目标转速减速步进值处理
    if (Drive_Mode == 3) //P
    {
        if(((GearD_Enable == 1)&&(gRotorSpeed.SpeedApplyFilter>0))||((GearR_Enable == 1)&&(gRotorSpeed.SpeedApplyFilter<0)))
        {
            DecRate = Rev_AccRate;//0.8*PMode_DecRate;//换挡时减速速率
        }
        else
        {
            if(fabs(m_SpeedAim)<100)
            {
                DecRate = 0.139*PMode_DecRate;
            }
            else if(fabs(m_SpeedAim)<500)
            {
                DecRate = 0.416*PMode_DecRate;
            }
            else if(fabs(m_SpeedAim)<2000)
            {
            	DecRate = 0.722*PMode_DecRate;
            }
            else
            {
            	DecRate = PMode_DecRate; //0.72
            }

            //爬坡减速率识别
       	    if(   ((fabs(gDmdCanRe.FreqCmd) - fabs(gRotorSpeed.SpeedApplyFilter)) > 120)  //平地爬坡，松油门快速降转速
       	      || ((m_SpeedAim != 0) && (fabs(gAsr.Asr.PIOut) > 100) ))  //坡起，松油门快速降转速
       	    {
       	    	AntiACCFlag = 1;
       	    }
       	    if (AntiACCFlag == 1)
       	    {
           	    DecRate = 1.2f;
           	    if (fabs(m_SpeedAim) == 0)
           	    {
           	    	AntiACCFlag = 0;
           	    }
       	    }
        }

    }
    else if (Drive_Mode == 2)//E
    {
        if(((GearD_Enable == 1)&&(gRotorSpeed.SpeedApplyFilter>0))||((GearR_Enable == 1)&&(gRotorSpeed.SpeedApplyFilter<0)))
        {
            DecRate = Rev_AccRate;//0.8*PMode_DecRate;//E档目标速度加速步进值
        }
        else
        {
		    if(fabs(m_SpeedAim)<100)
            {
                DecRate = 0.167*EMode_DecRate;
            }
            else if(fabs(m_SpeedAim)<500)
            {
                DecRate = 0.50*EMode_DecRate;
            }
            else if(fabs(m_SpeedAim)<2000)
            {
            	DecRate = 0.867*EMode_DecRate;
            }
            else
            {
            	DecRate = EMode_DecRate; //0.6
            }

            //爬坡减速率识别
       	    if(   ((fabs(gDmdCanRe.FreqCmd) - fabs(gRotorSpeed.SpeedApplyFilter)) > 120)  //平地爬坡，松油门快速降转速
       	      || ((m_SpeedAim != 0) && (fabs(gAsr.Asr.PIOut) > 100) ))  //坡起，松油门快速降转速
       	    {
       	    	AntiACCFlag = 1;
       	    }
       	    if (AntiACCFlag == 1)
       	    {
           	    DecRate = 1.2f;
           	    if (fabs(m_SpeedAim) == 0)
           	    {
           	    	AntiACCFlag = 0;
           	    }
       	    }
        }
    }
    else if (Drive_Mode == 1)//S
    {

        if(fabs(gRotorSpeed.SpeedApplyFilter)<25)
        {
            DecRate = 0.75*SMode_DecRate;
        }
        else
        {
            DecRate = SMode_DecRate; //0.4
        }
    }


#if 0
    if((BMS_OVP == 1)||(BMS_overdischarge == 1)||(BMS_CommFault == 1)||(BMS_singleuv == 1)||(BMS_OCP == 1)||(BMS_OTP == 1))
    {
        BMS_Faultlevel = 3;
    }
    else if((BMS_OTW == 1)||(BMS_SOC < BDI_LimLevel))
    {
        BMS_Faultlevel = 2;
    }
    else
    {
        BMS_Faultlevel = 1;
    }
#endif


    if( g_RunStatus != STATUS_PARAMEST )
    {
        if(KeyOn_Enable == 1)
        {
        	gDmdCanRe.ModeReq = SPEEDCTRL_CMD;   //控制模式固定转速控制
        }

        //座椅状态判断
        if(SeatSignal == 1) //坐上座椅
        {
            Flag_Seat3s = 0;
            Flag_SeatCheck = 0;
            SeatCnt = 0;
        }
        else if((SeatSignal == 0) && (Flag_SeatCheck_old == 1))//刚离开座椅
        {
            Flag_SeatCheck = 1;
            SeatCnt = 0;
        }

        if(Flag_SeatCheck == 1)
        {
            if (SeatSignal == 0)
            {
                SeatCnt++;     //离开座椅时刻开始计时
            }
            else
            {
                SeatCnt = 0;
            }

            if(SeatCnt >= SeatLeaveTime)  //离开计时达到
            {
                SeatCnt = SeatLeaveTime;
                Flag_Seat3s = 1;            //离开时间达到标志置1
            }

            if(SeatCnt >= 500)
            {
            	PumpFlag_Seat3s = 1;            //油泵离开0.5s
            }

        }

        Flag_SeatCheck_old = SeatSignal;

        if((SeatSignal == 1) || (PumpFlag_Seat3s == 0))
        {
        	PumpSeat_enable = 1;
        }
        else
        {
        	PumpSeat_enable = 0;       //油泵座椅离开0.5s，判断座椅丢失停电磁阀
        }

        if((SeatSignal == 1) || (Flag_Seat3s == 0))
        {
            Seat_enable = 1;
        }
        else
        {
            Seat_enable = 0;       //离开座椅一段时，判断座椅信号消失
        }


        if((Seat_enable == 1) && (Faultlevel  <= 2) && (KeyOn_Enable == 1) && \
            (Meter_access == 0) && (Lock_command != 3) && (Lock_command != 1) && \
            (BMS_ChrgStatus != 1) && (LiftMCU_FaultLevel < 3) && (VehicleCtrlGetLockSt​() != 2))
        {
            Drive_enable = 1;                       //坐上座椅，故障等级小于2，预充完成，驱动使能置1
        }
        else
        {
            Drive_enable = 0;                       //离开座椅时间达到，故障等级大于2，预充未完成，驱动使能置0.目标转速清零
        }

		Lift_Lock = VehicleCtrlGetLockSt​();

        //离开座椅，还有挡位和ACC信号报提前动作
        if (Seat_enable == 0)
        {
            if (GearD_Enable == 1)
            {
                FaultSeat_GearD = 1;
            }
            if (GearR_Enable == 1)
            {
                FaultSeat_GearR = 1;
            }
            if (ACC_enable == 1)
            {
                FaultSeat_ACC= 1;
            }
        }

        if (GearD_Enable == 0)
        {
            FaultSeat_GearD = 0;
        }
        if (GearR_Enable == 0)
        {
            FaultSeat_GearR = 0;
        }
        if (ACC_enable == 0)
        {
            FaultSeat_ACC= 0;
        }

        //座椅离开，电磁阀判断
        if (PumpSeat_enable == 1)
        {
            if (GearD_Enable == 0 && GearR_Enable == 0 && ACC_enable == 0)
            {
                FaultSeat_Pump = 0;
            }
        }
        else
        {
            if (GearD_Enable == 1 || GearR_Enable == 1 || ACC_enable == 1)
            {
                FaultSeat_Pump = 1;
            }
        }


        //报文超时判断
        BmsCanLiftCnt++;
        if(BmsCanLiftCnt >= 10000)
        {
        	DisableDrive();//
            BmsCanErrFlag = 1;
            gFault3rd.bit.CanTimeOut = 1;
            BmsCanLiftCnt = 10000;
        }

        LiftCanLiftCnt++;
        if(LiftCanLiftCnt >= 3000)
        {
            LiftCanErrFlag = 1;
            LiftCanLiftCnt = 3000;
        }

        MeterCanLiftCnt++;
        if(MeterCanLiftCnt >= 3000)
        {
            MeterCanErrFlag = 1;
            MeterCanLiftCnt = 3000;
        }

        CanBusssoffCnt++;
        if(CanBusssoffCnt >= 1500)
        {
            CanBusssoffCnt = 1500;
        }

        TboxA2CanLiftCnt++;
        if(TboxA2CanLiftCnt >= 5000)
        {
            TboxA2CanErrFlag = 1;
            TboxA2CanLiftCnt = 5000;
        }

        //故障码判断
        if(gFault3rd.bit.ErrorCurrentCheck == 1)
        {
            FaultcodeFlag[0] = 1;
        }
        else
        {
            FaultcodeFlag[0] = 0;
        }

        if(gFault3rd.bit.SwOverCurrent == 1 || gFault3rd.bit.HwOverCurrent == 1)
        {
            FaultcodeFlag[1] = 1;
        }
        else
        {
            FaultcodeFlag[1] = 0;
        }

        if(gFault3rd.bit.OverSpeed )
        {
            FaultcodeFlag[2] = 1;
        }
        else
        {
            FaultcodeFlag[2] = 0;
        }

        if(gFault3rd.bit.SwOverUdc || gFault3rd.bit.HwOverUdc)
        {
            FaultcodeFlag[3] = 1;
        }
        else
        {
            FaultcodeFlag[3] = 0;
        }


        if(++Poweron_ErrReport_Cnt > 1000)
        {
            if( gFault3rd.bit.SwUnderUdc)
            {
                FaultcodeFlag[4] = 1;
            }
            else
            {
                FaultcodeFlag[4] = 0;
            }

        }


        if(gFault2nd.bit.IGBTSenHighFault == 1)
        {
            FaultcodeFlag[5] = 1;
        }
        else
        {
            FaultcodeFlag[5] = 0;
        }

        if(gFault2nd.bit.IGBTSenLowFault == 1)
        {
            FaultcodeFlag[6] = 1;
        }
        else
        {
            FaultcodeFlag[6] = 0;
        }

        if(gFault2nd.bit.Motor1SenHighFault)
        {
            FaultcodeFlag[7] = 1;
        }
        else
        {
            FaultcodeFlag[7] = 0;
        }
        if(gFault2nd.bit.Motor1SenLowFault)
        {
            FaultcodeFlag[8] = 1;
        }
        else
        {
            FaultcodeFlag[8] = 0;
        }
        if( gFault3rd.bit.OverMotorTemp )
        {
            FaultcodeFlag[9] = 1;
        }
        else
        {
            FaultcodeFlag[9] = 0;
        }
//        if(gFault2nd.bit.AmbSenHighFault )
//        {
//            FaultcodeFlag[10] = 1;
//        }
//        else
//        {
//            FaultcodeFlag[10] = 0;
//        }
//        if(gFault2nd.bit.AmbSenLowFault)
//        {
//            FaultcodeFlag[11] = 1;
//        }
//        else
//        {
//            FaultcodeFlag[11] = 0;
//        }
        if( gFault3rd.bit.OverIGBTTemp )
        {
            FaultcodeFlag[12] = 1;
        }
        else
        {
            FaultcodeFlag[12] = 0;
        }
        if(gFault3rd.bit.CanTimeOut)
        {
            FaultcodeFlag[13] = 1;
        }
        else
        {
            FaultcodeFlag[13] = 0;
        }
        if(gFault3rd.bit.ErrorRTTrans )
        {
            FaultcodeFlag[14] = 1;
        }
        else
        {
            FaultcodeFlag[14] = 0;
        }
        if(FaultLogic_ACC | FaultSeat_ACC)
        {
            FaultcodeFlag[15] = 1;
        }
        else
        {
            FaultcodeFlag[15] = 0;
        }
        if(FaultLogic_GearD | FaultLogic_GearR | FaultSeat_GearD | FaultSeat_GearR)
        {
            FaultcodeFlag[16] = 1;
        }
        else
        {
            FaultcodeFlag[16] = 0;
        }


        if((GearR_Enable == 1) && (GearD_Enable == 1))
        {
            FaultcodeFlag[17] = 1;
        }
        else
        {
            FaultcodeFlag[17] = 0;
        }
        if((ACC_AD > Drive_Pot_ErrorHigh) || (ACC_AD < Drive_Pot_ErrorLow))
        {
            FaultcodeFlag[18] = 1;
        }
        else
        {
            FaultcodeFlag[18] = 0;
        }

        if((ACC_enable == 0) && (ACC_Percent > 0.1))
        {
            FaultcodeFlag[19] = 1;
        }
        else
        {
            FaultcodeFlag[19] = 0;
        }

        if((ACC_enable == 1) && (ParkHold == 1))
        {
            FaultcodeFlag[20] = 1;
        }
        else
        {
            FaultcodeFlag[20] = 0;
        }

        if((BMS_SOC <= BDI_LimLevel) || (Lock_command == 2) || (Lock_command == 3))
        {
            FaultcodeFlag[21] = 1;
        }
        else
        {
            FaultcodeFlag[21] = 0;
        }
        if(++Poweron_ErrReport_Cnt > 1000)
        {
            if(BMS_SOC <= BDI_LimLevel)
            {
            	fault_soc_cnt++;
            	if(fault_soc_cnt>20)
            	{
            		FaultcodeFlag[22] = 1;
            	}
            }
            else
            {
            	fault_soc_cnt = 0;
                FaultcodeFlag[22] = 0;
            }

        }


        if(BMS_Faultlevel > 0)
        {
            FaultcodeFlag[23] = 1;
        }
        else
        {
            FaultcodeFlag[23] = 0;
        }


        if(Lock_command == 1)
        {
            FaultcodeFlag[24] = 1;
        }
        else
        {
            FaultcodeFlag[24] = 0;
        }

        if((Seat_enable == 0) && (ACC_enable == 1))
        {
            FaultcodeFlag[26] = 1;
        }
        else
        {
            FaultcodeFlag[26] = 0;
        }



        if(MeterCanErrFlag != 0)
        {
            FaultcodeFlag[29] = 1;
        }
        else
        {
            FaultcodeFlag[29] = 0;
        }

        if(LiftCanErrFlag != 0)
        {
            FaultcodeFlag[30] = 1;
        }
        else
        {
            FaultcodeFlag[30] = 0;
        }

        if(BmsCanErrFlag != 0)
        {
            FaultcodeFlag[31] = 1;
        }
        else
        {
            FaultcodeFlag[31] = 0;
        }
#if 0
        if(TboxCanErrFlag != 0)
        {
            FaultcodeFlag[32] = 1;
        }
        else
        {
            FaultcodeFlag[32] = 0;
        }
#endif

        //刚坐上座椅，有挡位或加速踏板信息，判断提前动作，驱动使能置0.
        if((Last_Seat_enable == 0) && ((GearD_Enable == 1) || (GearR_Enable == 1) || (ACC_Percent > 0)) && (Seat_enable == 1))
        {
            Drive_enable = 0;
            if((GearD_Enable == 1) || (GearR_Enable == 1))
            {
                FaultcodeFlag[16] = 1;
            }
            if(ACC_Percent > 0)
            {
                FaultcodeFlag[18] = 1;
            }

        }
        else
        {
            Last_Seat_enable = Seat_enable;
        }


        //fault level
        if((BMS_Faultlevel >= 2) || (FaultcodeFlag[0] == 1) || (FaultcodeFlag[1] == 1) || (FaultcodeFlag[2] == 1) || (FaultcodeFlag[3] == 1)\
             || (FaultcodeFlag[4] == 1) || (FaultcodeFlag[5] == 1) || (FaultcodeFlag[6] == 1) || (FaultcodeFlag[7] == 1) || (FaultcodeFlag[8] == 1)\
             || (FaultcodeFlag[9] == 1) || (FaultcodeFlag[10] == 1) || (FaultcodeFlag[11] == 1) || (FaultcodeFlag[12] == 1) || (FaultcodeFlag[13] == 1)\
             || (FaultcodeFlag[14] == 1) || (FaultcodeFlag[15] == 1) || (FaultcodeFlag[16] == 1) || (FaultcodeFlag[17] == 1) || (FaultcodeFlag[18] == 1)\
             || (FaultcodeFlag[19] == 1) || (FaultcodeFlag[20] == 1) || (BMS_Faultlevel >= 2) || (FaultcodeFlag[25] == 1)\
             || (FaultcodeFlag[30] == 1) || (FaultcodeFlag[31] == 1) || (FaultcodeFlag[32] == 1))
        {
            Faultlevel = 3;
        }
        else if((FaultcodeFlag[29] == 1))
        {
            Faultlevel = 2;
        }
        else if((BMS_Faultlevel == 1) || (FaultcodeFlag[22] == 1) || (FaultcodeFlag[21] == 1))
        {
            Faultlevel = 1;
        }
        else
        {
            Faultlevel = 0;
        }



        //根据加速踏板量，得到目标转速
        if(Lock_command == 0)
        {
            m_SpeedAim_Pre = (int16_t)(ACC_Percent * MaxSpd);
            if(BMS_SOC <= BDI_LimLevel)
            {
                m_SpeedAim_Pre = (int16_t)(ACC_Percent * MaxSpd*0.5);
            }
        }
        else if(Lock_command == 1 || (Lock_command == 3))
        {
            m_SpeedAim_Pre = 0;
        }
        else if(Lock_command == 2)
        {
            m_SpeedAim_Pre = (int16_t)(ACC_Percent * MaxSpd*0.5);
        }
        else
        {
            m_SpeedAim_Pre = 0;
        }

        /*if(m_SpeedAim_Pre > MaxSpd )
        {
            m_SpeedAim_Pre = MaxSpd;
        }*/

        realSpeed = gRotorSpeed.SpeedApplyFilter*60/gMotorInfo.Poles;


        //油门目标量赋值
        if(Drive_enable == 0)
        {
            m_SpeedAim_Pre = 0; //离开座椅时间达到，故障等级大于2，预充未完成，提前动作等
        }
        else
        {
            if((m_SpeedAim_Pre - m_SpeedAim_Pre_Rsvd < -100) && (ACC_Enable == 1))
            {
                m_SpeedAim = realSpeed; //加速踏板量变化过大，目标转速给定反馈转速
            }

            if (   ((GearD_Enable == 1) && (GearD_EnableBackFlag == 0) && (fabs(m_SpeedAim) > fabs(realSpeed)))//切换挡位
                || ((GearR_Enable == 1) && (GearR_EnableBackFlag == 0) && (fabs(m_SpeedAim) > fabs(realSpeed)))
			    )
            {
                m_SpeedAim = realSpeed; //加速踏板量变化过大，目标转速给定反馈转速
            }
            GearD_EnableBackFlag = GearD_Enable;
            GearR_EnableBackFlag = GearR_Enable;


            if (GearD_Enable == 1)
            {

                m_SpeedAim_Pre = (0-m_SpeedAim_Pre);
            }
            else if (GearR_Enable == 1)
            {
                if(Drive_Mode == 3 || Drive_Mode == 2)
                {
                    m_SpeedAim_Pre = (m_SpeedAim_Pre)*gVclCtrl.Backpercentage;
                }
                else
                {
                    m_SpeedAim_Pre = m_SpeedAim_Pre;
                }
            }
            else
            {
                m_SpeedAim_Pre = 0;
            }
        }

        //驻坡时间达到，溜坡
        if(slopecnt > AntiTime) //300
        {
            m_SpeedAim_Pre = 200;
        }
        else if(slopecnt1 > AntiTime) //300
        {
            m_SpeedAim_Pre = -200;
        }
        else
        {

        }


        m_SpeedAim_Pre_Rsvd = m_SpeedAim_Pre;


        //脚刹处理
        if (BrakeSignalTest1 ==1)  //有脚刹信号
        {
            BrakeCnt++;            //脚刹信号计时
        }
        else
        {
            BrakeCnt = 0;
        }

        if (BrakeCnt>=180)
        {
            BrakeSignalTest = 1;   //脚刹信号计时达到标志
            if(BrakeCnt > 200)
            {
                BrakeCnt = 200;
            }
        }
        else
        {
            BrakeSignalTest = 0;
        }

        //松开加速踏板，加速踏板量为0时，如果目标转速大于反馈转速，反馈转速赋值给目标转速
        if((ACC_Enable == 0)&&(ACC_Enable_Rsvd == 1) && (GearD_Enable == 1 || GearR_Enable == 1))
        {
            if(fabs(realSpeed) < fabs(m_SpeedAim))
            {
                m_SpeedAim = realSpeed;

            }
        }
        ACC_Enable_Rsvd = ACC_Enable;



        //对目标速度进行平滑处理
        if (((BrakeSignalTest == 1) ||(Flag_SpdReduce1 == 1)) && (slopecnt <= AntiTime) && (slopecnt1 <= AntiTime))//脚刹信号计时达到或速度减速标志1未恢复且开始溜破计时未达到
        {
            m_SpeedAim_Pre  = 0;
            if(m_SpeedAim > 0)
            {
                if(m_SpeedAim > realSpeed)
                {
                    if(realSpeed > 2)
                    {
                        m_SpeedAim = realSpeed;
                    }
                    else
                    {
                        m_SpeedAim = 0;
                    }
                }
                else
                {
                    if((BrakeCnt<50)&&(BrakeCnt>0))
                    {
                        if(m_SpeedAim > (0.7*Brake_Rate))
                        {
                            m_SpeedAim = m_SpeedAim - (0.7*Brake_Rate);
                        }
                        else
                        {
                            m_SpeedAim = 0;
                        }
                    }
                    else
                    {
                        if(m_SpeedAim > Brake_Rate)
                        {
                            m_SpeedAim = m_SpeedAim - Brake_Rate;
                        }
                        else
                        {
                            m_SpeedAim = 0;
                        }
                    }
                }
            }
            else
            {
                if(m_SpeedAim < realSpeed)
                {
                    if(realSpeed < -2)
                    {
                        m_SpeedAim = realSpeed;
                    }
                    else
                    {
                        m_SpeedAim = 0;
                    }
                }
                else
                {
                    if((BrakeCnt<50)&&(BrakeCnt>0))
                    {
                        if(m_SpeedAim < (-0.7*Brake_Rate))
                        {
                            m_SpeedAim = m_SpeedAim +(0.7*Brake_Rate);
                        }
                        else
                        {
                            m_SpeedAim = 0;
                        }
                    }
                    else
                    {
                        if(m_SpeedAim < -Brake_Rate)
                        {
                            m_SpeedAim = m_SpeedAim +Brake_Rate;
                        }
                        else
                        {
                            m_SpeedAim = 0;
                        }
                    }
                }
            }

        }
        else if (m_SpeedAim  > 0)
        {
            if((m_SpeedAim_Pre -m_SpeedAim)> 0)                        //后退加速速率
            {
                if((m_SpeedAim_Pre -m_SpeedAim) > 2500)
                {
                    m_SpeedAim  = m_SpeedAim  + 0.8*AccRate;//0.8 * 0.75 * base
                }
                else if((m_SpeedAim_Pre -m_SpeedAim) > 800)
                {
                    m_SpeedAim  = m_SpeedAim  + 0.6*AccRate;
                }
                else if((m_SpeedAim_Pre -m_SpeedAim) > 50)
                {
                    m_SpeedAim  = m_SpeedAim  + 0.4*AccRate;
                }
                else
                {
                    m_SpeedAim  = m_SpeedAim  + 0.2*AccRate;
                }
            }
            else                                                       //后退减速速率
            {
                if(Flag_Anti == 1)
                {
                    if((m_SpeedAim -m_SpeedAim_Pre) <= 0.2)
                    {
                        m_SpeedAim	= m_SpeedAim_Pre;
                    }
                    else if((m_SpeedAim -m_SpeedAim_Pre) > 1000)
                    {
                        m_SpeedAim	= m_SpeedAim  - 1*DecRate;
                    }
                    else
                    {
                        if((m_SpeedAim_Pre == 0) && (fabs(gRotorSpeed.SpeedApplyFilter) < 13))
                        {
                            m_SpeedAim	= m_SpeedAim  - 0.2;
                        }
                        else
                        {
                            m_SpeedAim	= m_SpeedAim  - 1*DecRate;
                        }
                    }
                }
                else
                {
                    if((m_SpeedAim -m_SpeedAim_Pre) <= 0.2)
                    {
                        m_SpeedAim	= m_SpeedAim_Pre;
                    }
                    else if((m_SpeedAim -m_SpeedAim_Pre) > 1000)
                    {
                        m_SpeedAim	= m_SpeedAim  - 1.1*DecRate;  //1
                    }
                    else
                    {
                        if((m_SpeedAim_Pre == 0) && (fabs(gRotorSpeed.SpeedApplyFilter) < 12))
                        {
                            m_SpeedAim	= m_SpeedAim  - 0.2;
                        }
                        else
                        {
                            m_SpeedAim	= m_SpeedAim  - 1.1*DecRate; //1
                        }
                    }
                }
            }
        }
        else
        {
            if((m_SpeedAim -m_SpeedAim_Pre)> 0)                      //前进加速速率
            {
                //GearR_inc_cnt = 0;

                if((m_SpeedAim -m_SpeedAim_Pre) > 3000)
                {
                    m_SpeedAim  = m_SpeedAim  - 0.95*AccRate;
                }
                else if((m_SpeedAim -m_SpeedAim_Pre) > 800)
                {
                    m_SpeedAim  = m_SpeedAim  - 0.95*AccRate;
                }
                else if((m_SpeedAim -m_SpeedAim_Pre) > 50)
                {
                    m_SpeedAim  = m_SpeedAim  - 0.95*AccRate;
                }
                else
                {
                    m_SpeedAim  = m_SpeedAim  - 0.7*AccRate;
                }
            }
            else                                                      //前进减速速率
            {

                if((m_SpeedAim_Pre -m_SpeedAim) <= 1.8*DecRate)
                {
                    m_SpeedAim = m_SpeedAim_Pre;
                }
                else if((m_SpeedAim_Pre -m_SpeedAim) > 500)
                {
                    m_SpeedAim  = m_SpeedAim  + 1.1*DecRate;   //1.2 //1.8
                }
                else if((m_SpeedAim_Pre -m_SpeedAim) > 3500)
                {
                    m_SpeedAim  = m_SpeedAim  + 1.1*DecRate;   //1.2   1.8
                }
                else if((m_SpeedAim_Pre -m_SpeedAim) > 50)
                {
                    m_SpeedAim  = m_SpeedAim  + 1.1*DecRate;  //1.2  1.8
                }
                else
                {
                    m_SpeedAim  = m_SpeedAim  + 1.1*DecRate; //1
                }


            }
        }
        if(ParkHold == 1)  //有手刹目标转速清零
        {
            m_SpeedAim = 0;
        }

        if ((ACC_Percent == 0.0) && (Flag_Anti == 0) && (BrakeSignalTest == 0) && (LastBrakeSignal == 1) && (fabs(gRotorSpeed.SpeedApplyFilter)< 10) && (slopecnt <= AntiTime) && (slopecnt1 <= AntiTime))
        {
            m_SpeedAim = 0;
        }
        LastBrakeSignal = BrakeSignalTest;


        //空档关使能处理,停车状态下
        if ((GearR_Enable == 0 && GearD_Enable == 0) && (m_SpeedAim == 0) && (fabs(gRotorSpeed.SpeedApplyFilter) < 1) && (fabs(gAsr.Asr.PIOut) < 20))
        {
        	GearNCnt++;
        	if(GearNCnt >= 4000)
        	{
        		GearNCnt = 4100;
            	GearN_Disable = 1;
        	}
        }
        else
        {
        	GearNCnt = 0;
        	GearN_Disable = 0;
        }


        //叉车目标转速输出
        gDmdCanRe.FreqCmd =  ( 0.01667f*m_SpeedAim*gMotorInfo.Poles );

        //叉车使能与正反转判断
        if( 1 == KeyOn_Enable && m_SpeedAim >= 0 && Meter_access ==0 && GearN_Disable == 0 && BMS_Faultlevel < 2 && LKSIDisable == 0)
        {	//正转
        	gDmdCanRe.Direction = FORWARD_DIR;
        	gDmdCanRe.EnableReq = 1;

        }
        else if( 1 == KeyOn_Enable && m_SpeedAim < 0 && Meter_access ==0 &&  GearN_Disable == 0 && BMS_Faultlevel < 2 && LKSIDisable == 0)
        {   //反转
        	gDmdCanRe.Direction = REVERSE_DIR;
        	gDmdCanRe.EnableReq = 1;

        }
        else
        {
        	gDmdCanRe.Direction = FORWARD_DIR;
        	gDmdCanRe.EnableReq = 0;
        }

        //叉车转速上下限
        if( gDmdCanRe.FreqCmd > 0 )
        {
            if(fabs(gDmdCanRe.FreqCmd) > gDmdCanRe.FwdSpdLimit )
            {
            	gDmdCanRe.FreqCmd = gDmdCanRe.FwdSpdLimit;
            }
        }
        else
        {
            if(fabs(gDmdCanRe.FreqCmd) > gDmdCanRe.RevSpdLimit )
            {
            	gDmdCanRe.FreqCmd = -gDmdCanRe.RevSpdLimit;
            }
        }
    }
}

/*************************************************************
    叉车运行处理
*************************************************************/
void ForkLiftRunDeal(void)
{
	static uint16_t Count = 0;

	//加速踏板有模拟量无数字量，输出ACC_Release_flag
	if((ACC_Enable_cnt > 2000)&&(ACC_enable == 0))
	{
		ACC_Release_flag = 1;
	}

	//加速踏板有模拟量，输出ACC_Enable
	if(ACC_Percent > 0)
	{
		ACC_Enable = 1;
	}
	else
	{
		ACC_Enable = 0;
	}

	//加速踏板有模拟量，计时
	if(ACC_Enable == 1)
	{
		ACC_Enable_cnt++;   //有踩加速踏板，开始计时

		if(ACC_Enable_cnt >50000)
		{
			ACC_Enable_cnt = 50010;
		}
	}
	else
	{
		ACC_Enable_cnt = 0;
	}


	//有脚刹，输出Flag_SpdReduce和Flag_SpdReduce1
	if (BrakeSignalTest == 1)
	{
		Flag_SpdReduce = 1;   //脚刹信号时间达到，速度减小标志置1
	}
	else if ((ACC_Enable == 1) || ( fabs(gRotorSpeed.SpeedApplyFilter)>(18+Flag_SpdReduce1*15)))
	{
		Flag_SpdReduce = 0;   //ACC_Enable信号为1或目标频率
	}
	else
	{
		;
	}

	if (BrakeSignalTest == 1)
	{
		Flag_SpdReduce1 = 1;   //脚刹信号时间达到，速度减小标志1置1
	}
	else if( ((fabs(g_freqSet)-fabs(gMainCmd_FreqSet_Pre))>0) || (ACC_Enable == 1) || ( fabs(gRotorSpeed.SpeedApplyFilter)>11))

	{
		Flag_SpdReduce1 = 0;   //目标频率加速或ACC_Enable信号为1或目标频率大于11，速度减小标志1置0
	}
	else
	{
		;
	}

#if 1
	//目标速度等于0且速度减速标志未恢复且反馈速度小于2且脚刹信号时间达到且PI输出小于50且非驻坡状态或上反馈速度小于3且PI输出小于5且目标速度等于零
	if (((m_SpeedAim == 0) && (Flag_SpdReduce == 1) && ((fabs(gRotorSpeed.SpeedApplyFilter) <2) && (BrakeSignalTest == 1) && (fabs(Pre_LoopOut) < 50))&&(Flag_Anti==0)) \
	|| ((fabs(gRotorSpeed.SpeedApplyFilter) < 3) && (fabs(Pre_LoopOut) < 5) && (m_SpeedAim == 0)))
#endif
	{
		Flag_Stop = 1;   //车辆即停标志
	}
	else if ((ACC_Enable == 1) || ( fabs(gRotorSpeed.SpeedApplyFilter)>(4+Flag_SpdReduce1*5)))
	{
		Flag_Stop = 0;   //ACC_Enable信号为1或转速大于3，车辆即停标志清零

	}
	else
	{
		;
	}

	gMainCmd_FreqSet_Pre = g_freqSet;

//放在了转速环路处理
//	if(Flag_Stop == 1)
//	{
//		if(fabs(KI_Result) > 40)
//		{
//			KI_Result = 0.5*KI_Result;//车辆已判断即停，如果KI还很大，进行减半
//		}
//		else
//		{
//			if ((GearD_Enable == 1) && (gRotorSpeed.SpeedApplyFilter > 0))  //预防回弹
//			{
//				gAsr.Asr.Kp = 4.5f;
//			}
//			else if ((GearR_Enable == 1) && (gRotorSpeed.SpeedApplyFilter < 0))
//			{
//				gAsr.Asr.Kp = 4.5f;
//			}
//			else
//			{
//
//				ResetAsrPar();            //减半KI小于40，直接清零，转速环参数复位
//				KP_Result = 0;
//				KI_Result = 0;
//				Pre_LoopOut = 0;
//				gAsr.AsrOutFilter = 0;
//				gAsr.Asr.PIOut = 0;
//			}
//
//		}
//	}

	//驻坡状态退出判断
	 if(  ( ((gPowerTrq.TrqOut>=15) && Flag_Anti == 1 && (Flag_In == 1) && (GearR_Enable == 1)) || ((Flag_In == 1) && (Flag_Anti == 1) && (ACC_Enable == 1)))//踩油门加速退出驻坡状态
	   || ( ((gPowerTrq.TrqOut<=-15) && Flag_Anti == 1 && (Flag_In == 2) && (GearD_Enable == 1)) ||((Flag_In == 2) && (Flag_Anti == 1) && (ACC_Enable == 1)))
	   )
	 {
		 	SlopOut_cnt++;
	 }
	 else
	 {
		SlopOut_cnt = 0;
	 }

	 if (SlopOut_cnt > 100)
	 {
		AntiCnt = 0;
		Flag_Anti = 0;
		SlopOut_cnt = 0;
		Flag_Behind=0;
		Flag_Front = 0;
	 }

     if( IS_T_COMP_EN && fabs( gRotorSpeed.SpeedApply ) <= 500 )   // 100Hz
     {
         CalcCompTorque();
     }
     else
     {
         g_TorqRefComp.TRefCompVal = 0;

     }


     Count++;
     if (Count>1)
     {
#if 0
         if ((((fabs(g_freqSet) < 1)||(ACC_Enable == 0)))&&(fabs(gPowerTrq.TorqueRef)<=3)&&(fabs(gRotorSpeed.SpeedApplyFilter)<1))
         {
             Cnt_clear++;
             if (Cnt_clear >1500)
             {
                 Cnt_clear = 1505;
             }
         }
         else if (Cnt_clear>1500)
         {
             Cnt_clear--;
         }
         else
         {
             Cnt_clear = 0;
         }
#endif

         VCSpeedControl();

         #if 0
         if(Flag_Stop == 1)
         {
                 ResetAsrPar();
                 KP_Result = 0;
                 KI_Result = 0;
                 Pre_LoopOut = 0;
                 gAsr.AsrOutFilter = 0;
                 gAsr.Asr.PIOut = 0;
         }
#endif
         Count = 0;
     }
}


/*************************************************************
    叉车小时计
*************************************************************/
void ForkLiftHourCnt(void)
{
	static uint8_t PowerhourCurIndex = 0;
	static uint8_t WorkhourCurIndex = 0;

	Powerminute_cnt++;
	if(Powerminute_cnt>89999)
	{
		Powerhour++;
		Powerminute_cnt = 0;

		I2CWriteInt32WaitMode(5000+PowerhourCurIndex,Powerhour);
		PowerhourCurIndex = (PowerhourCurIndex+1)%10;
	}


	if(fabs(gRotorSpeed.SpeedApplyFilter)>1)
	{
		Workminute_cnt++;
		if(Workminute_cnt>89999)
		{
			workhour++;
			Workminute_cnt = 0;
			I2CWriteInt32WaitMode(5010+WorkhourCurIndex,workhour);
			WorkhourCurIndex = (WorkhourCurIndex+1)%10;
		}
	}
}

uint8_t VehicleTboxGPSID_MatchSt(void)
{
    uint8_t MatchSt = Matched;
    uint8_t i;
    for(i = 1;i<8;i++)
    {
        if(GPSID_DATA.U8t[i] != Tbox_MCU_GPSID.byte[i])
        {
            MatchSt = MatchFailed;
            break;
        }
    }
    return MatchSt;
}

void VehicleCtrlTboxBind​(void)
{
    uint8_t i;
    uint8_t TboxComm_st = Matching;
    uint8_t TboxMsg_st = 0;
    uint32_t ReadData = 0;

    if(TboxA1CanLiftCnt !=0)
    {
        TboxA1CanLiftCnt = 0;
        Tbox_MCU_CMD.byte[0] = uIlRxTboxDrvMcu0x6B68A1.byte[0];
        for(i = 1;i<8;i++)
        {
            //Tbox_MCU_CMD.byte[i] = uIlRxTboxDrvMcu0x6B68A1.byte[i];
            GPSID_DATA_Get.U8t[i] = uIlRxTboxDrvMcu0x6B68A1.byte[i];
        }
        GPSID_DATA_Get.U8t[0] = 0;
        TboxMsg_st = 1;
    }

    if(TboxA2CanFirst_VFlag == Action)
    {
        if(TboxA2CanErrFlag == 1)
        {
            TboxComm_st = MatchFailed;
        }
        else if(TboxA2CanErrFlag == 0)
        {
            for(i = 0;i<8;i++)
            {
                Tbox_MCU_GPSID.byte[i] = uIlRxTboxDrvMcu0x6B68A2.byte[i];
            }
            TboxComm_st = VehicleTboxGPSID_MatchSt();
            
        }

        
    }
    else
    {
        if(TboxA2CanErrFlag)
        {
            TboxComm_st = MatchFailed;
        }
        else
        {
            TboxComm_st = Matching;
        }
    }

        switch (TboxBind_st)
        {

        case UnBind:
            /* code */
            TboxBind_Standard_st = UnBind;
            TboxBind_Advanced_st = UnBind;
            TboxLock_Level_st = UnLock;
            TboxCommErr_st = CommPass;
            if(Tbox_MCU_CMD.dat.Bind_Advanced == Action)//if bind gps on
            {
                TboxBind_st = AdvancedBind;
                TboxBind_Standard_st = UnBind;
                break;
            }
            if(Tbox_MCU_CMD.dat.Bind_GPS_ON == Action)//if bind gps on
            {
                TboxBind_st = StandardBind;
                TboxBind_Advanced_st = UnBind;
                break;
            }
            if((Tbox_MCU_CMD.dat.Vec_Lock_LV1 == Action)||(Tbox_MCU_CMD.dat.Vec_Lock_LV2 == Action)||(Tbox_MCU_CMD.dat.Vec_UnLock == Action))//if lock level1 or level2 
            {
                Tbox_Lock_Unlock_st = Unlock_Lock_Failed;
            }

            break;

        case StandardBind:
            TboxBind_Standard_st = TboxBind;
            TboxBind_Advanced_st = UnBind;
            /* code */
            if((Tbox_MCU_CMD.dat.UnBind_ALL == Action)||(Tbox_MCU_CMD.dat.UnBind_GPS_OFF == Action))//if unbind all or turn gps off
            {
                TboxBind_st = UnBind;
                TboxBind_Standard_st = UnBind;
                TboxBind_Advanced_st = UnBind;
                TboxLock_Level_st = UnLock;
                //TboxLock_Level2_st = UnLock;/*Clear All st*/
                TboxBind_st = UnBind;
                break;
            }
            if(Tbox_MCU_CMD.dat.Bind_Advanced == Action)//if bind gps on
            {
                TboxBind_st = AdvancedBind;
                TboxBind_Standard_st = UnBind;
                break;
            }

            if(Tbox_MCU_CMD.dat.Vec_Lock_LV1 == Action)//if lock level1
            {
                TboxLock_Level_st = LockedLevel1;
            }       
            
            if(Tbox_MCU_CMD.dat.Vec_Lock_LV2 == Action)//if lock level2
            {
                TboxLock_Level_st = LockedLevel2;
            }
            if(Tbox_MCU_CMD.dat.Vec_UnLock == Action)//if lock level2
            {
                TboxLock_Level_st = UnLock;
            }  
            break;
        
        case AdvancedBind:
            //TboxBind_Standard_st = TboxBind;
            TboxBind_Advanced_st = TboxBind;        
            /* code */
            if((Tbox_MCU_CMD.dat.UnBind_ALL == Action)||(Tbox_MCU_CMD.dat.UnBind_GPS_OFF == Action))//if unbind all or turn gps off
            {
                TboxBind_st = UnBind;
                TboxBind_Standard_st = UnBind;
                TboxBind_Advanced_st = UnBind;
                TboxLock_Level_st = UnLock;
                TboxLock_Level_Action_st = UnLock;
                //TboxLock_Level2_st = UnLock;/*Clear All st*/
                TboxBind_st = UnBind;
                break;
            }

            if(TboxComm_st == Matched)
            {
                if(TboxLock_Level_st == LockedMissGPS)
                {
                    TboxLock_Level_st = UnLock;
                    TboxLock_Level_Action_st = UnLock;
                }
                TboxCommErr_st = CommPass;
                if((Tbox_MCU_CMD.dat.Bind_GPS_ON == Action)&&(Tbox_MCU_CMD.dat.Bind_Advanced == Ignore))//if bind gps on
                {
                    TboxBind_st = StandardBind;
                    TboxBind_Advanced_st = UnBind;
                    break;
                }

                if(Tbox_MCU_CMD.dat.Vec_Lock_LV1 == Action)//if lock level1
                {
                    TboxLock_Level_st = LockedLevel1;
                }       
                
                if(Tbox_MCU_CMD.dat.Vec_Lock_LV2 == Action)//if lock level2
                {
                    TboxLock_Level_st = LockedLevel2;
                }
                if(Tbox_MCU_CMD.dat.Vec_UnLock == Action)//if lock level2
                {
                    TboxLock_Level_st = UnLock;
                    TboxLock_Level_Action_st = UnLock;
                }               

            }
            else if(TboxComm_st == MatchFailed)
            {
                if(TboxLock_Level_st == UnLock)
                {
                    TboxLock_Level_st = LockedMissGPS;
                }
                
                TboxCommErr_st = CommFailed;
                if((Tbox_MCU_CMD.dat.Vec_Lock_LV1 == Action)||(Tbox_MCU_CMD.dat.Vec_Lock_LV2 == Action)||(Tbox_MCU_CMD.dat.Vec_UnLock == Action))//if lock level1 or level2 
                {
                    Tbox_Lock_Unlock_st = Unlock_Lock_Failed;
                }

            }
            break;
        
        default:
                TboxBind_st = UnBind;
                TboxBind_Standard_st = UnBind;
                TboxBind_Advanced_st = UnBind;
                TboxLock_Level_st = UnLock;
                TboxLock_Level_Action_st = UnLock;
                //TboxLock_Level2_st = UnLock;/*Clear All st*/
                TboxBind_st = UnBind;
            break;
        }
        if(TboxBind_st != TboxBind_st_Stor)
        {
            if(TboxBind_st_Rflag == 0)
            {
                I2CWriteInt32WaitMode(TBOX_Bind_Level_EE_ADDR,TboxBind_st);
                I2CReadInt32WaitMode(TBOX_Bind_Level_EE_ADDR,(uint32_t*)&ReadData);
                if(ReadData == TboxBind_st)
                {
                    TboxBind_st_Rflag = 1;
                    if(TboxBind_st == UnBind)
                    {
                        TboxBind_Advanced_Stor = TboxUnBind;
                        TboxBind_Standard_Stor = TboxUnBind;
                        TboxBind_st_Stor = TboxBind_st;
                        TboxBind_st_Rflag = 0;
                    }
                    else if(TboxBind_st == StandardBind)
                    {
                        TboxBind_Advanced_Stor = TboxUnBind;
                        TboxBind_Standard_Stor = TboxBind;
                        TboxBind_st_Stor = TboxBind_st;
                        TboxBind_st_Rflag = 0;
                    }
                    else if(TboxBind_st == AdvancedBind)
                    {
                        TboxBind_Advanced_Stor = TboxBind;
                        
                        TboxBind_Standard_Stor = TboxUnBind;
                        TboxBind_GPSID_Rflag = 1;
                    }                

                }

            }
            if(TboxBind_GPSID_Rflag == 1)
            {

                for(i = 0;i<2;i++)
                {
                    Tbox_MCU_GPSID.byte[i] = GPSID_DATA_Get.U8t[i];
                }
                if((GPSID_DATA_Get.EE_GPS[0] != GPSID_DATA.EE_GPS[0])||(GPSID_DATA_Get.EE_GPS[1] != GPSID_DATA.EE_GPS[1]))
                {
                    for(i = 0;i<2;i++)
                    {
                        if(GPSID_DATA_Get.EE_GPS[i] != GPSID_DATA.EE_GPS[i])
                        {
                            I2CWriteInt32WaitMode(TBOX_GPSID_EE_ADDR_START+i,GPSID_DATA_Get.EE_GPS[i]);
                            I2CReadInt32WaitMode(TBOX_GPSID_EE_ADDR_START+i,(uint32_t*)&ReadData);
                            if(ReadData == GPSID_DATA_Get.EE_GPS[i])
                            {
                                GPSID_DATA.EE_GPS[i] = GPSID_DATA_Get.EE_GPS[i];
                            }
                        }
                    }
                }
                if((GPSID_DATA_Get.EE_GPS[0] == GPSID_DATA.EE_GPS[0])&&(GPSID_DATA_Get.EE_GPS[1] == GPSID_DATA.EE_GPS[1]))
                {
                    TboxBind_st_Rflag = 0;
                    TboxBind_GPSID_Rflag = 0;
                    TboxBind_st_Stor = TboxBind_st;
                }
            }
        }

        if(TboxLock_Level_st != TboxLock_Level_St_Stor)
        {
            I2CWriteInt32WaitMode(TBOX_Lock_Level_EE_ADDR,TboxLock_Level_st);
            I2CReadInt32WaitMode(TBOX_Lock_Level_EE_ADDR,(uint32_t*)&ReadData);
            if(ReadData == TboxLock_Level_st)
            {
                TboxLock_Level_St_Stor = TboxLock_Level_st;
                if(TboxLock_Level_St_Stor == UnLock)
                {
                    TboxLock_Level_Action_st = UnLock;
                }
                Tbox_Lock_Unlock_st = Unlock_Lock_Success;
            }

        }


}

void VehicleCtrlTboxMsg_Updata​(uint8_t* Msg_data)
{
    Tbox_DMCU_Bind_Lock_st.st.GPS_Adv_st = TboxBind_Advanced_Stor;
    Tbox_DMCU_Bind_Lock_st.st.GPS_Sta_st = TboxBind_Standard_Stor;
    Tbox_DMCU_Bind_Lock_st.st.Tbox_Lock_st = TboxLock_Level_St_Stor&0x03;
    Tbox_DMCU_Bind_Lock_st.st.Tbox_Lock_Resp_st = Tbox_Lock_Unlock_st;
    Tbox_DMCU_Bind_Lock_st.st.Tbox_Bind_Comm_st = TboxCommErr_st;
    
    Msg_data[0] = Tbox_DMCU_Bind_Lock_st.MsgData;
    Msg_data[1] = MCUID_DATA.U8t[0];
    Msg_data[2] = MCUID_DATA.U8t[1];
    Msg_data[3] = MCUID_DATA.U8t[2];
    Msg_data[4] = MCUID_DATA.U8t[3];
    Msg_data[5] = MCUID_DATA.U8t[4];
    Msg_data[6] = MCUID_DATA.U8t[5];
    Msg_data[7] = 0x01; 
}

uint8_t VehicleCtrlGetLockSt​(void)
{
    return TboxLock_Level_Action_st;
}

//PWM control of the main contactor
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
     if (htim->Instance == TIM6)
     {
       RLY_pwm_cnt++;
       if (RLY_pwm_cnt >= RLY_PWM_PERIOD)
       RLY_pwm_cnt = 0;
       
       if (RLY_pwm_cnt < RLY_pwm_duty)
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
       else
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

     }

}



void Main_Contactor_On(void)
{
    if(Ctrl_Vin>=80)	//40%
    {
        RLY_pwm_duty = 40;
    }
    else if(Ctrl_Vin>=70)	//30%
    {
        RLY_pwm_duty = 30;
    }
    else if(Ctrl_Vin>=60)	//20%
    {
        RLY_pwm_duty = 20;
    }
    else if(Ctrl_Vin>=50)	//10%
    {
        RLY_pwm_duty = 10;
    }
    else if(Ctrl_Vin>=40)	//5%
    {
        RLY_pwm_duty = 5;
    }
    else 	                //0%
    {
        RLY_pwm_duty = 0;
    }

}

void Main_Contactor_Off(void)
{
        RLY_pwm_duty = 0;  //考虑是否关闭pwm
}


