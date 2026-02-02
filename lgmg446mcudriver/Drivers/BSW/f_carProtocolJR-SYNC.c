/****************************************************************** 
����������Function Description��:�綯����������CANͨѶЭ��Ĵ���      
����޸����ڣ�Date���� 
�޸���־��History��:�����¼�¼Ϊ��һ��ת���Ժ󣬿�ʼ��¼�� 
����          ʱ��                ����˵�� 
1  xx         xxxxx          	xxxxxxx 
2  yy         yyyyy          	yyyyyyy 
******************************************************************/ 
	

#include "f_carProtocolJR-SYNC.h"
#include "main.h"
#include "VehicleCtrl.h"
#include "MotorVC.h"
#include "MotorInclude.h"
#include "f_public.h"
#include "SDO_COB.h"

extern uint8_t TxMsgData[8];
extern uint8_t RxMsgData[8];

extern RotorSpeedStruct gRotorSpeed;
uint16_t   fanDuty = 0;
float PowerLimitP_D = 10;
float PowerLimitP_R = 10;


float MaxTroqLimit = 200.0f;
float MinTroqLimit = -200.0f;
float TorqueCtrlAccTime = 10000;
float TorqueCtrlDecTime = 10000;
float TorqCtrlMaxFwdFreqTemp = 500;
float TorqCtrlMaxRevFreqTemp = -500;
float g_f32ModeChangeSpeedRef = 0;


uint32_t Error_Info=0;




uint16_t  generatrixVoltage = 0;

int16_t MotorTemp2 = 0;
int16_t IGBTTemp = 0;
float ACC_Steering = 0.0;

const uint16_t FaultcodeTable[FAULT_CODE_NMB] = {3, 5, 8, 16, 18, 21, 22, 27, 28, 30, \
											  33, 34, 36, 60, 88, 93, 94, 95, 96, 97, \
											  100, 108, 110, 134, 142, 0, 92, 0, 0, 61, \
											  62, 63, 64};


uint16_t FaultcodeFlag[FAULT_CODE_NMB] = {0};

int    BmsCanLiftCnt = 0;
int    BmsCanErrFlag = 0;

int    LiftCanLiftCnt = 0;
int    LiftCanErrFlag = 0;

int    MeterCanLiftCnt = 0;
int    MeterCanErrFlag = 0;

int    TboxA1CanLiftCnt = 0;
int    TboxA1CanErrFlag = 0;

uint16_t    TboxA2CanLiftCnt = 0;
uint16_t    TboxA2CanErrFlag = 0;
uint16_t    TboxA2CanFirst_VFlag = 0;

int    TboxCanLiftCnt = 0;
int    TboxCanErrFlag = 0;

int    CanBusssoffCnt = 0;

uint16_t 	g_u16canerr_jr = 0;
double vehicle_spdFilter = 0;

IL_Tbox_DRV_MCU_0x6B68A1_U uIlRxTboxDrvMcu0x6B68A1;

IL_Tbox_DRV_MCU_0x6B68A2_U uIlRxTboxDrvMcu0x6B68A2;

uint16_t g_PumpRunStatus = 0;

void CarJRRcvFrameCalibrationCmd(void)
{
	uint8_t val[8];
	val[0] = RxMsgData[0];
	val[1] = RxMsgData[1];
	val[2] = RxMsgData[2];
	val[3] = RxMsgData[3];
	val[4] = RxMsgData[4];
	val[5] = RxMsgData[5];
	val[6] = RxMsgData[6];
	val[7] = RxMsgData[7];

    gDmdCanRe.EnableReq = val[0];
    gDmdCanRe.Direction = val[1];
    gDmdCanRe.ModeReq  = val[2];
    gCommProVar.iqSet = (uint16_t)(val[6] + (val[7]<<8));

}

//BMS —> VCU
void CarJRRcvFrameBMS0Deal(void)
{
	uint8_t val[8];
	BmsCanLiftCnt = 0;
	BmsCanErrFlag = 0;
    gFault3rd.bit.CanTimeOut = 0;
	val[0] = RxMsgData[0];
	val[1] = RxMsgData[1];
	val[2] = RxMsgData[2];
	val[3] = RxMsgData[3];
	val[4] = RxMsgData[4];
	val[5] = RxMsgData[5];
	val[6] = RxMsgData[6];
	val[7] = RxMsgData[7];

	if(val[0]!=0)
	{
		BMS_SOC = val[0]*0.4;
	}
	BMS_Current = (float)((val[3]+(uint16_t)(val[2]<<8))*0.1)-1000;
	BMS_Voltage = (float)((val[5]+(uint16_t)(val[4]<<8))*0.1);
	BMS_Faultlevel = (val[6]&0x03);
	BMS_ChrgMode = (val[6]&0x30)>>4;
	BMS_ChrgStatus = (val[6]&0xC0)>>6;


}

void CarJRRcvFrameBMS1Deal(void)
{


}

void CarJRRcvFrameBMS2Deal(void)
{

}

void CarJRRcvFrameBMS3Deal(void)
{


}

void CarJRRcvFrameBMS4Deal(void)
{


}

void CarJRRcvFrameBMS5Deal(void)
{


}

//仪表 —> VCU
void CarJRRcvFrameMETERDeal(void)
{
	uint8_t val[8];
	
	uint8_t i = 0;
	MeterCanErrFlag = 0;
	MeterCanLiftCnt = 0;
	val[0] = RxMsgData[0];
	val[1] = RxMsgData[1];
	val[2] = RxMsgData[2];
	val[3] = RxMsgData[3];
	val[4] = RxMsgData[4];
	val[5] = RxMsgData[5];
	val[6] = RxMsgData[6];
	val[7] = RxMsgData[7];


	if((val[1]&0x01) == 1)
	{
		if(HourClearFlag == 0)
		{
			workhour = 0;
			Powerhour = 0;
			Powerminute_cnt = 0;
			Workminute_cnt = 0;
			//DINT;
			
			for(i = 0; i < 10; i++)
			{
				I2CWriteInt32WaitMode(5000+i,Powerhour);
				I2CWriteInt32WaitMode(5010+i,workhour);
			}
			
			//EINT;
			HourClearFlag = 1;
		}

	}
	Drive_Mode = ((val[1]&0x06)>>1);		//3:P; 2:E; 1:S;
	Meter_access = ((val[1]&0x08)>>3);
}

void CarJRRcvFrameMETER2Deal(void)
{
	CanMsgSDO_SetDataID2(&RxMsgData[0]);
}

void CarJRRcvFrameMETER3Deal(void)
{
	CanMsgSDO_SetDataID3(&RxMsgData[0]);
}

void CarJRRcvFrameTboxA1Deal(void)
{
	//uint8_t val[8];
	//TboxA1CanErrFlag = 0;
	TboxA1CanLiftCnt = 1;
	uIlRxTboxDrvMcu0x6B68A1.byte[0] = RxMsgData[0];
	uIlRxTboxDrvMcu0x6B68A1.byte[1] = RxMsgData[1];
	uIlRxTboxDrvMcu0x6B68A1.byte[2] = RxMsgData[2];
	uIlRxTboxDrvMcu0x6B68A1.byte[3] = RxMsgData[3];
	uIlRxTboxDrvMcu0x6B68A1.byte[4] = RxMsgData[4];
	uIlRxTboxDrvMcu0x6B68A1.byte[5] = RxMsgData[5];
	uIlRxTboxDrvMcu0x6B68A1.byte[6] = RxMsgData[6];
	uIlRxTboxDrvMcu0x6B68A1.byte[7] = RxMsgData[7];

}

void CarJRRcvFrameTboxA2Deal(void)
{

	uIlRxTboxDrvMcu0x6B68A2.byte[0] = RxMsgData[0];
	uIlRxTboxDrvMcu0x6B68A2.byte[1] = RxMsgData[1];
	uIlRxTboxDrvMcu0x6B68A2.byte[2] = RxMsgData[2];
	uIlRxTboxDrvMcu0x6B68A2.byte[3] = RxMsgData[3];
	uIlRxTboxDrvMcu0x6B68A2.byte[4] = RxMsgData[4];
	uIlRxTboxDrvMcu0x6B68A2.byte[5] = RxMsgData[5];
	uIlRxTboxDrvMcu0x6B68A2.byte[6] = RxMsgData[6];
	uIlRxTboxDrvMcu0x6B68A2.byte[7] = RxMsgData[7];
	if(uIlRxTboxDrvMcu0x6B68A2.dat.BYTE0 == 0x01)
	{
		TboxA2CanErrFlag = 0;
		TboxA2CanLiftCnt = 0;
		TboxA2CanFirst_VFlag = 1;
		
	}

}

uint16_t PumpTargetSpeed = 0;

//PMCU —> DMCU
void CarJRRcvFramePumpInfo1Deal(void)
{
	uint8_t val[8];
	uint16_t speed = 0;
	val[0] = RxMsgData[0];
	val[1] = RxMsgData[1];
	val[2] = RxMsgData[2];
	val[3] = RxMsgData[3];
	val[4] = RxMsgData[4];
	val[5] = RxMsgData[5];
	val[6] = RxMsgData[6];
	val[7] = RxMsgData[7];

	LiftDownSignal = ((val[5] >> 6) & 0x01);

	speed = val[0];
	PumpTargetSpeed = (speed << 8) | val[1];
	LiftMCU_FaultLevel = (val[7]&0x3);
}

//PMCU —> DMCU
void CarJRRcvFramePumpInfo2Deal(void)
{
	uint8_t val[8];
	uint16_t speed = 0;
	val[0] = RxMsgData[0];
	val[1] = RxMsgData[1];
	val[2] = RxMsgData[2];
	val[3] = RxMsgData[3];
	val[4] = RxMsgData[4];
	val[5] = RxMsgData[5];
	val[6] = RxMsgData[6];
	val[7] = RxMsgData[7];

	g_PumpRunStatus = (val[7] & 0x0F);
}

//PMCU —> DMCU
void CarJRRcvFramePump2TboxDeal(void)
{
	
	uint8_t val[8];
	
	uint16_t speed = 0;
	LiftCanErrFlag = 0;
	LiftCanLiftCnt = 0;
	val[0] = RxMsgData[0];
	val[1] = RxMsgData[1];
	val[2] = RxMsgData[2];
	val[3] = RxMsgData[3];
	val[4] = RxMsgData[4];
	val[5] = RxMsgData[5];
	val[6] = RxMsgData[6];
	val[7] = RxMsgData[7];

	if(val[0]== 0x01)
	{
		LiftMCU_FaultCode = val[1];
	}

	if(val[0]== 0x02)
	{
		Pumphour = val[1] | ((0x0000ff & val[2]) << 8) | ((0x0000ff & val[3]) << 16);
		speed = val[5];

		LiftMCU_Speed = (speed << 8) | val[4];
	}
}


void CarJRFrameMETER1InfoUpdate(void)
{	

	float  m_Speed  = 0;
	int16_t m_SpeedOut = 0;

	if( IS_POSITIVE_ROTATE_WITH_SPEED_N )
	{
		m_Speed  = -gRotorSpeed.SpeedApplyFilter;				
	}
	else
	{
		m_Speed  = (gRotorSpeed.SpeedApplyFilter);
	}
	
	m_SpeedOut = (int16_t)(m_Speed*60/4) ;

	if(m_SpeedOut < 0)
	{
		m_SpeedOut = -m_SpeedOut;
	}
	if(Drive_Mode == 3)
	{
		TxMsgData[0] = 1;
	}
	else if(Drive_Mode == 2)
	{
		TxMsgData[0] = 2;
	}
	else if(Drive_Mode == 1)
	{
		TxMsgData[0] = 0;
	}
	else
	{
		
	}
	if(GearD_Enable == 1)
	{
		TxMsgData[0]|= 0x04;
	}
	else if(GearR_Enable == 1)
	{
		TxMsgData[0]|= 0x08;
	}
	else
	{
		TxMsgData[0]&=0xF3;
	}
	if(Seat_enable == 1)
	{
		TxMsgData[0]|=0x10;
	}
	else
	{
		TxMsgData[0]&=0xEF;
	}
	if(Lift_Lock != 0)
	{
		TxMsgData[0]|=0x20;
	}
	else
	{
		TxMsgData[0]&=0xDF;
	}
	if(BrakeSignalTest == 1)
	{
		TxMsgData[0]|=0x40;
	}
	else
	{
		TxMsgData[0]&=0xBF;
	}
	if(ParkHold == 1)
	{
		TxMsgData[0]|=0x80;
	}
	else
	{
		TxMsgData[0]&=0x7F;
	}
	TxMsgData[1] = Steering_Angle+90;
	TxMsgData[2] = BMS_SOC;
	TxMsgData[3] = vehicle_spdFilter;
	TxMsgData[4] = (m_SpeedOut&0xFF00)>>8;
	TxMsgData[5] = (m_SpeedOut&0x00FF);
	TxMsgData[6] = (LiftMCU_Speed&0xFF00)>>8;
	TxMsgData[7] = (LiftMCU_Speed&0x00FF);
	
//#if PROT_FLAG
//	CarJRMcuInfo1Update();
//#endif
}
void CarJRFrameMETER2InfoUpdate(void)
{	
	//check faultcode
	uint16_t flag = 0;
	static int fault_index = 0;
	uint16_t count = 0;
	IGBTTemp = (int16_t)(gTemperature.Temp);
	MotorTemp2 = (int16_t)(gTemperature.MotorTemp2);
	Meter_msg_index++;
	if(Meter_msg_index>4)
	{
		Meter_msg_index=1;
	}
	if(Meter_msg_index == 1)
	{
		TxMsgData[0] = Meter_msg_index;

		while(count < FAULT_CODE_NMB)
		{
			if(1==FaultcodeFlag[fault_index])
			{
				TxMsgData[1] = FaultcodeTable[fault_index];
				fault_index = (fault_index + 1)%FAULT_CODE_NMB;
				//Faultlevel = 3;
				break;
			}
			else
			{
				fault_index = (fault_index + 1)%FAULT_CODE_NMB;
				TxMsgData[1] = 0;
				//Faultlevel = 0;
			}
			count++;
		}


		/*
		uint16_t code = 0;
		for(i = 0; i < FAULT_CODE_NMB;i++)
		{
			code |= FaultcodeFlag[i];
		}
		
		if(code == 0)
		{
			val[1] = 0;
		}
		*/
		TxMsgData[2] = (I_phase&0xFF00)>>8;
		TxMsgData[3] = (I_phase&0x00FF);
		TxMsgData[4] = ((int16_t)(gTemperature.Temp*10)&0xFF00)>>8;
		TxMsgData[5] = ((int16_t)(gTemperature.Temp*10)&0x00FF);
		TxMsgData[6] = ((int16_t)(gTemperature.MotorTemp*10)&0xFF00)>>8;
		TxMsgData[7] = ((int16_t)(gTemperature.MotorTemp*10)&0x00FF);
	}
	else if(Meter_msg_index == 2)
	{
		TxMsgData[0] = Meter_msg_index;
		TxMsgData[1] = 0;
		TxMsgData[2] = 0;
		TxMsgData[3] = LiftMCU_FaultCode;
		TxMsgData[4] = ((unsigned int)ACC_AD&0xFF00)>>8;
		TxMsgData[5] = ((unsigned int)ACC_AD&0x00FF);
		TxMsgData[6] = ((unsigned int)ACC_Steering&0xFF00)>>8;
		TxMsgData[7] = ((unsigned int)ACC_Steering&0x00FF);
	}
	else if(Meter_msg_index == 3)
	{
		TxMsgData[0] = Meter_msg_index;
		TxMsgData[1] = (Powerhour&0x000000FF);
		TxMsgData[2] = ((Powerhour&0x0000FF00)>>8);
		TxMsgData[3] = ((Powerhour&0x00FF0000)>>16);
		TxMsgData[4] = (Pumphour&0x000000FF);
		TxMsgData[5] = ((Pumphour&0x0000FF00)>>8);
		TxMsgData[6] = ((Pumphour&0x00FF0000)>>16);
		TxMsgData[7] = 0;
	}
	else if(Meter_msg_index == 4)
	{
		TxMsgData[0] = Meter_msg_index;
		TxMsgData[1] = (uint8_t)((int16_t)(fabs(m_SpeedAim) )>>8);
		TxMsgData[2] = (uint8_t)((int16_t)(fabs(m_SpeedAim) ) & 0x00FF);
		TxMsgData[3] = (uint8_t)(PumpTargetSpeed >> 8);
		TxMsgData[4] = (uint8_t)(PumpTargetSpeed & 0x00FF);
		TxMsgData[5] = 0;
		TxMsgData[6] = 0;
		TxMsgData[7] = 0;
	}
	else
	{
	}

//#if PROT_FLAG
//	CarJRMcuInfo2Update();
//#endif
}
// TBOX��Ϣ
void CarJRFrameTBOXInfoUpdate(void)
{	
	int16_t  m_TrqOut = 0;
	float  m_Speed  = 0;
	uint16_t m_SpeedOut = 0;
	static int fault_index = 0;
	uint16_t count = 0;

	if( IS_POSITIVE_ROTATE_WITH_SPEED_N )
	{
		m_Speed  = -gRotorSpeed.SpeedApplyFilter;				
	}
	else
	{
		m_Speed  = (gRotorSpeed.SpeedApplyFilter);
	}
	if(m_Speed<0)
	{
		m_Speed = 0 - m_Speed;
	}

	m_SpeedOut = (uint16_t)(m_Speed*60/gMotorInfo.Poles) ;

	vehicle_spd = 0.02889 * m_SpeedOut;
	if (vehicle_spd != 0)
	{
		vehicle_spdFilter = 0.95f* vehicle_spdFilter + 0.05f*vehicle_spd;
	}
	else
	{
		vehicle_spdFilter = 0;
	}

	Tbox_msg_index++;
	if(Tbox_msg_index>6)
	{
		Tbox_msg_index=1;
	}
	if(Tbox_msg_index == 5)
	{
		Tbox_msg_index++;
	}
	I_phase = (uint16_t)(gLineCur.CurrentFilter*Cnst_1divsqrt2);
	m_TrqOut = -gPowerTrq.TrqOut*10;
	if(Tbox_msg_index == 1)
	{
		TxMsgData[0] = Tbox_msg_index;

		while(count < FAULT_CODE_NMB)
		{
			if(FaultcodeFlag[fault_index])
			{
				TxMsgData[1] = FaultcodeTable[fault_index];
				fault_index = (fault_index + 1)%FAULT_CODE_NMB;
				break;
			}
			else
			{
				fault_index = (fault_index + 1)%FAULT_CODE_NMB;
				TxMsgData[1] = 0;
				//Faultlevel = 0;
			}
			count++;
		}
		
		TxMsgData[2] = (I_phase&0x00FF);
		TxMsgData[3] = (I_phase&0xFF00)>>8;
		TxMsgData[4] = ((int16_t)(gTemperature.Temp*10)&0x00FF);
		TxMsgData[5] = ((int16_t)(gTemperature.Temp*10)&0xFF00)>>8;
		TxMsgData[6] = ((int16_t)(gTemperature.MotorTemp*10)&0x00FF);
		TxMsgData[7] = ((int16_t)(gTemperature.MotorTemp*10)&0xFF00)>>8;
	}
	else if(Tbox_msg_index == 2)
	{
		TxMsgData[0] = Tbox_msg_index;
		TxMsgData[1] = (Powerhour&0x000000FF);
		TxMsgData[2] = ((Powerhour&0x0000FF00)>>8);
		TxMsgData[3] = ((Powerhour&0x00FF0000)>>16);
		TxMsgData[4] = (workhour&0x000000FF);
		TxMsgData[5] = ((workhour&0x0000FF00)>>8);
		TxMsgData[6] = ((workhour&0x00FF0000)>>16);
		TxMsgData[7] = BMS_SOC;
	}
	else if(Tbox_msg_index == 3)
	{
		TxMsgData[0] = Tbox_msg_index;
		TxMsgData[1] = Steering_Angle;
		TxMsgData[2] = ((unsigned int)vehicle_spdFilter&0x00FF);
		TxMsgData[3] = ((unsigned int)vehicle_spdFilter&0xFF00)>>8;
		TxMsgData[4] = (m_SpeedOut&0x00FF);
		TxMsgData[5] = (m_SpeedOut&0xFF00)>>8;
		TxMsgData[6] = 0xFF;
		TxMsgData[7] = 0xFF;
	}
	else if(Tbox_msg_index == 4)
	{
		TxMsgData[0] = Tbox_msg_index;
		TxMsgData[1] = ((unsigned int)ACC_AD&0x00FF);
		TxMsgData[2] = ((unsigned int)ACC_AD&0xFF00)>>8;
		TxMsgData[3] = Remote_Lock_Status;
		if(Drive_Mode == 0)
		{
			TxMsgData[4] = 2;
		}
		else if(Drive_Mode == 1)
		{
			TxMsgData[4] = 0;
		}
		else if(Drive_Mode == 2)
		{
			TxMsgData[4] = 1;
		}
		else
		{
			TxMsgData[4] = 0xFF;
		}
		//Ctrl_Vin = 79.99;
		TxMsgData[5] = (((int)(Ctrl_Vin*100))&0x00FF);
		TxMsgData[6] = (((int)(Ctrl_Vin*100))&0xFF00)>>8;

		TxMsgData[7] = SeatSignal+(SeatBelt<<1) + (GearD_Enable<<2) + (GearR_Enable <<3) + (BrakeSignalTest<<4) + (ParkHold<<5) + (ACC_enable <<6) + (MainContactorStatus << 7);
	}
	else if(Tbox_msg_index == 6)
	{
		TxMsgData[0] = Tbox_msg_index;
		TxMsgData[1] = 0x40;
		TxMsgData[2] = (m_TrqOut&0x00FF);
		TxMsgData[3] = (m_TrqOut&0xFF00)>>8;
		TxMsgData[4] = 0xFF;
		TxMsgData[5] = 0xFF;
		TxMsgData[6] = 0xFF;
		TxMsgData[7] = 0xFF;
	}
}

void CarJRDriverTboxUpdate(void)
{	
	VehicleCtrlTboxMsg_Updata​(&TxMsgData[0]);
}

void CarJRDriverInfo1Update(void)
{	
	TxMsgData[0] = (uint8_t)((int16_t)(Pre_LoopOut)&0x00FF);
	TxMsgData[1] = (uint8_t)((int16_t)(Pre_LoopOut)>>8);
	TxMsgData[2] = (uint8_t)((int16_t)(KI_Result+1000)&0x00FF);
	TxMsgData[3] = (uint8_t)((int16_t)(KI_Result+1000)>>8);
	TxMsgData[4] = Flag_Anti +(Flag_Behind<<1) + (Flag_Front<<2) + (Flag_Stop<<3) +(ACC_Enable<<4) + (Flag_Fast2Zero<<5) + (Flag_De<<6) + (Flag_In<<7);
	TxMsgData[5] = BrakeSignalTest + (KI_Reset_Flag1<<1);

	TxMsgData[6] = SOFTWARE_VERSION;
	TxMsgData[7] = Faultlevel + (PROJECT_VERSION << 2);
}

void CarJRDriverInfo2Update(void)
{

	uint16_t DFaultlevel = 0;

	if (gFault3rd.all != 0)  //行驶控制器故障
	{
		DFaultlevel = 3;
	}
	else if (gFault2nd.all != 0)
	{
		DFaultlevel = 2;
	}
	else
	{
		DFaultlevel = 0;
	}

	TxMsgData[0] = g_PwmAlreadyOn +(KeyOn_Enable<<1) + (gDmdCanRe.EnableReq<<2) + (gADOffsetInfo.Flag<<3) +(PreChargeStatus<<4) + (PreCharOnFlag<<5) + (DFaultlevel<<6);

	TxMsgData[1] = (Uint8)(gFault3rd.all&0x00FF);
	TxMsgData[2] = (Uint8)(gFault3rd.all>>8);

	TxMsgData[3] = (Uint8)(gFault2nd.all&0x00FF);
	TxMsgData[4] = (Uint8)(gFault2nd.all>>8);

	TxMsgData[5] = (Uint8)((uint16_t)gUDC.uDCBigFilter&0x00FF);
	TxMsgData[6] = (Uint8)((uint16_t)gUDC.uDCBigFilter>>8);

	TxMsgData[7] = g_RunStatus + (Lift_Lock << 4);

}

void CarJRDriverInfo3Update(void)
{
	float  m_Speed  = 0;
	uint16_t m_SpeedOut = 0;

	uint16_t SpeedAim = 0;
	uint16_t SpeedAim_Pre = 0;

	uint16_t AsrRef = 0;
	uint16_t AsrFb = 0;

	m_Speed  = (gRotorSpeed.SpeedApplyFilter);
	m_SpeedOut = (int16_t)(m_Speed*60/4) ;

	SpeedAim = (int16_t)m_SpeedAim;
	SpeedAim_Pre = (int16_t)m_SpeedAim_Pre;

	AsrRef = (int16_t)(gAsr.Asr.Ref*60/4);
	AsrFb = (int16_t)(gAsr.Asr.Fb*60/4);

	TxMsgData[0] = (m_SpeedOut&0xFF00)>>8;
	TxMsgData[1] = (m_SpeedOut&0x00FF);

	TxMsgData[2] = (SpeedAim&0xFF00)>>8;
	TxMsgData[3] = (SpeedAim&0x00FF);

	TxMsgData[4] = (AsrRef&0x00FF);
	TxMsgData[5] = (AsrRef&0xFF00)>>8;

	TxMsgData[7] = ((unsigned int)ACC_AD&0xFF00)>>8;
	TxMsgData[6] = ((unsigned int)ACC_AD&0x00FF);
}

void CarJRDriverInfo4Update(void)
{
	double m_PowerLimit = 0;

	m_PowerLimit = PowerLimit;

	if (m_PowerLimit >125)
	{
		m_PowerLimit = 125;
	}


	TxMsgData[0] = (Uint8)((int16_t)(gAsr.Asr.Kp*10));
	TxMsgData[1] = (Uint8)((int16_t)(gAsr.Asr.Ki*1000));

	TxMsgData[2] = ((int16_t)KP_Result&0x00FF);
	TxMsgData[3] = ((int16_t)KP_Result&0xFF00)>>8;

	TxMsgData[4] = ((int16_t)gAsr.Asr.PIOut&0x00FF);
	TxMsgData[5] = ((int16_t)gAsr.Asr.PIOut&0xFF00)>>8;

	TxMsgData[6] = ((int16_t)(g_TorqRefComp.TRefCompVal * 10)&0x00FF);
	TxMsgData[7] = ((int16_t)(g_TorqRefComp.TRefCompVal * 10)&0xFF00)>>8;
}
