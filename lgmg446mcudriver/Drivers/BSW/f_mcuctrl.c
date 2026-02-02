/****************************************************************** 
锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷Function Description锟斤拷:锟界动锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷CAN通讯协锟斤拷拇锟斤拷锟?
锟斤拷锟斤拷薷锟斤拷锟斤拷冢锟紻ate锟斤拷锟斤拷
锟睫革拷锟斤拷志锟斤拷History锟斤拷:锟斤拷锟斤拷锟铰硷拷录为锟斤拷一锟斤拷转锟斤拷锟皆后，匡拷始锟斤拷录锟斤拷
锟斤拷锟斤拷          时锟斤拷                锟斤拷锟斤拷说锟斤拷
1  xx         xxxxx          	xxxxxxx 
2  yy         yyyyy          	yyyyyyy 
******************************************************************/ 
	


#include "f_public.h"

float gIdTSetM = 0;
float gIqTSetM = 0;

extern uint8_t TxMsgData[8];	
extern uint8_t RxMsgData[8];	


extern  uint16_t G_dgghdhw;
extern  uint16_t G_ErrStep;
extern  uint16_t gAdcResult1_iw;
extern  uint16_t gAdcResult1_iv;

extern uint16_t gASR_KP;
extern uint16_t gASR_Ki;
extern int16_t gASR_OUTkp;
extern int16_t gASR_OUTki;
extern int16_t gASR_SpdGmd;
extern int16_t gASR_SpdFb;

int16_t ColdFlag = 0;
void CarJRMcuCmd1Deal(void)
{
	uint8_t val[8];

	uint16_t FreqCmd = 0;



	val[0] = RxMsgData[0];
	val[1] = RxMsgData[1];
	val[2] = RxMsgData[2];
	val[3] = RxMsgData[3];
	val[4] = RxMsgData[4];
	val[5] = RxMsgData[5];
	val[6] = RxMsgData[6];
	val[7] = RxMsgData[7];

	if (val[0] == 1)
	{
//		if (g_PwmAlreadyOn == 0)
//		{
//			RunDataIntit();
//		}

		gDmdCanRe.EnableReq = 1;
	}
	else
	{
		gDmdCanRe.EnableReq =  0;
	}

	if (val[1] == 1)
	{
		gDmdCanRe.Direction =  1;
	}
	else
	{
		gDmdCanRe.Direction =  0;
	}

	FreqCmd = (val[2]+(val[3]<<8));

    gDmdCanRe.FreqCmd =  ( 0.0167f*FreqCmd*gMotorInfo.Poles );

    gDmdCanRe.TorqueCmd =  val[6];


	if (val[4] == 0)
	{
		gDmdCanRe.ModeReq = 0;
	}
	else if (val[4] == 1)
	{
		if (ColdFlag == 0)
		{
			ColdFlag = 1;
			gDmdCanRe.ModeReq = PARAMEST_CMD;
		}

	}
	else if (val[4] == 2)
	{
		gDmdCanRe.ModeReq = SPEEDCTRL_CMD;
	}
	else if (val[4] == 3)
	{
		gDmdCanRe.ModeReq = TORQUECTRL_CMD;
	}

//	if (val[0] == 1)
//	{
//		if ((g_PwmAlreadyOn == 0) && (gFault3rd.all == 0))
//		{
//			RunDataIntit();
//			g_PwmEnable = 1;
//		}
//	}
//	else
//	{
//		if (g_PwmAlreadyOn == 1)
//		{
//			g_PwmEnable = 0;
//			DisableDrive();
//		}
//	}
//
//
//	gIdTSetM = (float)(int16_t)(val[2]+(uint16_t)(val[3]<<8));
//	gIqTSetM = (float)(int16_t)(val[4]+(uint16_t)(val[5]<<8));


}
uint16_t g_ydwCrr = 0;
void CarJRMcuCmd2Deal(void)
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

	if (val[7] == 1)
	{
		g_ydwCrr++;
		DisableDrive();
	}
}






void CarJRMcuInfo1Update(void)
{	
	float  m_Speed  = 0;
	int16_t m_SpeedOut = 0;

	m_Speed = fabs(g_freqSet);
	m_SpeedOut = (uint16_t)(m_Speed*60/gMotorInfo.Poles) ;


	TxMsgData[0] = (Uint8)((int16)(gPowerTrq.TorqueRef * 10 + 400)&0x00FF); //gTemperature.Temp
	TxMsgData[1] = (Uint8)((int16)(gPowerTrq.TorqueRef * 10 + 400)>>8);  //gTemperature.Temp
	TxMsgData[2] = (Uint8)((int16)(gTemperature.MotorTemp * 10 + 600)&0x00FF);
	TxMsgData[3] = (Uint8)((int16)(gTemperature.MotorTemp * 10 + 600)>>8);
//	TxMsgData[0] = (Uint8)((int16)(m_SpeedOut * 10 + 400)&0x00FF);
//	TxMsgData[1] = (Uint8)((int16)(m_SpeedOut * 10 + 400)>>8);
//	TxMsgData[2] = (Uint8)((int16)(gAsr.Asr.PIOut * 10 + 600)&0x00FF);
//	TxMsgData[3] = (Uint8)((int16)(gAsr.Asr.PIOut * 10 + 600)>>8);
	TxMsgData[4] = (Uint8)(g_RunStatus&0x00FF);
	TxMsgData[5] = (Uint8)(g_PwmAlreadyOn>>8);

	TxMsgData[6] = (Uint8)(gDmdCanRe.EnableReq&0x00FF);
	TxMsgData[7] = (Uint8)(gDmdCanRe.EnableReq>>8);


//	TxMsgData[0] = (Uint8)(gASR_KP & 0x00FF);
//	TxMsgData[1] = (Uint8)(gASR_Ki & 0x00FF);
//	TxMsgData[2] = (Uint8)((int16)(gAsr.Asr.PIOut * 10 + 600)&0x00FF);
//	TxMsgData[3] = (Uint8)((int16)(gAsr.Asr.PIOut * 10 + 600)>>8);
//	TxMsgData[4] = (Uint8)(gASR_OUTkp &0x00FF);
//	TxMsgData[5] = (Uint8)(gASR_OUTkp >>8);
//
//	TxMsgData[6] = (Uint8)(gASR_OUTki&0x00FF);
//	TxMsgData[7] = (Uint8)(gASR_OUTki>>8);


}
void CarJRMcuInfo2Update(void)
{	
	float  m_Speed  = 0;
	int16_t m_SpeedOut = 0;

	m_Speed = gRotorSpeed.SpeedApply;//gRotorSpeed.SpeedApply;//gRotorSpeed.SpeedApplyFilter;
	m_SpeedOut = (int16_t)(m_Speed*60/gMotorInfo.Poles) ;

		TxMsgData[0] = (Uint8)(gRotorTrans.RTPos&0x00FF);
		TxMsgData[1] = (Uint8)(gRotorTrans.RTPos>>8);
//		TxMsgData[2] = (Uint8)(G_ErrStep &0x00FF);
		TxMsgData[2] = (Uint8)(m_SpeedOut &0x00FF);
		TxMsgData[3] = (Uint8)(m_SpeedOut >>8);
		TxMsgData[4] = (Uint8)(gFault3rd.all&0x00FF);
		TxMsgData[5] = (Uint8)(gFault3rd.all>>8);

		TxMsgData[6] = (Uint8)((int16)(Ctrl_Vin*10)&0x00FF);
		TxMsgData[7] = (Uint8)((int16)(Ctrl_Vin*10)>>8);

//		TxMsgData[0] = (Uint8)(u16AdcResult3_tempIGBT&0x00FF);
//		TxMsgData[1] = (Uint8)(u16AdcResult3_tempIGBT>>8);
////		TxMsgData[2] = (Uint8)(G_ErrStep &0x00FF);
//		TxMsgData[2] = (Uint8)(u16AdcResult4_KSI &0x00FF);
//		TxMsgData[3] = (Uint8)(u16AdcResult4_KSI >>8);
//		TxMsgData[4] = (Uint8)(u16AdcResult9_VDC&0x00FF);
//		TxMsgData[5] = (Uint8)(u16AdcResult9_VDC>>8);
//
//		TxMsgData[6] = (Uint8)((int16)(Ctrl_Vin*10)&0x00FF);
//		TxMsgData[7] = (Uint8)((int16)(Ctrl_Vin*10)>>8);



}
// TBOX锟斤拷息
void CarJRMcuInfo3Update(void)
{	
	TxMsgData[0] = (Uint8)((uint16_t)(gIMTSetApply.M + 3000)&0x00FF);
	TxMsgData[1] = (Uint8)((uint16_t)(gIMTSetApply.M + 3000)>>8);
	TxMsgData[2] = (Uint8)((uint16_t)(gIMTSetApply.T + 3000)&0x00FF);
	TxMsgData[3] = (Uint8)((uint16_t)(gIMTSetApply.T + 3000)>>8);

	TxMsgData[4] = (Uint8)((uint16_t)(gIMT.M + 3000)&0x00FF);
	TxMsgData[5] = (Uint8)((uint16_t)(gIMT.M + 3000)>>8);
	TxMsgData[6] = (Uint8)((uint16_t)(gIMT.T + 3000)&0x00FF);
	TxMsgData[7] = (Uint8)((uint16_t)(gIMT.T + 3000)>>8);


}



void CarJRMcuInfo4Update(void)
{	
	TxMsgData[0] = (Uint8)((int16)(gCurSamp.U + 3000)&0x00FF);
	TxMsgData[1] = (Uint8)((int16)(gCurSamp.U + 3000)>>8);
	TxMsgData[2] = (Uint8)((int16)(gCurSamp.V + 3000)&0x00FF);
	TxMsgData[3] = (Uint8)((int16)(gCurSamp.V + 3000)>>8);
	TxMsgData[4] = (Uint8)((int16)(gCurSamp.W + 3000)&0x00FF);
	TxMsgData[5] = (Uint8)((int16)(gCurSamp.W + 3000)>>8);

	TxMsgData[6] = (Uint8)((int16)(gUDC.uDCFilter*10)&0x00FF);
	TxMsgData[7] = (Uint8)((int16)(gUDC.uDCFilter*10)>>8);


}

