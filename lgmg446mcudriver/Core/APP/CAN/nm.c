#include "def.h"
#include "il.h"
#include "nm.h"
#include "SDO_COB.h"
/*====================CAN1 TX===================*/
NM_DRV_MCU1_VCU1_0x88_T  tNmTxDrvMcu1Vcu10x88;
NM_DRV_MCU1_VCU2_0x89_T  tNmTxDrvMcu1Vcu20x89;
NM_DRV_RT_INFO_0x90_T    tNmTxDrvRtInfo0x90;
NM_DRV_MCU1_VCU3_0x188_T tNmTxDrvMcu1Vcu30x188;
NM_DRV_MCU1_VCU4_0x288_T tNmTxDrvMcu1Vcu40x288;

NM_DRV_MCU1_INFO_0x582_T tNmTxDrvMcu1INFOx582;
NM_DRV_MCU1_INFO_0x583_T tNmTxDrvMcu1INFOx583;
/*====================CAN1 RX===================*/
NM_BMS_INFO_0x53_T     tNmRxBmsInfo0x53={1,1};
NM_BRAKE_INFO_0x1CA_T  tNmRxBrakeInfo0x1CA={1,1};
NM_VCU_DRV_MCU_0x208_T tNmRxVcuDrvMcu0x208={1,1};

/*====================CAN2 TX===================*/
NM_TX_0x4616401_T tNmTx0x4616401;
/*====================CAN2 RX===================*/
NM_RX_0x4300164_T tNmRxReadPara0x4300164={1,1};
NM_RX_0x4300164_T tNmRxSavePara0x4500164={1,1};
NM_RX_0x4300164_T tNmRxSetPara0x4600164={1,1};

void NM_CanRxMessageAnalysis0x53(uint8 *pdat)
{
	IL_BMS_INFO_0x53_U *pmsg;

	if(pdat!=NULL)
	{
		pmsg =(IL_BMS_INFO_0x53_U *) pdat;
		tNmRxBmsInfo0x53.msgMiss = 0;
		tNmRxBmsInfo0x53.msgNever  = 0;
		tNmRxBmsInfo0x53.msgUpd = 1;

		tNmRxBmsInfo0x53.mainNegRlySta = pmsg->dat.mainNegRlySta;
	}
	else
	{
		tNmRxBmsInfo0x53.msgMiss = 1;
	}
}
void NM_CanRxMessageAnalysis0x1CA(uint8 *pdat)
{
	IL_BRAKE_INFO_0x1CA_U *pmsg;
	if(pdat!=NULL)
	{
		pmsg =(IL_BRAKE_INFO_0x1CA_U *) pdat;
		tNmRxBrakeInfo0x1CA.msgMiss = 0;
		tNmRxBrakeInfo0x1CA.msgNever  = 0;
		tNmRxBrakeInfo0x1CA.msgUpd = 1;

		tNmRxBrakeInfo0x1CA.slopeAntiEnable = pmsg->dat.slopeAntiEnable;
		tNmRxBrakeInfo0x1CA.slopeAntiFlag2 = pmsg->dat.slopeAntiFlag2;
		tNmRxBrakeInfo0x1CA.brakeSignal 	= pmsg->dat.brakeSignal;
		tNmRxBrakeInfo0x1CA.slopeSigFlag 	= pmsg->dat.slopeSigFlag;
		tNmRxBrakeInfo0x1CA.bmsDcCurrent 	= pmsg->dat.bmsDcCurrent;
	}
	else
	{
		tNmRxBrakeInfo0x1CA.msgMiss = 1;
	}
}
void NM_CanRxMessageAnalysis0x208(uint8 *pdat)
{
	IL_VCU_DRV_MCU_0x208_U *pmsg;
	if(pdat!=NULL)
	{
		pmsg =(IL_VCU_DRV_MCU_0x208_U *) pdat;
		tNmRxVcuDrvMcu0x208.msgMiss = 0;
		tNmRxVcuDrvMcu0x208.msgNever  = 0;
		tNmRxVcuDrvMcu0x208.msgUpd = 1;

		tNmRxVcuDrvMcu0x208.reqSpd = pmsg->dat.reqSpd;
		tNmRxVcuDrvMcu0x208.enable = pmsg->dat.enable;
		tNmRxVcuDrvMcu0x208.contactorEnable 	= pmsg->dat.contactorEnable;
		tNmRxVcuDrvMcu0x208.direction 	= pmsg->dat.direction;
		tNmRxVcuDrvMcu0x208.diffSwitch 	= pmsg->dat.diffSwitch;
	}
	else
	{
		tNmRxVcuDrvMcu0x208.msgMiss = 1;
	}
}


void NM_CanRxMessageAnalysis0x602(uint8 *pdat)
{
	IL_VCU_DRV_MCU_0x602_U *pmsg;
	if(pdat!=NULL)
	{
		CanMsgSDO_SetDataID2(pdat);

	}
	else
	{
		//tNmRxVcuDrvMcu0x208.msgMiss = 1;
	}
}

void NM_CanRxMessageAnalysis0x603(uint8 *pdat)
{
	IL_VCU_DRV_MCU_0x603_U *pmsg;
	if(pdat!=NULL)
	{
		CanMsgSDO_SetDataID3(pdat);

	}
	else
	{
		//tNmRxVcuDrvMcu0x208.msgMiss = 1;
	}
}
/*====================CAN2 RX===================*/
void NM_CanRxMessageAnalysis4300164(uint8 *pdat)
{
	IL_RX_USER_PARA_U *pmsg;

	if(pdat!=NULL)
	{
		pmsg =(IL_RX_USER_PARA_U *) pdat;
		tNmRxReadPara0x4300164.msgMiss = 0;
		tNmRxReadPara0x4300164.msgNever  = 0;
		tNmRxReadPara0x4300164.msgUpd = 1;

		tNmRxReadPara0x4300164.paraAdd = pmsg->dat.paraAdd;
		tNmRxReadPara0x4300164.paraVal = pmsg->dat.paraVal;
	}
	else
	{
		tNmRxReadPara0x4300164.msgMiss = 1;
	}
}
void NM_CanRxMessageAnalysis4500164(uint8 *pdat)
{
	IL_RX_USER_PARA_U *pmsg;

	if(pdat!=NULL)
	{
		pmsg =(IL_RX_USER_PARA_U *) pdat;
		tNmRxSavePara0x4500164.msgMiss = 0;
		tNmRxSavePara0x4500164.msgNever  = 0;
		tNmRxSavePara0x4500164.msgUpd = 1;

		tNmRxSavePara0x4500164.paraAdd = pmsg->dat.paraAdd;
		tNmRxSavePara0x4500164.paraVal = pmsg->dat.paraVal;
	}
	else
	{
		tNmRxSavePara0x4500164.msgMiss = 1;
	}
}
void NM_CanRxMessageAnalysis4600164(uint8 *pdat)
{
	IL_RX_USER_PARA_U *pmsg;

	if(pdat!=NULL)
	{
		pmsg =(IL_RX_USER_PARA_U *) pdat;
		tNmRxSetPara0x4600164.msgMiss = 0;
		tNmRxSetPara0x4600164.msgNever  = 0;
		tNmRxSetPara0x4600164.msgUpd = 1;

		tNmRxSetPara0x4600164.paraAdd = pmsg->dat.paraAdd;
		tNmRxSetPara0x4600164.paraVal = pmsg->dat.paraVal;
	}
	else
	{
		tNmRxSetPara0x4600164.msgMiss = 1;
	}
}


