#include <string.h>
#include "def.h"
#include "can.h"
#include "uds.h"
#include "il.h"
#include "nm.h"

/*====================CAN1 TX===================*/
IL_DRV_MCU1_VCU1_0x88_U  uIlTxDrvMcu1Vcu10x88;
IL_DRV_MCU1_VCU2_0x89_U  uIlTxDrvMcu1Vcu20x89;
IL_DRV_RT_INFO_0x90_U    uIlTxDrvRtInfo0x90;
IL_DRV_MCU1_VCU3_0x188_U uIlTxDrvMcu1Vcu30x188;
IL_DRV_MCU1_VCU4_0x288_U uIlTxDrvMcu1Vcu40x288;
IL_DRV_MCU1_INFO_0x582_T uIlTxDrvMcuINFOx582;
IL_DRV_MCU1_INFO_0x583_T uIlTxDrvMcuINFOx583;

/*====================CAN1 RX===================*/
IL_BMS_INFO_0x53_U     uIlRxBmsInfo0x53;
IL_BRAKE_INFO_0x1CA_U  uIlRxBrakeInfo0x1CA;
IL_VCU_DRV_MCU_0x208_U uIlRxVcuDrvMcu0x208;
IL_VCU_DRV_MCU_0x602_U uIlRxVcuDrvMcu0x602;
IL_VCU_DRV_MCU_0x603_U uIlRxVcuDrvMcu0x603;
/*====================CAN2 TX===================*/
IL_TX_0x4616401_U uIlTxParaSend0x4616401;
/*====================CAN2 RX===================*/
IL_RX_USER_PARA_U uIlRxReadPara0x04300164;
IL_RX_USER_PARA_U uIlRxSavePara0x04500164;
IL_RX_USER_PARA_U uIlRxSetPara0x04600164;


uint16 Can1TxMsgNum;
uint16 Can1RxMsgNum;
uint16 Can2TxMsgNum;
uint16 Can2RxMsgNum;

void IL_FillSendMessage0x88(void);
void IL_FillSendMessage0x89(void);
void IL_FillSendMessage0x90(void);
void IL_FillSendMessage0x188(void);
void IL_FillSendMessage0x288(void);

void IL_FillSendMessage0x582(void);
void IL_FillSendMessage0x583(void);

void IL_FillSendMessage0x4616401(void);
/***************CAN1 Rx Message***************/
const MSG_RX_LIST_T Can1RxMsgList[]={
		{0x53, 100*5,(uint8 *)&uIlRxBmsInfo0x53,   &NM_CanRxMessageAnalysis0x53},
		{0x1CA,100*5,(uint8 *)&uIlRxBrakeInfo0x1CA,&NM_CanRxMessageAnalysis0x1CA},
		{0x208,100*5,(uint8 *)&uIlRxVcuDrvMcu0x208,&NM_CanRxMessageAnalysis0x208},
		{0x602,0,(uint8 *)&uIlRxVcuDrvMcu0x602,&NM_CanRxMessageAnalysis0x602},


		//{0x603,0,(uint8 *)&uIlRxVcuDrvMcu0x603,&NM_CanRxMessageAnalysis0x603},

};
MSG_RX_STATE_T Can1RxMsgSta[sizeof(Can1RxMsgList)/sizeof(MSG_RX_LIST_T)];
/***************CAN1 Tx Message***************/
const MSG_TX_LIST_T Can1TxMsgList[]={\
		{0x88 ,10,(uint8 *)&uIlTxDrvMcu1Vcu10x88, &IL_FillSendMessage0x88, &tNmTxDrvMcu1Vcu10x88.msgUpd},
		{0x89 ,10,(uint8 *)&uIlTxDrvMcu1Vcu20x89, &IL_FillSendMessage0x89, &tNmTxDrvMcu1Vcu20x89.msgUpd},
		{0x90 ,10,(uint8 *)&uIlTxDrvRtInfo0x90,   &IL_FillSendMessage0x90, &tNmTxDrvRtInfo0x90.msgUpd},
		{0x188,100,(uint8 *)&uIlTxDrvMcu1Vcu30x188,&IL_FillSendMessage0x188,&tNmTxDrvMcu1Vcu30x188.msgUpd},
		{0x288,100,(uint8 *)&uIlTxDrvMcu1Vcu40x288,&IL_FillSendMessage0x288,&tNmTxDrvMcu1Vcu40x288.msgUpd},
		{0x582,0,(uint8 *)&uIlTxDrvMcuINFOx582,&IL_FillSendMessage0x582,&tNmTxDrvMcu1INFOx582.msgUpd},
		
		//{0x583,0,(uint8 *)&uIlTxDrvMcuINFOx583,&IL_FillSendMessage0x583,&tNmTxDrvMcu1INFOx583.msgUpd},

};

uint16 Can1TxMsgSta[sizeof(Can1TxMsgList)/sizeof(MSG_TX_LIST_T)];

/***************CAN2 Rx Message***************/
const MSG_RX_LIST_T Can2RxMsgList[]={
		{0x04300164,0,(uint8 *)&uIlRxReadPara0x04300164,&NM_CanRxMessageAnalysis4300164},
		{0x04500164,0,(uint8 *)&uIlRxSavePara0x04500164,&NM_CanRxMessageAnalysis4500164},
		{0x04600164,0,(uint8 *)&uIlRxSetPara0x04600164, &NM_CanRxMessageAnalysis4600164},
};
MSG_RX_STATE_T Can2RxMsgSta[sizeof(Can2RxMsgList)/sizeof(MSG_RX_LIST_T)];
/***************CAN1 Tx Message***************/
const MSG_TX_LIST_T Can2TxMsgList[]={\
		{0x4316401,0,(uint8 *)&uIlTxParaSend0x4616401,&IL_FillSendMessage0x4616401,&tNmTx0x4616401.msgUpd},
};
uint16 Can2TxMsgSta[sizeof(Can2TxMsgList)/sizeof(MSG_TX_LIST_T)];

void IL_FillSendMessage0x88(void)
{
	uIlTxDrvMcu1Vcu10x88.dat.faultCode = tNmTxDrvMcu1Vcu10x88.faultCode;
	uIlTxDrvMcu1Vcu10x88.dat.spdReq    = tNmTxDrvMcu1Vcu10x88.spdReq + 1000;
	uIlTxDrvMcu1Vcu10x88.dat.spdReal   = tNmTxDrvMcu1Vcu10x88.spdReal+ 1000;
	uIlTxDrvMcu1Vcu10x88.dat.spdLoop   = tNmTxDrvMcu1Vcu10x88.spdLoop+ 1000;
}

void IL_FillSendMessage0x89(void)
{
	uIlTxDrvMcu1Vcu20x89.dat.faultCode 		= tNmTxDrvMcu1Vcu20x89.faultCode;
	uIlTxDrvMcu1Vcu20x89.dat.piRev    		= tNmTxDrvMcu1Vcu20x89.piRev + 1000;
	uIlTxDrvMcu1Vcu20x89.dat.brakeSignalFB   	= tNmTxDrvMcu1Vcu20x89.brakeSignalFB;
	uIlTxDrvMcu1Vcu20x89.dat.slopeAntiFlag2FB = tNmTxDrvMcu1Vcu20x89.slopeAntiFlag2FB;
	uIlTxDrvMcu1Vcu20x89.dat.slopeTriggerFB   = tNmTxDrvMcu1Vcu20x89.slopeTriggerFB;
	uIlTxDrvMcu1Vcu20x89.dat.piLock   		= tNmTxDrvMcu1Vcu20x89.piLock;
	uIlTxDrvMcu1Vcu20x89.dat.clearFlag   		= tNmTxDrvMcu1Vcu20x89.clearFlag;
	uIlTxDrvMcu1Vcu20x89.dat.brakeBuild   	= tNmTxDrvMcu1Vcu20x89.brakeBuild;
	uIlTxDrvMcu1Vcu20x89.dat.flagEnableOff   	= tNmTxDrvMcu1Vcu20x89.flagEnableOff;
	uIlTxDrvMcu1Vcu20x89.dat.mosTemp   		= tNmTxDrvMcu1Vcu20x89.mosTemp + 40;
}
void IL_FillSendMessage0x90(void)
{
	uIlTxDrvRtInfo0x90.dat.lot 		= tNmTxDrvRtInfo0x90.lot;
	uIlTxDrvRtInfo0x90.dat.dos 		= tNmTxDrvRtInfo0x90.dos;
	uIlTxDrvRtInfo0x90.dat.lotFilter= tNmTxDrvRtInfo0x90.lotFilter;
	uIlTxDrvRtInfo0x90.dat.dosFilter= tNmTxDrvRtInfo0x90.dosFilter;
}


void IL_FillSendMessage0x188(void)
{
	uIlTxDrvMcu1Vcu30x188.dat.motorSpeed 	= tNmTxDrvMcu1Vcu30x188.motorSpeed;
	uIlTxDrvMcu1Vcu30x188.dat.ksiVoltage 	= tNmTxDrvMcu1Vcu30x188.ksiVoltage;
	uIlTxDrvMcu1Vcu30x188.dat.motorTorq  	= tNmTxDrvMcu1Vcu30x188.motorTorq + 1000;
	uIlTxDrvMcu1Vcu30x188.dat.phaseCurrent= tNmTxDrvMcu1Vcu30x188.phaseCurrent;
}
void IL_FillSendMessage0x288(void)
{
	uIlTxDrvMcu1Vcu40x288.dat.faultCode 	= tNmTxDrvMcu1Vcu40x288.faultCode;
	uIlTxDrvMcu1Vcu40x288.dat.motorTemp 	= tNmTxDrvMcu1Vcu40x288.motorTemp + 40;
	uIlTxDrvMcu1Vcu40x288.dat.envTemp  	= tNmTxDrvMcu1Vcu40x288.envTemp + 40;
	uIlTxDrvMcu1Vcu40x288.dat.swVersion	= tNmTxDrvMcu1Vcu40x288.swVersion=DEBUG_VER;
	uIlTxDrvMcu1Vcu40x288.dat.inCurrent  	= tNmTxDrvMcu1Vcu40x288.inCurrent;
	uIlTxDrvMcu1Vcu40x288.dat.inVoltage	= tNmTxDrvMcu1Vcu40x288.inVoltage;
}
void IL_FillSendMessage0x4616401(void)
{
	uIlTxParaSend0x4616401.dat.paraAdd = tNmTx0x4616401.paraAdd;
	uIlTxParaSend0x4616401.dat.paraSta = tNmTx0x4616401.paraSta;
	uIlTxParaSend0x4616401.dat.paraVal = tNmTx0x4616401.paraVal;

	tNmTx0x4616401.msgUpd = 1;
}

void IL_FillSendMessage0x582(void)
{

}

void IL_FillSendMessage0x582_SET(uint8* CAN_data)
{
	uint8 i;
	for(i = 0;i<8;i++)
	{
		uIlTxDrvMcuINFOx582.byte[i] = CAN_data[i];
	}
	
	tNmTxDrvMcu1INFOx582.msgUpd = 1;
}

void IL_FillSendMessage0x583(void)
{

}

void IL_FillSendMessage0x583_SET(uint8* CAN_data)
{
	uint8 i;
	for(i = 0;i<8;i++)
	{
		uIlTxDrvMcuINFOx583.byte[i] = CAN_data[i];
	}
	tNmTxDrvMcu1INFOx583.msgUpd = 1;
}

void IL_Can1MessageRxISR(uint32 can_id, uint8 *pmsg)
{
	uint16 i,j;
	for(i=0;i<Can1RxMsgNum;i++)
	{
		if(can_id==Can1RxMsgList[i].msgID)
		{
			Can1RxMsgSta[i].msgUpd = 1;
			for(j=0;j<8;j++)
			{
				Can1RxMsgList[i].pmsg[j] = pmsg[j];
			}
			break;
		}
	}
}
void IL_Can1SendMessageProgress(void)
{
	uint16 i;
	for(i=0;i<Can1TxMsgNum;i++)
	{
		Can1TxMsgSta[i]+=5;
		if(Can1TxMsgList[i].sndTi==0)	//event frame
		{
			if(*(Can1TxMsgList[i].pmsgUpd)==1)
			{
				if(Can1TxMsgList[i].msgFun!=NULL)
				{
					Can1TxMsgList[i].msgFun();
				}
			}
		}
		else if(Can1TxMsgSta[i]>=Can1TxMsgList[i].sndTi)
		{
			Can1TxMsgSta[i] = Can1TxMsgList[i].sndTi - Can1TxMsgSta[i] ;
			if(Can1TxMsgList[i].msgFun!=NULL)
			{
				Can1TxMsgList[i].msgFun();
				*(Can1TxMsgList[i].pmsgUpd) = 1;
			}
		}
		else if(Can1TxMsgList[i].sndTi==0xFFFF)
		{

		}
	}
}
void IL_Can1SendMessageToBus(void)
{
	uint16 snd_sta;
	uint16 i;
	for(i=0;i<Can1TxMsgNum;i++)
	{
		if(*(Can1TxMsgList[i].pmsgUpd)==1)
		{
			snd_sta = CAN_Can1SendMessage(Can1TxMsgList[i].msgID,Can1TxMsgList[i].pmsg,8);
			if(snd_sta==1)
			{
				*(Can1TxMsgList[i].pmsgUpd) = 0;
				break;
			}
		}
	}
}
void IL_Can1ReceiveMessageProggress(void)
{
	uint16 i;
	for(i=0;i<Can1RxMsgNum;i++)
	{
		Can1RxMsgSta[i].msgCnt+=5;
		if(Can1RxMsgSta[i].msgUpd==1)
		{
			Can1RxMsgSta[i].msgCnt = 0;
			if(Can1RxMsgList[i].msgFun!=NULL)
			{
				Can1RxMsgList[i].msgFun(Can1RxMsgList[i].pmsg);
			}
			Can1RxMsgSta[i].msgUpd = 0;
		}
		else
		{
			if(Can1RxMsgSta[i].msgCnt>=Can1RxMsgList[i].maxTi)
			{
				if(Can1RxMsgList[i].msgFun!=NULL)
				{
					Can1RxMsgList[i].msgFun(NULL);
				}
			}
		}

	}
}
void IL_Can2MessageRxISR(uint32 can_id, uint8 *pmsg)
{
	uint16 i,j;
	for(i=0;i<Can2RxMsgNum;i++)
	{
		if(can_id==Can2RxMsgList[i].msgID)
		{
			Can2RxMsgSta[i].msgUpd = 1;
			for(j=0;j<8;j++)
			{
				Can2RxMsgList[i].pmsg[j] = pmsg[j];
			}
			break;
		}
	}
}

void IL_Can2SendMessageProgress(void)
{
	uint16 i;

	for(i=0;i<Can2TxMsgNum;i++)
	{
		Can2TxMsgSta[i]+=5;
		if(Can2TxMsgList[i].sndTi==0)	//event frame
		{
			if(*(Can2TxMsgList[i].pmsgUpd)==1)
			{
				if(Can2TxMsgList[i].msgFun!=NULL)
				{
					Can2TxMsgList[i].msgFun();
				}
			}
		}
		else if(Can2TxMsgSta[i]>=Can2TxMsgList[i].sndTi)	//cycle frame
		{
			Can2TxMsgSta[i] = Can2TxMsgList[i].sndTi - Can2TxMsgSta[i] ;
			if(Can2TxMsgList[i].msgFun!=NULL)
			{
				Can2TxMsgList[i].msgFun();
				*(Can2TxMsgList[i].pmsgUpd) = 1;
			}
		}
		else 
		{

		}
	}
}
void IL_Can2SendMessageToBus(void)
{
	uint16 snd_sta;
	uint16 i;
	for(i=0;i<Can2TxMsgNum;i++)
	{
		if(*(Can2TxMsgList[i].pmsgUpd)==1)
		{
			snd_sta = CAN_Can2SendMessage(Can2TxMsgList[i].msgID,Can2TxMsgList[i].pmsg,8);
			if(snd_sta==1)
			{
				*(Can2TxMsgList[i].pmsgUpd) = 0;
				break;
			}
		}
	}
}
void IL_Can2ReceiveMessageProggress(void)
{
	uint16 i;
	for(i=0;i<Can2RxMsgNum;i++)
	{
		Can2RxMsgSta[i].msgCnt+=5;
		if(Can2RxMsgSta[i].msgUpd==1)
		{
			Can2RxMsgSta[i].msgCnt = 0;
			if(Can2RxMsgList[i].msgFun!=NULL)
			{
				Can2RxMsgList[i].msgFun(Can2RxMsgList[i].pmsg);
			}
			Can2RxMsgSta[i].msgUpd = 0;
		}
		else
		{
			if(Can2RxMsgSta[i].msgCnt>=Can2RxMsgList[i].maxTi)
			{
				if(Can2RxMsgList[i].msgFun!=NULL)
				{
					Can2RxMsgList[i].msgFun(NULL);
				}
			}
		}

	}
}

void IL_InitTask(void)
{
	Can1TxMsgNum = sizeof(Can1TxMsgList) / sizeof(MSG_TX_LIST_T);
	Can1RxMsgNum = sizeof(Can1RxMsgList) / sizeof(MSG_RX_LIST_T);
	Can2TxMsgNum = sizeof(Can2TxMsgList) / sizeof(MSG_TX_LIST_T);
	Can2RxMsgNum = sizeof(Can2RxMsgList) / sizeof(MSG_RX_LIST_T);
}
void IL_MainTask(void)
{
	IL_Can1ReceiveMessageProggress();
	IL_Can2ReceiveMessageProggress();
	if(tUdsSrv.msgSendDisable==0)
	{
		IL_Can1SendMessageProgress();
		IL_Can2SendMessageProgress();
	}
}
