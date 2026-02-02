#include <string.h>
#include "def.h"
#include "tp.h"
#include "can.h"
#include "uds.h"
#include "il.h"
TP_DAT_T tTpRx,tTpTx;

void TP_SingleFrameAnalysis(TP_DAT_T *ptp,uint8 *pdat)
{
	uint16 i;
	ptp->datLen = *pdat&0x0F;
	pdat++;
	for(i=0;i<ptp->datLen;i++)
	{
		ptp->datBuf[i] = *pdat++;
	}
	ptp->frmTyp = FRAME_TYPE_SF;
	ptp->datCnt = i;
}
void TP_FirstFrameAnalysis(TP_DAT_T *ptp,uint8 *pdat)
{
	uint16 i;
	ptp->datLen = (*pdat&0x0F)<<8;
	pdat++;
	ptp->datLen += *pdat;
	pdat++;
	for(i=0;i<6;i++)
	{
		ptp->datBuf[i] = *pdat++;
	}
	ptp->frmTyp = FRAME_TYPE_FF;
	ptp->datCnt = 6;
	ptp->frmCnt = 1;
}

void TP_ConsecutiveFrameAnalysis(TP_DAT_T *ptp,uint8 *pdat)
{
	uint16 i;
	uint16 len = ptp->datLen;
	if(len>=7)
	{
		len = 7;
	}

	if((*pdat&0x0F)==ptp->frmCnt)
	{
		pdat++;
		for(i=0;i<len;i++)
		{
			ptp->datBuf[ptp->datCnt++] = *pdat++;
		}
		ptp->frmTyp = FRAME_TYPE_CF;
		ptp->frmCnt++;
		if(ptp->frmCnt>0x0F)
		{
			ptp->frmCnt = 0;
		}
	}
	else
	{

		ptp->frmTyp = FRAME_TYPE_ERR;
	}
}
void TP_FlowControlSend(TP_DAT_T *ptp)
{
	ptp->canID = UDS_RES_ADDR;
	ptp->datLen = 3;
	ptp->datBuf[0] = 0x30;
	ptp->datBuf[1] = 0x00;
	ptp->datBuf[2] = 2;	//STmin = 2;
	ptp->frmTyp = FRAME_TYPE_FC;
}

void TP_Can1ReceiveMessageHandler(uint32 id,uint8 *pdat)
{
	uint8 frm_typ;
	if((id==UDS_FUN_ADDR)||(id==UDS_PHY_ADDR))
	{
		tUdsSrv.drvNo = UDS_DRV_CAN1;
		frm_typ = pdat[0]>>4;

		if(frm_typ==FRAME_TYPE_SF)	//SFm
		{
			TP_SingleFrameAnalysis(&tTpRx,pdat);
			UDS_UdsServiceAnalysis(&tTpRx.datBuf[0],tTpRx.datLen,UDS_DRV_CAN1);
		}
		else if(FRAME_TYPE_FF==frm_typ)	//FF
		{
			TP_FirstFrameAnalysis(&tTpRx,pdat);
			TP_FlowControlSend(&tTpTx);
			TP_Can1TransmitData(tTpTx.datBuf,tTpTx.datLen);
		}
		else if(FRAME_TYPE_CF==frm_typ)	//CF
		{
			TP_ConsecutiveFrameAnalysis(&tTpRx,pdat);
			if(tTpRx.datCnt>=tTpRx.datLen)
			{
				UDS_UdsServiceAnalysis(&tTpRx.datBuf[0],tTpRx.datLen,UDS_DRV_CAN1);
			}
		}
		else if(FRAME_TYPE_FC==frm_typ)	//FC
		{
			//receive do nothing
		}
	}
	else
	{
		IL_Can1MessageRxISR(id,pdat);
	}
}

void TP_Can2ReceiveMessageHandler(uint32 id,uint8 *pdat)
{
	uint8 frm_typ;

	if((id==UDS_FUN_ADDR)||(id==UDS_PHY_ADDR))
	{
		tUdsSrv.drvNo = UDS_DRV_CAN2;
		frm_typ = pdat[0]>>4;
		if(frm_typ==FRAME_TYPE_SF)	//SF
		{
			TP_SingleFrameAnalysis(&tTpRx,pdat);
			UDS_UdsServiceAnalysis(&tTpRx.datBuf[0],tTpRx.datLen,UDS_DRV_CAN2);
		}
		else if(FRAME_TYPE_FF==frm_typ)	//FF
		{
			TP_FirstFrameAnalysis(&tTpRx,pdat);
			TP_FlowControlSend(&tTpTx);
			TP_Can2TransmitData(tTpTx.datBuf,tTpTx.datLen);
		}
		else if(FRAME_TYPE_CF==frm_typ)	//CF
		{
			TP_ConsecutiveFrameAnalysis(&tTpRx,pdat);
			if(tTpRx.datCnt>=tTpRx.datLen)
			{
				UDS_UdsServiceAnalysis(&tTpRx.datBuf[0],tTpRx.datLen,UDS_DRV_CAN2);
			}
		}
		else if(FRAME_TYPE_FC==frm_typ)	//FC
		{
			//receive do nothing
		}
	}
	else
	{
		IL_Can2MessageRxISR(id,pdat);
	}
}

void TP_Can1TransmitData(uint8 *pdat,uint16 len)
{
	uint8 buff[8];
	uint16 snd_len=8;
	if(len<8)
	{
		if(pdat[0]!=0x30)
		{	
			buff[0]= len;
			snd_len = len + 1;
			memcpy(&buff[1],pdat,len);
			CAN_Can1SendMessage(UDS_RES_ADDR,buff,snd_len);
		}
		else
		{
			CAN_Can1SendMessage(UDS_RES_ADDR,pdat,len);
		}
		
	}
	else
	{
	}
}
void TP_Can2TransmitData(uint8 *pdat,uint16 len)
{
	uint8 buff[8];
	uint16 snd_len=8;
	if(len<8)
	{
		if(pdat[0]!=0x30)
		{	
			buff[0]= len;
			snd_len = len + 1;
			memcpy(&buff[1],pdat,len);
			CAN_Can2SendMessage(UDS_RES_ADDR,buff,snd_len);
		}
		else
		{
			CAN_Can2SendMessage(UDS_RES_ADDR,pdat,len);
		}
		
	}
	else
	{

	}
}
void TP_InitTask(void)
{
	  IL_InitTask();
}
void TP_MainTask(void)
{
	IL_Can1SendMessageToBus();
	IL_Can1SendMessageToBus();
}
