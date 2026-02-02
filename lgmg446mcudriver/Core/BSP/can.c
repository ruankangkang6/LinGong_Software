#include "def.h"
#include "string.h"
#include "can.h"
#include "main.h"
#include "tp.h"
#include "main.h"
#include "can.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
CAN_RxHeaderTypeDef Can1RxFifo0,Can2RxFifo1;
uint8 Can1RxBuff0[8],Can2RxBuff1[8];

uint8 CAN_TransmitMessage(CAN_HandleTypeDef *hcan,uint32 can_id,const uint8 *pbuf,uint32 len)
{
	uint8 ret = 1;
	uint32 tx_mailbox = 0;
	uint32 offset = 0;
	CAN_TxHeaderTypeDef hdr;

	if(can_id<0x800)
	{
		hdr.IDE = CAN_ID_STD;
		hdr.StdId = can_id;
	}
	else
	{
		hdr.IDE = CAN_ID_EXT;
		hdr.ExtId = can_id;
	}
	hdr.RTR = CAN_RTR_DATA;
	hdr.TransmitGlobalTime = DISABLE;

	while(len!=0)
	{
		hdr.DLC = len > 8 ? 8 : len;
		if(HAL_CAN_AddTxMessage(hcan, &hdr, ((uint8 *)pbuf + offset), &tx_mailbox) == HAL_OK)
		{
			offset += hdr.DLC;
			len -= hdr.DLC;
		}
		else
		{
			ret = 0;
			break;
		}
	}
	return ret;
}

uint8 CAN_Can1SendMessage(const uint32 can_id,u8 *pbuf,u8 len)
{
	return CAN_TransmitMessage(&hcan1,can_id,pbuf,len);
}

uint8 CAN_Can2SendMessage(const uint32 can_id,u8 *pbuf,u8 len)
{
	return CAN_TransmitMessage(&hcan2,can_id,pbuf,len);
}
void CAN_Can1ReadFifo0Message(void)
{
	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Can1RxFifo0,Can1RxBuff0) == HAL_OK)
	{
		if(Can1RxFifo0.IDE==0)
		{
			TP_Can1ReceiveMessageHandler(Can1RxFifo0.StdId, Can1RxBuff0);
		}
		else
		{
			TP_Can1ReceiveMessageHandler(Can1RxFifo0.ExtId, Can1RxBuff0);
		}
		HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);	//enable fifo0 rx again
	}
}
void CAN_Can2ReadFifo1Message(void)
{
	if(HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &Can2RxFifo1, Can2RxBuff1) == HAL_OK)
	{
		if(Can2RxFifo1.IDE==0)
		{
			TP_Can2ReceiveMessageHandler(Can2RxFifo1.StdId, Can2RxBuff1);
		}
		else
		{
			TP_Can2ReceiveMessageHandler(Can2RxFifo1.ExtId, Can2RxBuff1);
		}
		HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
	}
}


