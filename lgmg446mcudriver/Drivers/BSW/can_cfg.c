/*
 * can_cfg.c
 *
 *  Created on: Feb 14, 2025
 *      Author: Administrator
 */
#include "f_mcuctrl.h"
#include "f_carProtocolJR-SYNC.h"
#include "stm32f4xx_hal.h"
#include "string.h"

#define  TX_MSG_NUM	6
#define  RX_MSG_NUM	8


#define MSGRX       1
#define MSGTX       0



extern CAN_HandleTypeDef hcan1;
extern  int    CanBusssoffCnt;

//CAN_HandleTypeDef     CanHandle;
//CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
//uint8_t               RxData[8];
uint32_t              TxMailbox;

uint8_t TxMsgData[8];	
uint8_t RxMsgData[8];	


typedef struct
{	
    Uint32	CanID;
    Uint16	CanPeriod;
    //Uint16	CanRxTx;
	//Uint16	CanMode;
    void (*pfMsgProcess)(void);
}CanMsgDetail;
typedef struct
{	
	CAN_TxHeaderTypeDef   TxHeader;
	uint8_t 			  TxData[8];
	Uint16				  CanTxFlag;
	Uint16				  PeriodCnt;
	
}CanTxDetail;
typedef struct
{	
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t 			  RxData[8];
	Uint16				  CanRxFlag;
	//Uint16				  PeriodCnt;
	
}CanRxDetail;

CanTxDetail	Can1TxDetail[TX_MSG_NUM];
CanRxDetail Can1RxDetail[RX_MSG_NUM];

void CarMsgPackage(void)
{


}
void CarMsgUnPackage(void)
{


}

const CanMsgDetail Can1TxMsgDetailTbl[TX_MSG_NUM] =
        /* CanID        Period      RxTx         */
{
	{   0x388		,   10  ,    CarJRFrameMETER1InfoUpdate },
	{   0x3F1		,   10  ,    CarJRFrameTBOXInfoUpdate },
	{   0x542		,   10  ,    CarJRFrameMETER2InfoUpdate },
	{   0x0CFF10EF	,   40  ,    CarJRDriverInfo1Update },
	{   0x0CFF20EF	,   40  ,    CarJRDriverInfo2Update },
	{   0x006B6806	,   4  ,    CarJRDriverTboxUpdate },	
//	{   0x0CFF30EF	,   2  ,    CarJRDriverInfo3Update },
//	{   0x0CFF40EF	,   2  ,    CarJRDriverInfo4Update },


};

const CanMsgDetail Can1RxMsgDetailTbl[RX_MSG_NUM] =
        /* CanID        Period      RxTx         */
{
	{   0x3C0		,   1  ,    CarJRRcvFrameMETERDeal },

	{   0x5A0		,   1  ,    CarJRRcvFrameBMS0Deal },
	{   0x3F2		,   1  ,    CarJRRcvFramePump2TboxDeal},
	
	{   0x0CFF01EF	,   1  ,    CarJRRcvFramePumpInfo1Deal},
	
	{   0x0CFF02EF	,   1  ,    CarJRRcvFramePumpInfo2Deal},

	{   0x602	,   1  ,    CarJRRcvFrameMETER2Deal},

	{   0x6B68A1	,   1  ,    CarJRRcvFrameTboxA1Deal},

	{   0x6B68A2	,   1  ,    CarJRRcvFrameTboxA2Deal},	
	
};


void CAN1_Rx_Msg_Init(void)
{
	CAN_FilterTypeDef  sFilterConfig;
	int i = 0;
	uint32_t CanID = 0;

//	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN \
//										| CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	memset(RxMsgData, 0x00, sizeof(RxMsgData));
	memset(Can1RxDetail, 0x00, sizeof(Can1RxDetail));
	
	/*##-2- Configure the CAN Filter ###########################################*/

	for(i = 0; i < RX_MSG_NUM; i++)
	{
		if(IS_CAN_STDID(Can1RxMsgDetailTbl[i].CanID))
		{
			sFilterConfig.FilterIdHigh = Can1RxMsgDetailTbl[i].CanID << 5;
			//sFilterConfig.FilterIdHigh = 0x0000;
			sFilterConfig.FilterIdLow = 0x0000;
			sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		}
		else
		{
			CanID = ((Can1RxMsgDetailTbl[i].CanID << CAN_TI0R_EXID_Pos) | CAN_ID_EXT | CAN_RTR_DATA);
			sFilterConfig.FilterIdHigh = ((0xFFFF0000 & CanID) >> 16);
			sFilterConfig.FilterIdLow = 0x0000FFFF & CanID;
			sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		}
		sFilterConfig.FilterBank = i;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterMaskIdHigh = 0xFFFF;
		sFilterConfig.FilterMaskIdLow = 0xFFFF;


		sFilterConfig.FilterActivation = ENABLE;
		sFilterConfig.SlaveStartFilterBank = RX_MSG_NUM;

		
		if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
		{
		  /* Filter configuration Error */
		  Error_Handler();
		}
	}
}

void Can1_Rx_Process(void)
{
	int i = 0, j = 0;
	for(i = 0; i < RX_MSG_NUM; i++)
	{
		if(Can1RxDetail[i].CanRxFlag == 1)
		{
			for(j = 0; j < RX_MSG_NUM; j++)
			{
				if(((Can1RxDetail[i].RxHeader.IDE == CAN_ID_STD) && (Can1RxMsgDetailTbl[j].CanID == Can1RxDetail[i].RxHeader.StdId))
					|| ((Can1RxDetail[i].RxHeader.IDE == CAN_ID_EXT) && (Can1RxMsgDetailTbl[j].CanID == Can1RxDetail[i].RxHeader.ExtId)))
					{
						RxMsgData[0] = Can1RxDetail[i].RxData[0];
						RxMsgData[1] = Can1RxDetail[i].RxData[1];
						RxMsgData[2] = Can1RxDetail[i].RxData[2];
						RxMsgData[3] = Can1RxDetail[i].RxData[3];
						RxMsgData[4] = Can1RxDetail[i].RxData[4];
						RxMsgData[5] = Can1RxDetail[i].RxData[5];
						RxMsgData[6] = Can1RxDetail[i].RxData[6];
						RxMsgData[7] = Can1RxDetail[i].RxData[7];
						memset(Can1RxDetail[i].RxData, 0x00, sizeof(Can1RxDetail[i].RxData));
						Can1RxMsgDetailTbl[j].pfMsgProcess();
					}

			}

			Can1RxDetail[i].CanRxFlag = 0;
			
			break;
		}
	}



}

void CAN1_Tx_Msg_Init(void)
{
	int i = 0;
	memset(TxMsgData, 0x00, sizeof(TxMsgData));
	memset(Can1TxDetail, 0x00, sizeof(Can1TxDetail));
	for(i = 0; i < TX_MSG_NUM; i++)
	{
		Can1TxDetail[i].PeriodCnt = i*2;
	}
	//HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
	  /* Start Error */
	  Error_Handler();
	}

}


void Can1_Tx_Process(void)
{
	int i = 0;
	for(i = 0; i < TX_MSG_NUM; i++)
	{
		Can1TxDetail[i].PeriodCnt++;
		if(Can1TxDetail[i].PeriodCnt >= Can1TxMsgDetailTbl[i].CanPeriod)
		{
			Can1TxDetail[i].CanTxFlag = 1;
			Can1TxDetail[i].PeriodCnt = 0;
		}
	}
}

void Can1_Tx_Message(void)
{
	/*##-4- Start the Transmission process #####################################*/

	
	int i = 0;
	
	for(i = 0; i < TX_MSG_NUM; i++)
	{
		if(Can1TxDetail[i].CanTxFlag == 1)
		{
			if(IS_CAN_STDID(Can1TxMsgDetailTbl[i].CanID))
			{
				Can1TxDetail[i].TxHeader.StdId = Can1TxMsgDetailTbl[i].CanID;
				Can1TxDetail[i].TxHeader.RTR = CAN_RTR_DATA;
				Can1TxDetail[i].TxHeader.IDE = CAN_ID_STD;
				Can1TxDetail[i].TxHeader.DLC = 8;
				Can1TxDetail[i].TxHeader.TransmitGlobalTime = DISABLE;
			}
			else
			{
				Can1TxDetail[i].TxHeader.ExtId = Can1TxMsgDetailTbl[i].CanID;
				Can1TxDetail[i].TxHeader.RTR = CAN_RTR_DATA;
				Can1TxDetail[i].TxHeader.IDE = CAN_ID_EXT;
				Can1TxDetail[i].TxHeader.DLC = 8;
				Can1TxDetail[i].TxHeader.TransmitGlobalTime = DISABLE;
			}
			
			Can1TxMsgDetailTbl[i].pfMsgProcess();
			
			Can1TxDetail[i].TxData[0] = TxMsgData[0];
			Can1TxDetail[i].TxData[1] = TxMsgData[1];
			Can1TxDetail[i].TxData[2] = TxMsgData[2];
			Can1TxDetail[i].TxData[3] = TxMsgData[3];
			Can1TxDetail[i].TxData[4] = TxMsgData[4];
			Can1TxDetail[i].TxData[5] = TxMsgData[5];
			Can1TxDetail[i].TxData[6] = TxMsgData[6];
			Can1TxDetail[i].TxData[7] = TxMsgData[7];
	
			/* Request transmission */
			if(HAL_CAN_AddTxMessage(&hcan1, &(Can1TxDetail[i].TxHeader), Can1TxDetail[i].TxData, &TxMailbox) != HAL_OK)
			{
			  /* Transmission request Error */
			  Error_Handler();
			}
			else
			{
				Can1TxDetail[i].CanTxFlag = 0;
				break;
			}

		}
	}
}


void CAN1_Fifo0_Full_IRQHandler(void)
{
	int i = 0, j = 0;
	int readCnt = 0;
	uint8_t RxData[8];
	
	CAN_RxHeaderTypeDef   RxHeader;
	
	memset(RxData, 0x00, 8);
	
	readCnt = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);

	if(readCnt > 0)
	{
		CanBusssoffCnt = 0;
		for(i = 0; i < readCnt; i++)
		{
			if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
			{
				for(j = 0; j < RX_MSG_NUM; j++)
				{
					if(Can1RxDetail[j].CanRxFlag == 0)
					{
						memset(Can1RxDetail[j].RxData, 0x00, sizeof(Can1RxDetail[j].RxData));

						Can1RxDetail[j].RxHeader = RxHeader;
						
						Can1RxDetail[j].RxData[0] = RxData[0];
						Can1RxDetail[j].RxData[1] = RxData[1];
						Can1RxDetail[j].RxData[2] = RxData[2];
						Can1RxDetail[j].RxData[3] = RxData[3];
						Can1RxDetail[j].RxData[4] = RxData[4];
						Can1RxDetail[j].RxData[5] = RxData[5];
						Can1RxDetail[j].RxData[6] = RxData[6];
						Can1RxDetail[j].RxData[7] = RxData[7];
						
						Can1RxDetail[j].CanRxFlag = 1;
						
						break;
					}
				}
			}
		}
	}
}


void CAN1_Fifo1_Full_IRQHandler(void)
{
	int i = 0, j = 0;
	int readCnt = 0;
	uint8_t RxData[8];
	
	CAN_RxHeaderTypeDef   RxHeader;
	
	memset(RxData, 0x00, 8);
	
	readCnt = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO1);

	if(readCnt > 0)
	{
		for(i = 0; i < readCnt; i++)
		{
			if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
			{
				for(j = 0; j < RX_MSG_NUM; j++)
				{
					if(Can1RxDetail[j].CanRxFlag == 0)
					{
						memset(Can1RxDetail[j].RxData, 0x00, sizeof(Can1RxDetail[j].RxData));

						Can1RxDetail[j].RxHeader = RxHeader;
						
						Can1RxDetail[j].RxData[0] = RxData[0];
						Can1RxDetail[j].RxData[1] = RxData[1];
						Can1RxDetail[j].RxData[2] = RxData[2];
						Can1RxDetail[j].RxData[3] = RxData[3];
						Can1RxDetail[j].RxData[4] = RxData[4];
						Can1RxDetail[j].RxData[5] = RxData[5];
						Can1RxDetail[j].RxData[6] = RxData[6];
						Can1RxDetail[j].RxData[7] = RxData[7];
						
						Can1RxDetail[j].CanRxFlag = 1;
						
						break;
					}
				}
			}
		}
	}
	




}


