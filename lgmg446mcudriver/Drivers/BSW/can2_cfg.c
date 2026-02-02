/*
 * can2_cfg.c
 *
 *  Created on: Feb 26, 2025
 *      Author: Administrator
 */
#include "f_public.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "can_typedef.h"
#include "can2_cfg.h"
#include "main.h"
#include "tp.h"
#define  CAN2_TX_MSG_NUM	1
#define  CAN2_RX_MSG_NUM	5



#define UINT32_BYTE0( Data )    (   (Uint16)((Uint32)Data & 0xff)    )
#define UINT32_BYTE1( Data )    (   (Uint16)( ((Uint32)Data >> 8)& 0xff))
#define UINT32_BYTE2( Data )    (   (Uint16)( ((Uint32)Data >> 16)& 0xff))
#define UINT32_BYTE3( Data )    (   (Uint16)( ((Uint32)Data >> 24)& 0xff))
#define Uint32_FROM_4BYTE( Byte0, Byte1, Byte2, Byte3 )     ((Uint32)((Uint32)(Byte0) + ((Uint32)(Byte1) << 8) + ((Uint32)(Byte2) << 16) + ((Uint32)(Byte3) << 24)))
//#define INT32_FROM_4BYTE( Byte0, Byte1, Byte2, Byte3 )      ((int32)((Uint32)(Byte0) + ((Uint32)(Byte1) << 8) + ((Uint32)(Byte2) << 16) + ((Uint32)(Byte3) << 24)))

extern CAN_HandleTypeDef hcan2;

//CAN_HandleTypeDef     CanHandle;
//CAN_TxHeaderTypeDef   TxHeader;
//CAN_RxHeaderTypeDef   Can2RxHeader;
//uint8_t               RxData[8];
uint32_t              Can2TxMailbox;

uint8_t Can2TxMsgData[8];	
uint8_t Can2RxMsgData[8];	




CanTxDetail	Can2TxDetail[CAN2_TX_MSG_NUM];
CanRxDetail Can2RxDetail[CAN2_RX_MSG_NUM];

//const Uint8 CanACKData[8] = [0x3A, 0x02, 0x00, 0xC4, 0x00, 0x00, 0x00, 0x00];




void MsgReadPrmRegister(void)
{
    Uint16 rc;
    Uint16 CanAddr;
    int32 UsrData;
    uint8_t data_send[8];

    CanAddr = ((Uint16)((Uint16)(Can2RxMsgData[0]) + ((Uint16)(Can2RxMsgData[1]) << 8)));
    if((CanAddr >> 12 ) == 3)
    {
         rc = GetMonitorValue(CanAddr,&UsrData);
    }
    else
    {
         rc = GetPrmValue(CanAddr,&UsrData);
    }
    data_send[0] = CanAddr&0xFF;
    data_send[1] = CanAddr>>8;
    data_send[2] = rc&0xFF;
    data_send[3] = rc>>8;
    data_send[4] = UINT32_BYTE0(UsrData);
    data_send[5] = UINT32_BYTE1(UsrData);
    data_send[6] = UINT32_BYTE2(UsrData);
    data_send[7] = UINT32_BYTE3(UsrData);
    Can2_Tx_Message(0x4316401,data_send);
}

void MsgWritePrmRegister(void)
{
    Uint16 rc;
    Uint16 CanAddr;
    int32 UsrData;
    uint8_t data_send[8];
    CanAddr = ((Uint16)((Uint16)(Can2RxMsgData[0]) + ((Uint16)(Can2RxMsgData[1]) << 8)));
    UsrData =  ((Uint32)((Uint16)(Can2RxMsgData[4]) + ((Uint16)(Can2RxMsgData[5]) << 8) + ((Uint32)(Can2RxMsgData[6]) << 16) + ((Uint32)(Can2RxMsgData[7]) << 24)));

    rc = SetPrmValue(CanAddr, UsrData);
    data_send[0] = CanAddr&0xFF;
    data_send[1] = CanAddr>>8;
    data_send[2] = rc&0xFF;
    data_send[3] = rc>>8;
    data_send[4] = UINT32_BYTE0(UsrData);
    data_send[5] = UINT32_BYTE1(UsrData);
    data_send[6] = UINT32_BYTE2(UsrData);
    data_send[7] = UINT32_BYTE3(UsrData);
    Can2_Tx_Message(0x4616401,data_send);

}

void MsgWriteEepromCmd(void)
{
    Uint16 rc=0;
    Uint16 CanAddr;
    int32 UsrData;
    uint8_t data_send[8];
    int i;
    CanAddr = ((Uint16)((Uint16)(Can2RxMsgData[0]) + ((Uint16)(Can2RxMsgData[1]) << 8)));
    UsrData =  ((Uint32)((Uint16)(Can2RxMsgData[4]) + ((Uint16)(Can2RxMsgData[5]) << 8) + ((Uint32)(Can2RxMsgData[6]) << 16) + ((Uint32)(Can2RxMsgData[7]) << 24)));

    if(CanAddr == 0x0100)
    {
    	if(g_StartFlag == 1)
    	{
    		rc = 1;
    	}
    	else
    	{
            for(i = 0; i < TOTAL_MONITOR_NUM; i++)
			{
				I2CWriteInt32WaitMode(0x10+i,parameter[i]);
			}
            rc = 0;
    	}
    }
    data_send[0] = CanAddr&0xFF;
    data_send[1] = CanAddr>>8;
    data_send[2] = rc&0xFF;
    data_send[3] = rc>>8;
    data_send[4] = UINT32_BYTE0(UsrData);
    data_send[5] = UINT32_BYTE1(UsrData);
    data_send[6] = UINT32_BYTE2(UsrData);
    data_send[7] = UINT32_BYTE3(UsrData);
    Can2_Tx_Message(0x4516401,data_send);


}

uint16_t g_DriveUpgradeFlag = 0;

uint8 CAN2_GetUpgradeStat(void)
{
	uint8 ret=0;
	if (fabs(gRotorSpeed.SpeedApplyFilter) <= 2)
	{
		g_RunStatus = STATUS_UPDATE_FIRMWARE;
		g_DriveUpgradeFlag = 1;
		ret = 1;
	}
	return ret;
}
void CarDriveUpgrade_UnPackage(void)
{
#if 0
	if (fabs(gRotorSpeed.SpeedApplyFilter) <= 2)
	{
		g_RunStatus = STATUS_UPDATE_FIRMWARE;
		g_DriveUpgradeFlag = 1;
	}
#endif
}

void CarPumpUpgrade_UnPackage(void)
{
#if 0
	if (fabs(gRotorSpeed.SpeedApplyFilter) <= 2)
	{
		g_RunStatus = STATUS_UPDATE_FIRMWARE;
	}
#endif
}

const CanMsgDetail Can2RxMsgDetailTbl[CAN2_RX_MSG_NUM] =
        /* CanID        Period      RxTx         */
{
	{   0x7DF			,   1  ,    CarDriveUpgrade_UnPackage },
	{   0x700			,   1  ,    CarPumpUpgrade_UnPackage },
	{	0x04300164		,	1	,	MsgReadPrmRegister		},
	{	0x04600164		,	1	,	MsgWritePrmRegister		},
	{	0x04500164		,	1	,	MsgWriteEepromCmd		},

};


void CAN2_Rx_Msg_Init(void)
{
	CAN_FilterTypeDef  sFilterConfig;
	int i = 0;
	uint32_t CanID = 0;

	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

	
	memset(Can2RxMsgData, 0x00, sizeof(Can2RxMsgData));
	memset(Can2RxDetail, 0x00, sizeof(Can2RxDetail));
	
	/*##-2- Configure the CAN Filter ###########################################*/

	for(i = 0; i < CAN2_RX_MSG_NUM; i++)
	{
		if(IS_CAN_STDID(Can2RxMsgDetailTbl[i].CanID))
		{
			sFilterConfig.FilterIdHigh = Can2RxMsgDetailTbl[i].CanID << 5;
			sFilterConfig.FilterIdLow = 0x0000;
		}
		else
		{
			CanID = ((Can2RxMsgDetailTbl[i].CanID << CAN_TI0R_EXID_Pos) | CAN_ID_EXT | CAN_RTR_DATA);
			sFilterConfig.FilterIdHigh = ((0xFFFF0000 & CanID) >> 16);
			sFilterConfig.FilterIdLow = 0x0000FFFF & CanID;
		}
		
		sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
		sFilterConfig.FilterBank = i+11;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterMaskIdHigh = 0xFFFF;
		sFilterConfig.FilterMaskIdLow = 0xFFFF;


		sFilterConfig.FilterActivation = ENABLE;
//		sFilterConfig.SlaveStartFilterBank = CAN2_RX_MSG_NUM;

		
		if(HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
		{
		  /* Filter configuration Error */
		  Error_Handler();
		}
	}
}

void Can2_Rx_Process(void)
{
	int i = 0, j = 0;
	for(i = 0; i < CAN2_RX_MSG_NUM; i++)
	{
		if(Can2RxDetail[i].CanRxFlag == 1)
		{
			for(j = 0; j < CAN2_RX_MSG_NUM; j++)
			{
				if(((Can2RxDetail[i].RxHeader.IDE == CAN_ID_STD) && (Can2RxMsgDetailTbl[j].CanID == Can2RxDetail[i].RxHeader.StdId))
					|| ((Can2RxDetail[i].RxHeader.IDE == CAN_ID_EXT) && (Can2RxMsgDetailTbl[j].CanID == Can2RxDetail[i].RxHeader.ExtId)))
					{
						Can2RxMsgData[0] = Can2RxDetail[i].RxData[0];
						Can2RxMsgData[1] = Can2RxDetail[i].RxData[1];
						Can2RxMsgData[2] = Can2RxDetail[i].RxData[2];
						Can2RxMsgData[3] = Can2RxDetail[i].RxData[3];
						Can2RxMsgData[4] = Can2RxDetail[i].RxData[4];
						Can2RxMsgData[5] = Can2RxDetail[i].RxData[5];
						Can2RxMsgData[6] = Can2RxDetail[i].RxData[6];
						Can2RxMsgData[7] = Can2RxDetail[i].RxData[7];
						memset(Can2RxDetail[i].RxData, 0x00, sizeof(Can2RxDetail[i].RxData));
						Can2RxMsgDetailTbl[j].pfMsgProcess();
					}

			}

			Can2RxDetail[i].CanRxFlag = 0;
			
			break;
		}
	}



}

void CAN2_Tx_Msg_Init(void)
{
	memset(Can2TxMsgData, 0x00, sizeof(Can2TxMsgData));
	memset(Can2TxDetail, 0x00, sizeof(Can2TxDetail));
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY);
	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(&hcan2) != HAL_OK)
	{
	  /* Start Error */
	  Error_Handler();
	}

}



void Can2_Tx_Message(uint32_t CanID, Uint8 *TxData)
{
		if(IS_CAN_STDID(CanID))
		{
			Can2TxDetail[0].TxHeader.StdId = CanID;
			Can2TxDetail[0].TxHeader.RTR = CAN_RTR_DATA;
			Can2TxDetail[0].TxHeader.IDE = CAN_ID_STD;
			Can2TxDetail[0].TxHeader.DLC = 8;
			Can2TxDetail[0].TxHeader.TransmitGlobalTime = DISABLE;
		}
		else
		{
			Can2TxDetail[0].TxHeader.ExtId = CanID;
			Can2TxDetail[0].TxHeader.RTR = CAN_RTR_DATA;
			Can2TxDetail[0].TxHeader.IDE = CAN_ID_EXT;
			Can2TxDetail[0].TxHeader.DLC = 8;
			Can2TxDetail[0].TxHeader.TransmitGlobalTime = DISABLE;
		}
				
		Can2TxDetail[0].TxData[0] = TxData[0];
		Can2TxDetail[0].TxData[1] = TxData[1];
		Can2TxDetail[0].TxData[2] = TxData[2];
		Can2TxDetail[0].TxData[3] = TxData[3];
		Can2TxDetail[0].TxData[4] = TxData[4];
		Can2TxDetail[0].TxData[5] = TxData[5];
		Can2TxDetail[0].TxData[6] = TxData[6];
		Can2TxDetail[0].TxData[7] = TxData[7];

		/* Request transmission */
		if(HAL_CAN_AddTxMessage(&hcan2, &(Can2TxDetail[0].TxHeader), Can2TxDetail[0].TxData, &Can2TxMailbox) != HAL_OK)
		{
		  /* Transmission request Error */
//		  Error_Handler();
		}
		else
		{

		}

}


void CAN2_Fifo0_Full_IRQHandler(void)
{
	int i = 0, j = 0;
	int readCnt = 0;
	uint8_t RxData[8];
	
	CAN_RxHeaderTypeDef   RxHeader;
	
	memset(RxData, 0x00, 8);
	
	readCnt = HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0);

	if(readCnt > 0)
	{
		for(i = 0; i < readCnt; i++)
		{
			if(HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
			{
				for(j = 0; j < CAN2_RX_MSG_NUM; j++)
				{
					if(Can2RxDetail[j].CanRxFlag == 0)
					{
						memset(Can2RxDetail[j].RxData, 0x00, sizeof(Can2RxDetail[j].RxData));

						Can2RxDetail[j].RxHeader = RxHeader;
						
						Can2RxDetail[j].RxData[0] = RxData[0];
						Can2RxDetail[j].RxData[1] = RxData[1];
						Can2RxDetail[j].RxData[2] = RxData[2];
						Can2RxDetail[j].RxData[3] = RxData[3];
						Can2RxDetail[j].RxData[4] = RxData[4];
						Can2RxDetail[j].RxData[5] = RxData[5];
						Can2RxDetail[j].RxData[6] = RxData[6];
						Can2RxDetail[j].RxData[7] = RxData[7];
						
						Can2RxDetail[j].CanRxFlag = 1;
						
						break;
					}
				}
			}
		}
	}
}


void CAN2_Fifo1_Full_IRQHandler(void)
{
	int i = 0, j = 0;
	int readCnt = 0;
	uint8_t RxData[8];
	
	CAN_RxHeaderTypeDef   RxHeader;
	
	memset(RxData, 0x00, 8);
	
	readCnt = HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO1);

	if(readCnt > 0)
	{
		for(i = 0; i < readCnt; i++)
		{
			if(HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
			{
				TP_Can2ReceiveMessageHandler(RxHeader.StdId, RxData);
				for(j = 0; j < CAN2_RX_MSG_NUM; j++)
				{
					if(Can2RxDetail[j].CanRxFlag == 0)
					{
						memset(Can2RxDetail[j].RxData, 0x00, sizeof(Can2RxDetail[j].RxData));

						Can2RxDetail[j].RxHeader = RxHeader;
						
						Can2RxDetail[j].RxData[0] = RxData[0];
						Can2RxDetail[j].RxData[1] = RxData[1];
						Can2RxDetail[j].RxData[2] = RxData[2];
						Can2RxDetail[j].RxData[3] = RxData[3];
						Can2RxDetail[j].RxData[4] = RxData[4];
						Can2RxDetail[j].RxData[5] = RxData[5];
						Can2RxDetail[j].RxData[6] = RxData[6];
						Can2RxDetail[j].RxData[7] = RxData[7];
						
						Can2RxDetail[j].CanRxFlag = 1;
						
						break;
					}
				}
			}
		}
	}
	




}



