/*
 * can_typedef.h
 *
 *  Created on: Feb 26, 2025
 *      Author: Administrator
 */

#ifndef SRC_CAN_TYPEDEF_H_
#define SRC_CAN_TYPEDEF_H_

typedef struct
{	
    uint32_t	CanID;
    uint16_t	CanPeriod;
    //uint16_t	CanRxTx;
	//uint16_t	CanMode;
    void (*pfMsgProcess)(void);
}CanMsgDetail;
typedef struct
{	
	CAN_TxHeaderTypeDef   TxHeader;
	uint8_t 			  TxData[8];
	uint16_t			  CanTxFlag;
	uint16_t			  PeriodCnt;
	
}CanTxDetail;
typedef struct
{	
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t 			  RxData[8];
	uint16_t			  CanRxFlag;
	//uint16_t				  PeriodCnt;
	
}CanRxDetail;


#endif /* SRC_CAN_TYPEDEF_H_ */
