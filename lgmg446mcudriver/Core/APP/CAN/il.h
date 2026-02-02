#ifndef __IL_H__
#define __IL_H__
#include "def.h"

typedef struct
{
	uint8 msgUpd;
	uint16 msgCnt;
}MSG_RX_STATE_T;

typedef struct
{
	uint32 msgID;
	uint16 maxTi;
	uint8  *pmsg;
	void   (*msgFun)(uint8 *pmsg);

}MSG_RX_LIST_T;

typedef struct
{
	uint32 msgID;
	uint16 sndTi;
	uint8  *pmsg;
	void   (*msgFun)(void);
	uint8  *pmsgUpd;
}MSG_TX_LIST_T;


/*===============CAN 1 Rx Message===============*/
typedef struct
{
	uint8 res0_0:1;
	uint8 mainNegRlySta:1;
}IL_BMS_INFO_0x53_T;
typedef union
{
	uint8 byte[8];
	IL_BMS_INFO_0x53_T dat;
}IL_BMS_INFO_0x53_U;
extern IL_BMS_INFO_0x53_U uIlBmsInfo0x53;

typedef struct
{
	uint8 slopeAntiEnable	:1;
	uint8 slopeAntiFlag2	:1;
	uint8 brakeSignal		:1;
	uint8 slopeSigFlag		:1;
	uint8 res0_4			:4;
	uint8 res1;
	uint16 bmsDcCurrent;
}IL_BRAKE_INFO_0x1CA_T;
typedef union
{
	uint8 byte[8];
	IL_BRAKE_INFO_0x1CA_T dat;
}IL_BRAKE_INFO_0x1CA_U;
extern IL_BRAKE_INFO_0x1CA_U uIlRxBrakeInfo0x1CA;
typedef struct
{
	uint16 reqSpd;
	uint8  enable			:1;
	uint8  contactorEnable	:1;
	uint8  res2_2	:1;
	uint8  direction		:2;
	uint8  res2_5			:3;
	uint8  res3;
	uint8  diffSwitch		:1;
}IL_VCU_DRV_MCU_0x208_T;
typedef union
{
	uint8 byte[8];
	IL_VCU_DRV_MCU_0x208_T dat;
}IL_VCU_DRV_MCU_0x208_U;
extern IL_VCU_DRV_MCU_0x208_U uIlRxVcuDrvMcu0x208;

typedef union
{
	uint8 byte[8];
}IL_VCU_DRV_MCU_0x602_U;
extern IL_VCU_DRV_MCU_0x602_U uIlRxVcuDrvMcu0x602;

typedef union
{
	uint8 byte[8];
}IL_VCU_DRV_MCU_0x603_U;
extern IL_VCU_DRV_MCU_0x603_U uIlRxVcuDrvMcu0x603;
/*===============CAN 1 Tx Message===============*/
typedef struct
{
	uint16 faultCode;
	uint16 spdReq;
	uint16 spdReal;
	uint16 spdLoop;
}IL_DRV_MCU1_VCU1_0x88_T;
typedef union
{
	uint8 byte[8];
	IL_DRV_MCU1_VCU1_0x88_T dat;
}IL_DRV_MCU1_VCU1_0x88_U;
extern IL_DRV_MCU1_VCU1_0x88_U uTxDrvMcu1Vcu10x88;

typedef struct
{
	uint16 faultCode;
	uint16 piRev;
	uint8  brakeSignalFB	:1;
	uint8  slopeAntiFlag2FB:1;
	uint8  slopeTriggerFB	:1;
	uint8  piLock			:1;
	uint8  clearFlag		:1;
	uint8  brakeBuild		:1;
	uint8  flagEnableOff	:1;
	uint8  res4_0			:1;
	uint8  res5;
	uint16 mosTemp;
}IL_DRV_MCU1_VCU2_0x89_T;
typedef union
{
	uint8 byte[8];
	IL_DRV_MCU1_VCU2_0x89_T dat;
}IL_DRV_MCU1_VCU2_0x89_U;
extern IL_DRV_MCU1_VCU2_0x89_U uTxDrvMcu1Vcu20x89;

typedef struct
{
	uint16 lot;
	uint16 dos;
	uint16 lotFilter;
	uint16 dosFilter;
}IL_DRV_RT_INFO_0x90_T;
typedef union
{
	uint8 byte[8];
	IL_DRV_RT_INFO_0x90_T dat;
}IL_DRV_RT_INFO_0x90_U;
extern IL_DRV_RT_INFO_0x90_U uTxDrvRtInfo0x90;


typedef struct
{
	uint16 motorSpeed;
	uint16 ksiVoltage;
	uint16 motorTorq;
	uint16 phaseCurrent;
}IL_DRV_MCU1_VCU3_0x188_T;
typedef union
{
	uint8 byte[8];
	IL_DRV_MCU1_VCU3_0x188_T dat;
}IL_DRV_MCU1_VCU3_0x188_U;
extern IL_DRV_MCU1_VCU3_0x188_U uTxDrvMcu1Vcu30x188;

typedef struct
{
	uint16 faultCode;
	uint8  motorTemp;
	uint8  envTemp;
	uint8  swVersion;
	uint8  inCurrent;
	uint16 inVoltage;
}IL_DRV_MCU1_VCU4_0x288_T;
typedef union
{
	uint8 byte[8];
	IL_DRV_MCU1_VCU4_0x288_T dat;
}IL_DRV_MCU1_VCU4_0x288_U;
extern IL_DRV_MCU1_VCU4_0x288_U uTxDrvMcu1Vcu40x288;
/*===============CAN 2 Rx Message===============*/
typedef struct
{
	uint16 paraAdd;
	int32 paraVal;
}IL_USER_PARA_T;
typedef union
{
	uint8 byte[8];
	IL_USER_PARA_T dat;
}IL_RX_USER_PARA_U;
extern IL_RX_USER_PARA_U uIlRxReadPara0x04300164;
extern IL_RX_USER_PARA_U uIlRxSavePara0x04500164;
extern IL_RX_USER_PARA_U uIlRxSetPara0x04600164;
/*===============CAN 2 Tx Message===============*/
typedef struct
{
	uint16 paraAdd;
	uint16 paraSta;
	uint32 paraVal;
}IL_TX_0x4616401_T;
typedef union
{
	uint8 byte[8];
	IL_TX_0x4616401_T dat;
}IL_TX_0x4616401_U;
extern IL_TX_0x4616401_U uIlTxParaSend0x4616401;

typedef union
{	struct
	{
		uint32_t byte0:8; // 
		uint32_t inx:16; // 
		uint32_t SUBindex:8; // 
		uint8_t data4[4]; // 
	}msg;
	uint8 byte[8];
}IL_DRV_MCU1_INFO_0x582_T;
extern IL_DRV_MCU1_INFO_0x582_T uIlTxDrvMcuINFOx582;

typedef union
{	struct
	{
		uint32_t byte0:8; // 
		uint32_t inx:16; // 
		uint32_t SUBindex:8; // 
		uint8_t data4[4]; // 
	}msg;
	uint8 byte[8];
}IL_DRV_MCU1_INFO_0x583_T;
extern IL_DRV_MCU1_INFO_0x583_T uIlTxDrvMcuINFOx583;

extern void IL_InitTask(void);
extern void IL_MainTask(void);
extern void IL_Can1MessageRxISR(uint32 can_id, uint8 *pmsg);
extern void IL_Can2MessageRxISR(uint32 can_id, uint8 *pmsg);
extern void IL_Can1MessageTxProgress(void);
extern void IL_Can2MessageTxProgress(void);
extern void IL_Can1SendMessageToBus(void);
extern void IL_Can2SendMessageToBus(void);

extern void IL_FillSendMessage0x582_SET(uint8* CAN_data);
extern void IL_FillSendMessage0x583_SET(uint8* CAN_data);


#endif
