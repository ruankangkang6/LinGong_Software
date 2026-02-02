#ifndef __NM_H__
#define __NM_H__
#include "def.h"

typedef struct
{
	uint8 msgMiss;
	uint8 msgUpd;
	uint8 msgNever;

	uint8 mainNegRlySta;
}NM_BMS_INFO_0x53_T;
extern NM_BMS_INFO_0x53_T tNmRxBmsInfo0x53;
extern void NM_CanRxMessageAnalysis0x53(uint8 *pdat);

typedef struct
{
	uint8 msgMiss;
	uint8 msgUpd;
	uint8 msgNever;

	uint8 slopeAntiEnable;
	uint8 slopeAntiFlag2;
	uint8 brakeSignal;
	uint8 slopeSigFlag;
	uint16 bmsDcCurrent;
}NM_BRAKE_INFO_0x1CA_T;
extern NM_BRAKE_INFO_0x1CA_T tNmRxBrakeInfo0x1CA;
extern void NM_CanRxMessageAnalysis0x1CA(uint8 *pdat);

typedef struct
{
	uint8 msgMiss;
	uint8 msgUpd;
	uint8 msgNever;

	uint16 reqSpd;
	uint8  enable;
	uint8  contactorEnable;
	uint8  direction;
	uint8  diffSwitch;
}NM_VCU_DRV_MCU_0x208_T;
extern NM_VCU_DRV_MCU_0x208_T tNmRxVcuDrvMcu0x208;
extern void NM_CanRxMessageAnalysis0x208(uint8 *pdat);

void NM_CanRxMessageAnalysis0x602(uint8 *pdat);

void NM_CanRxMessageAnalysis0x603(uint8 *pdat);



typedef struct
{
	uint8 msgMiss;
	uint8 msgUpd;
	uint8 msgNever;

	uint16 faultCode;
	int16 spdReq;
	int16 spdReal;
	int16 spdLoop;
}NM_DRV_MCU1_VCU1_0x88_T;
extern NM_DRV_MCU1_VCU1_0x88_T tNmTxDrvMcu1Vcu10x88;

typedef struct
{
	uint8 msgMiss;
	uint8 msgUpd;
	uint8 msgNever;

	uint16 faultCode;
	int16 piRev;
	uint8  brakeSignalFB	;
	uint8  slopeAntiFlag2FB;
	uint8  slopeTriggerFB	;
	uint8  piLock			;
	uint8  clearFlag		;
	uint8  brakeBuild		;
	uint8  flagEnableOff	;
	int16  mosTemp;
}NM_DRV_MCU1_VCU2_0x89_T;
extern NM_DRV_MCU1_VCU2_0x89_T tNmTxDrvMcu1Vcu20x89;

typedef struct
{
	uint8 msgMiss;
	uint8 msgUpd;
	uint8 msgNever;
	
	uint16 lot;
	uint16 dos;
	uint16 lotFilter;
	uint16 dosFilter;
}NM_DRV_RT_INFO_0x90_T;
extern NM_DRV_RT_INFO_0x90_T tNmTxDrvRtInfo0x90;

typedef struct
{
	uint8 msgMiss;
	uint8 msgUpd;
	uint8 msgNever;

	uint16 motorSpeed;
	uint16 ksiVoltage;
	int16 motorTorq;
	uint16 phaseCurrent;
}NM_DRV_MCU1_VCU3_0x188_T;
extern NM_DRV_MCU1_VCU3_0x188_T tNmTxDrvMcu1Vcu30x188;

typedef struct
{
	uint8 msgMiss;
	uint8 msgUpd;
	uint8 msgNever;

	uint16 faultCode;
	int16  motorTemp;
	int16  envTemp;
	uint8  swVersion;
	uint8  inCurrent;
	uint16 inVoltage;
}NM_DRV_MCU1_VCU4_0x288_T;
extern NM_DRV_MCU1_VCU4_0x288_T tNmTxDrvMcu1Vcu40x288;

typedef struct
{
	uint8 msgMiss;
	uint8 msgUpd;
	uint8 msgNever;


}NM_DRV_MCU1_INFO_0x582_T;
extern NM_DRV_MCU1_INFO_0x582_T tNmTxDrvMcu1INFOx582;

typedef struct
{
	uint8 msgMiss;
	uint8 msgUpd;
	uint8 msgNever;


}NM_DRV_MCU1_INFO_0x583_T;
extern NM_DRV_MCU1_INFO_0x583_T tNmTxDrvMcu1INFOx583;

typedef struct
{
	uint8 msgMiss;
	uint8 msgUpd;
	uint8 msgNever;

	uint16 paraAdd;
	uint32 paraVal;
}NM_RX_0x4300164_T;
extern NM_RX_0x4300164_T tNmRxReadPara0x4300164;
extern NM_RX_0x4300164_T tNmRxSavePara0x4500164;
extern NM_RX_0x4300164_T tNmRxSetPara0x4600164;


typedef struct
{
	uint8 msgUpd;

	uint16 paraAdd;
	uint16 paraSta;
	int32 paraVal;
}NM_TX_0x4616401_T;
extern NM_TX_0x4616401_T tNmTx0x4616401;

extern void NM_CanRxMessageAnalysis4300164(uint8 *pdat);
extern void NM_CanRxMessageAnalysis4500164(uint8 *pdat);
extern void NM_CanRxMessageAnalysis4600164(uint8 *pdat);




#endif
