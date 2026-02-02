
#ifndef	__UDS_H__
#define __UDS_H__
#include "def.h"

#define UDS_FUN_ADDR	(0x7DF)
#define UDS_PHY_ADDR	(0x700)
#define UDS_RES_ADDR	(0x708)


#define UDS_DAT_LEN			(4096)
#define UDS_PAGE_SIZE		(1024)
typedef enum
{
	UDS_DRV_NONE=0,
	UDS_DRV_CAN1,
	UDS_DRV_CAN2,
	UDS_DRV_UART,
}UDS_DRV_ENUM;

typedef struct
{
	uint8  buffTx[UDS_DAT_LEN];
	uint8  buffRx[UDS_DAT_LEN];
	uint16 buffSize;
	uint8  drvNo;
	uint8  addrOffset;
	uint8  sessCtrl;
	uint8  secyAccs;
	uint8  frameLen;
	uint8  rollCnt36;
	uint8  seed270304[4];
	uint8  msgSendDisable;
	uint8  dtcSaveDisable;
	uint32 sessTmr;
	uint32 writeAddr;
	uint32 flaAddr;
	uint32 flaSize;
	uint32 chkSum;
}UDS_SERVICE_T;
extern UDS_SERVICE_T tUdsSrv;

extern void UDS_InitTask(void);
extern uint32 UDS_Crc32CheckSum(UINT32 init_crc, UINT8* pdat, UINT32 length);
extern void UDS_UdsServiceAnalysis(uint8 *pdat,uint16 len,uint8 drv_no);
extern void UDS_MainTask(void);
#endif
