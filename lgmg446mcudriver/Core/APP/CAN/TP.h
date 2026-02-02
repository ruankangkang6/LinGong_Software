#ifndef __TP_H__
#define __TP_H__
#include "def.h"




typedef enum
{
	FRAME_TYPE_SF=0,
	FRAME_TYPE_FF,
	FRAME_TYPE_CF,
	FRAME_TYPE_FC,
	FRAME_TYPE_MF,
	FRAME_TYPE_OK,
	FRAME_TYPE_ERR,
}FRAME_TYPE_E;

typedef struct
{
	uint8  datBuf[2050];
	uint8  frmTyp;
	uint8  frmCnt;
	uint16 datLen;
	uint16 datCnt;
	uint16 canID;
	
}TP_DAT_T;

extern void TP_Can1ReceiveMessageHandler(uint32 id,uint8 *pdat);
extern void TP_Can2ReceiveMessageHandler(uint32 id,uint8 *pdat);
extern void TP_Can1TransmitData(uint8 *pdat,uint16 len);
extern void TP_Can2TransmitData(uint8 *pdat,uint16 len);

extern void TP_InitTask(void);
extern void TP_MainTask(void);
#endif
