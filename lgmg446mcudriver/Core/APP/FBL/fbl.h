#ifndef __FBL_H__
#define __FBL_H__
#include "def.h"
#include "fbl_cfg.h"
typedef enum
{
    APP_A=0,
    APP_NUM,
}APP_ENUM;

typedef enum
{
    APP_STATE_UNKNOW=0,
    APP_STATE_NORMAL,
    APP_STATE_NONE,
    APP_STATE_FAIL,
    APP_STATE_REQUEST,
    APP_STATE_NUM,
}APP_STATE_ENUM;

typedef struct
{
    uint32 blockAddr;
    uint32 blockSize;
    uint32 blockCrc;
}BLOCK_MSG_T;
typedef struct
{
    uint32 fblMsg;
    uint16 upgCnt;
    uint8  actFlg;
    uint8  blkNum;
    BLOCK_MSG_T blkMsg[10];
    uint32 appCanBaudRate;
}FBL_MSG_T;
extern FBL_MSG_T tFblMsg[APP_NUM];

extern void FBL_ResetSystem(void);
extern U08 FBL_JumpToApp(uint8 app);
extern void FBL_InitTask(void);
extern void FBL_MainTask(void);
extern uint8 FBL_GetEcuAddress(void);
extern void FBL_WriteUpgradeSuccessMessage(uint8 app);
extern void FBL_WriteUpgradeFailedMessage(uint8 app);

#endif
