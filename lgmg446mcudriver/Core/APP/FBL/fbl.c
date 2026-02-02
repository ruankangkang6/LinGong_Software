#include "main.h"
#include "fbl_cfg.h"
#include "fbl.h"
#include "uds.h"
#include "can.h"
#include "uds.h"
#include "def.h"

FBL_MSG_T tFblMsg[APP_NUM];

void (*JumpToApp)(void);

uint32 AppStartDelay=0;

void FBL_ResetSystem(void)
{
}
void FBL_SetAppCanBaudRate(uint16 baud_rate)
{
	APP_CAN_BAUD_RATE = baud_rate;
	APP_CAN_BAUD_RATE = APP_CAN_BAUD_RATE<<16;
	APP_CAN_BAUD_RATE = ~APP_CAN_BAUD_RATE;
	APP_CAN_BAUD_RATE = (APP_CAN_BAUD_RATE)&0xFFFF0000;
	APP_CAN_BAUD_RATE = APP_CAN_BAUD_RATE + baud_rate;
}
uint8 FBL_GetEcuAddress(void)
{
	uint16 addr_h,addr_l,addr=0;

	APP_RUNNING_FLAG = 0xAA55A541;

	addr_l = ECU_CAN_ADDRESS;
	addr_h = ECU_CAN_ADDRESS>>16;
	addr_h = ~addr_h;
	if(addr_h==addr_l)
	{
		if(addr_l<=2)
		{
			addr = addr_l;
		}
	}
	return addr;
}

void FBL_InitTask(void)
{


}
void FBL_MainTask(void)
{

}

