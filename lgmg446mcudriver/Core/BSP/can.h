#ifndef __CAN_H__
#define __CAN_H__	 
#include "def.h"

extern uint8 CAN_Can1SendMessage(const uint32 can_id,u8 *pbuf,u8 len);
extern uint8 CAN_Can2SendMessage(const uint32 can_id,u8 *pbuf,u8 len);
extern void CAN_Can1ReadFifo0Message(void);
extern void CAN_Can2ReadFifo1Message(void);
#endif

















