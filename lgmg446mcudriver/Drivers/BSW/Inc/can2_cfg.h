/*
 * can2_cfg.h
 *
 *  Created on: Feb 26, 2025
 *      Author: Administrator
 */

#ifndef SRC_CAN2_CFG_H_
#define SRC_CAN2_CFG_H_
#include "main.h"
extern void CAN2_Rx_Msg_Init(void);
extern void Can2_Rx_Process(void);
extern void CAN2_Tx_Msg_Init(void);

extern void Can2_Tx_Message(uint32_t CanID, unsigned char *TxData);
extern void CAN2_Fifo0_Full_IRQHandler(void);
extern void CAN2_Fifo1_Full_IRQHandler(void);

extern uint16_t g_DriveUpgradeFlag;

#endif /* SRC_CAN2_CFG_H_ */
