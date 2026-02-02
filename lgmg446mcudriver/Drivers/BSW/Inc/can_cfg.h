/*
 * can_cfg.h
 *
 *  Created on: Feb 20, 2025
 *      Author: Administrator
 */

#ifndef SRC_CAN_CFG_H_
#define SRC_CAN_CFG_H_

extern void CAN1_Rx_Msg_Init(void);
extern void Can1_Rx_Process(void);
extern void CAN1_Tx_Msg_Init(void);

extern void Can1_Tx_Process(void);
extern void Can1_Tx_Message(void);
extern void CAN1_Fifo0_Full_IRQHandler(void);
extern void CAN1_Fifo1_Full_IRQHandler(void);



#endif /* SRC_CAN_CFG_H_ */
