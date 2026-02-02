/*
 * TorqueDeal.h
 *
 *  Created on: 2024年11月29日
 *      Author: pc
 */

#ifndef TORQUEDEAL_H_
#define TORQUEDEAL_H_

#define TIME_UNIT_TORQUE_CTRL_ACC_DEC       10      // 转矩控制加减速时间单位, ms

extern float g_torquecmd;

extern void TorqueCalc(void);


#endif /* TORQUEDEAL_H_ */
