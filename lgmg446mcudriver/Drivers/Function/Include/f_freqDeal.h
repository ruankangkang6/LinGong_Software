/*
 * STatusSkip.h
 *
 *  Created on: 2024年11月21日
 *      Author: pc
 */

#ifndef SPEEDDEAL_H_
#define SPEEDDEAL_H_

//#define FREQ_AIM_EPS    0.00001f                //hengd

#define FORWARD_DIR         0   // 正方向
#define REVERSE_DIR         1   // 反方向

extern float g_freqSet;
extern uint32_t g_AccFrqTime;
extern uint32_t g_DecFrqTime;


extern void FrqSrcDeal(void);

#endif /* STATUSSKIP_H_ */
