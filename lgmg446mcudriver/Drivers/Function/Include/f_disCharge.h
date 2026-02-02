/*
 * f_disCharge.h
 *
 *  Created on: 2024年12月2日
 *      Author: pc
 */

#ifndef DISCHARGE_H_
#define DISCHARGE_H_

#define ACTV_DISCHG_WAIT      1     // 泄放等待
#define ACTV_DISCHG_RUN       2     // 正在放电
#define ACTV_DISCHG_END       3     // 放电结束
#define ACTV_DISCHG_TIMEOUT   4     // 泄放超时


typedef struct ACTV_DISCHG_CTRL_STRUCT_DEF
{
    float IdRef;
    float IqRef;
    float SafetyVolt;
    float TimeoutSec;
} ACTV_DISCHG_CTRL_STRUCT;

extern ACTV_DISCHG_CTRL_STRUCT gActvDischgCtrl;

extern void RunActvDischgMode(void);

extern uint16_t gu16ActvDischgRunStatus;

#endif /* DISCHARGE_H_ */
