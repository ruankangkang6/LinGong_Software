/*
 * MotorPWM.h
 *
 *  Created on: 2024年12月16日
 *      Author: pc
 */

#ifndef SOURCE_MOTOR_INCLUDE_MOTORPWM_H_
#define SOURCE_MOTOR_INCLUDE_MOTORPWM_H_

//作为PWM输出的结构
typedef struct PWM_OUT_STRUCT_DEF {
    float U;
    float V;
    float W;              //该结构前面的参数不要改变
    float Fan;
    uint16_t gPWMPrd;        //计算得到载波周期（20ns为单位）
    uint16_t gPWMPrdApply;   //中断中实际使用载波周期
}PWM_OUT_STRUCT;
extern PWM_OUT_STRUCT gPWM;


extern uint16_t g_PwmEnable;
extern uint16_t g_PwmAlreadyOn;

extern void OutPutPWMVF(void);
extern void SendPWM(void);

#endif /* SOURCE_MOTOR_INCLUDE_MOTORPWM_H_ */
