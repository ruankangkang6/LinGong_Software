/*
 * MotorCarrier.h
 *
 *  Created on: 2024年12月20日
 *      Author: pc
 */

#ifndef SOURCE_MOTOR_INCLUDE_MOTORCARRIER_H_
#define SOURCE_MOTOR_INCLUDE_MOTORCARRIER_H_

#define C_INIT_PRD          9000//3750    //初始(10KHz)的PWM周期（PWM定时器时间为20ns）


typedef struct FC_CAL_STRUCT_DEF{
    uint16_t  Cnt;
    uint16_t  Time;
    uint16_t  FcBak;
    uint16_t  FcLow;
}FC_CAL_STRUCT;//计算载波频率程序使用的数据结构

typedef struct BASE_PAR_STRUCT_DEF {
    uint16_t  FcSet;                  //设定载波频率
    uint16_t  FcSetApply;             //实际载波频率
    float Ts;                     // carrier wave period
}BASE_PAR_STRUCT;   //基本运行信息结构

extern FC_CAL_STRUCT           gFcCal;
extern BASE_PAR_STRUCT         gBasePar;   //基本运行参数


extern void CalCarrierWaveFreq(void);
#endif /* SOURCE_MOTOR_INCLUDE_MOTORCARRIER_H_ */
