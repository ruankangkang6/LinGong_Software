/*
 * f_asrControl.h
 *
 *  Created on: 2024年12月2日
 *      Author: pc
 */

#ifndef ASRCONTROL_H_
#define ASRCONTROL_H_

typedef struct ASR_STRUCT_DEF
{
    PIReg   Asr;                       //PI计算
    float KPHigh;                    //高速KP
    float KPLow;                     //低速KP
    float KIHigh;                    //高速KI
    float KILow;                     //低速KI
    int16_t   SwitchLow;                 //低速切换阈值
    int16_t   SwitchHigh;                //高速切换阈值
    float OutFilterCoef;             //PI输出滤波系数
    float AsrOutFilter;              //PI输出滤波值
}ASR_STRUCT;


extern ASR_STRUCT gAsr;
extern uint16_t g_MaxFeedbackCurr;
extern uint16_t g_SpdCtrlEnable;

extern void VCSpeedControl(void);
extern void ResetAsrPar( void );

#endif /* ASRCONTROL_H_ */
