/*
 * MotorAdc.h
 *
 *  Created on: 2024年12月4日
 *      Author: pc
 */

#ifndef SOURCE_MOTOR_INCLUDE_MOTORADC_H_
#define SOURCE_MOTOR_INCLUDE_MOTORADC_H_

//#define	ADC_RESULT_I_A		(2048)
//#define	ADC_RESULT_I_B		(2048)
//#define	ADC_RESULT_I_C		(2048)
//#define	ADC_RESULT_V_DC		(2048)
//
//#define	ADC_RESULT_MLTTEMP	(2048) //IGBTtemp  AMBtemp
//#define	ADC_RESULT_TEMP		(2048) //motortemp
//
//
//#define	ADC_RESULT_ACCA		(2048) //ACC_AD       //加速踏板深度
//#define	ADC_RESULT_VLV		(2048) //AD_Ctrl_Vin  //预充电压

typedef struct UDC_STRUCT_DEF {
    float uDC;                //母线电压
    float uDCFilter;          //滤波母线电压，查表使用？
    float uDCBigFilter;       //滤波母线电压， 过压、欠压判断用母线电压
}UDC_STRUCT;    //母线电压数据

typedef struct AD_SAMPLE_DEF
{
    float f32Ia;              //A相电流采样
    float f32Ib;              //B相电流采样
    float f32Ic;              //C相电流采样
    float f32Vdc;             //母线电压采样
    float f32Temp;            //IGBT温度采样
    float f32AI4;             //电机温度采样1
    float f32AI5;             //电机温度采样2
    float f32Temp_AMB;        //环境温度采样
}AD_SAMPLE;


typedef struct AD_CONVERT_DEF
{
    AD_SAMPLE gain;
    AD_SAMPLE offset;
}AD_CONVERT;

typedef struct AD_OFFSET_STRUCT_DEF{
    float     Count;
    float     EnableCount;
    uint16_t        Flag;
}AD_OFFSET_STRUCT;//检测零漂使用的结构


extern uint16_t gADCResetTime;
extern AD_CONVERT g_ADConvert;
extern AD_CONVERT g_ADConvertSum;
extern AD_SAMPLE  g_SampleRealValue;
extern AD_SAMPLE  g_SampleDValue;
extern AD_SAMPLE  g_FirstSampleValue;
extern AD_OFFSET_STRUCT  gADOffsetInfo;
extern UDC_STRUCT gUDC;

extern float ACC_AD;
extern float Ctrl_Vin;

//extern void InitAdc(void);//ADC外设初始化
extern void GetAdcResult(void);
extern void ADCProcess(void);

#endif /* SOURCE_MOTOR_INCLUDE_MOTORADC_H_ */
