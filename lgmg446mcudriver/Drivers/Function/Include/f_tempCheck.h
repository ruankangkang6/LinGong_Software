/*
 * f_tempCheck.h
 *
 *  Created on: 2024年12月17日
 *      Author: pc
 */

#ifndef SOURCE_FUNCTION_INCLUDE_F_TEMPCHECK_H_
#define SOURCE_FUNCTION_INCLUDE_F_TEMPCHECK_H_

//电机温度采样选择
#define PT100           0
#define PT1000          1
#define NTC10K          2
#define NTC100K         3
#define NTC30K          4
#define NTC_KTY84_150   5
#define NTC_3970_3K3    6
#define NTC_XDLT_70KW_11K83 7

#define MOTOR_TEMP_SENSOR  PT1000

typedef struct TEMPLETURE_STRUCT_DEF
{
    uint16_t    TempAD;             //AD获取值，用于于温度查表
    uint16_t    MotorTempAD;        //AD获取值，用于温度查表
    uint16_t    MotorTempAD2;       //AD获取值，用于温度查表
    uint16_t    TempAMBAD2;         //AD获取值，用于温度查表
    float   Temp;               //表示的实际温度值
    float   MotorTemp;          //表示的实际温度值
    float   MotorTemp2;         //表示的实际温度值
    float   AMBTemp;            //表示的实际温度值
    uint16_t    TempBak;            //表示的实际温度值
    uint16_t    ErrCnt;             //IGBT过温保护计时
    uint16_t    ErrMotorCnt;        //电机过温保护计时
    float DerateTorqueByTEnv;   //环境温度扭矩限制
    float DerateTorqueByTIgbt;  //IGBT温度扭矩限制
    float DerateTorqueByTMotor; //电机温度扭矩限制
}TEMPLETURE_STRUCT;//和变频器温度相关的数据结构

extern uint32_t g_AccTorTime;
extern uint32_t g_AecTorTime;
extern TEMPLETURE_STRUCT gTemperature;
extern float g_DerateTorq;


void TemperatureCheck(void);
#endif /* SOURCE_FUNCTION_INCLUDE_F_TEMPCHECK_H_ */
