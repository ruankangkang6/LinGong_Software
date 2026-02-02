/*
 * f_faultProtect.h
 *
 *  Created on: 2025 Jan 24
 *      Author: pc
 */

#ifndef FUNCTION_INCLUDE_F_FAULTPROTECT_H_
#define FUNCTION_INCLUDE_F_FAULTPROTECT_H_

//---------------------故障结构体----------------------------------
//1级故障
typedef union FAULT_LV1
{
    uint16_t all;

    struct
    {
        uint16_t REV0                        :8;       // bit0-bit7
        uint16_t REV1                        :8;       // bit8-bit15
    }bit;
}FaultLv1;
extern FaultLv1 gFault1st;

//2级故障
typedef union FAULT_LV2
{
    uint16_t all;

    struct
    {
        uint16_t Motor1SenLowFault           :1;       // bit0  电机温度传感器1短路
        uint16_t Motor2SenLowFault           :1;       // bit1  电机温度传感器2短路
        uint16_t Motor1SenHighFault          :1;       // bit2  电机温度传感器1断路
        uint16_t Motor2SenHighFault          :1;       // bit3  电机温度传感器2断路
        uint16_t IGBTSenLowFault             :1;       // bit4  IGBT温度传感器短路
        uint16_t IGBTSenHighFault            :1;       // bit5  IGBT温度传感器断路
        uint16_t AmbSenLowFault              :1;       // bit6  环境温度传感器短路
        uint16_t AmbSenHighFault             :1;       // bit7  环境温度传感器断路

        uint16_t MotorTempLimTor             :1;       // bit8  电机过温限功率
        uint16_t IgbtTempLimTor              :1;       // bit9  IGBT过温限功率
        uint16_t REV0                        :6;       // bit10-bit15
    }bit;
}FaultLv2;
extern FaultLv2 gFault2nd;

//3级故障
typedef union FAULT_LV3
{
    uint16_t all;

    struct
    {
        uint16_t SwOverCurrent               :1;       // bit0  软件过流
        uint16_t HwOverCurrent               :1;       // bit1  硬件过流
        uint16_t SwOverUdc                   :1;       // bit2  软件过压
        uint16_t HwOverUdc                   :1;       // bit3  硬件过压
        uint16_t SwUnderUdc                  :1;       // bit4  软件欠压
        uint16_t ErrorRTTrans                :1;       // bit5  旋变故障
        uint16_t OverIGBTTemp                :1;       // bit6  IGBT过温
        uint16_t IGBTFault                   :1;       // bit7  IGBT故障

        uint16_t ErrorCurrentCheck           :1;       // bit8  零漂故障
        uint16_t OverSpeed                   :1;       // bit9  超速故障
        uint16_t OverMotorTemp               :1;       // bit10 电机过温
        uint16_t ErrorParamEst               :1;       // bit11 参数辨识故障
        uint16_t LackPhase                   :1;       // bit12 缺相故障
        uint16_t CanTimeOut                  :1;       // bit13 CAN超时故障
        uint16_t MainContactorFault          :1;       // bit14 主接触器粘连
        uint16_t PreChargeFault              :1;       // bit15 预充失败
    }bit;
}FaultLv3;
extern FaultLv3 gFault3rd;

//---------------------故障保护点----------------------------------
typedef struct LIMIT_PARAMS_STRUCT_DEF
{
    float IACOffsetLimit;         //零漂故障检测阈值

    float motorTempLimit;         //电机过温保护点
    float motorTempWarn;          //电机过温预警点
    float igbtTempLimit;          //IGBT过温保护点
    float igbtTempWarn;           //IGBT过温预警点

    float speedPLimit;            //正转超速保护点
    float speedNLimit;            //反转超速保护点
    float OcpInstLimit;           //瞬时过流点
    float OcpFilterLimit;         //滤波过流点

    float  LowVoltUnderVol;       //低压欠压保护阈值
    float  LowVoltOverVol;        //低压过压保护阈值

    float VoltOnDiff;             //开机电压滞环
    float inputVoltOff;           //母线电压欠压保护点
    float inputVoltOvr;           //母线电压过压保护点

    float  LockRotorCarrier;      //堵转载频
    float  LockRotorTorProt;      //堵转保护扭矩
    float  LockRotorTimeProt;     //堵转保护时间
    float  LockRotorSafeTor;      //堵转安全扭矩

}LIMIT_PARAMS_STRUCT;
extern LIMIT_PARAMS_STRUCT   gLimitParams;

void TemperatureProtect(void);
void StallProtect(void);
void ImmSoftWareErrorDeal( void );
void HardWareOCDeal(void);
float MaxUVWCurrent( void );
void CalcADOffset( void );
void RTCheck(void);
void SoftWareErrorDeal(void);
void SenLowFaultCheck(void);

#endif /* FUNCTION_INCLUDE_F_FAULTPROTECT_H_ */
