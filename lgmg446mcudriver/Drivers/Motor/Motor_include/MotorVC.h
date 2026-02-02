/*
 * MotorVCInclude.h
 *
 *  Created on: 2024年12月4日
 *      Author: pc
 */

#ifndef SOURCE_MOTOR_INCLUDE_MOTORVC_H_
#define SOURCE_MOTOR_INCLUDE_MOTORVC_H_


typedef struct ROTOR_SPEED_STRUCT_DEF
{
    float Speed;
    float SpeedApply;
    float SpeedApplyFilter;
    float SpeedApplyFilter500ms;
    float SpeedApplyFilter500msZ1;
    float SpeedApplyFilter500msZ2;
    uint16_t  SpeedApplyFilter500msCnt;
    float SpeedApplyFilter500msSum;
}RotorSpeedStruct;    //速度反馈部分数据结构
extern RotorSpeedStruct gRotorSpeed;

typedef struct SYN_PWM_STRUCT_DEF {
    uint32_t  FcApply;
    uint16_t  ModuleSet;      //设置的调制方式：0异步；1：同步；3：方波
    uint16_t  ModuleApply;    //实际使用的调制方式
    uint16_t  Index;          //同步调制的载波比
    uint16_t  Flag;
}SYN_PWM_STRUCT;    //
extern SYN_PWM_STRUCT gSynPWM;

typedef struct PI
{
    float Ref;
    float Fb;
    float Err;
    float ErrZ;
    float ErrLmt;
    float PIOut;
    float MaxPIOut;
    float MinPIOut;
    float Kp;
    float Ki;
}PIReg;
extern PIReg gIMAcr;
extern PIReg gITAcr;


typedef struct ACR_STRUCT_DEF
{
    uint16_t  GainChgSel;                      //电流环增益切换选择  新增
    float ChgSpd;                          //电流环增益切换速度点 新增
    float ChgCurr;                         //电流环增益切换电流点 新增
    float ChgHysterVal;                    //电流环增益切换滞环值 新增

    float IdKpH;                           //D轴电流环KP1 新增
    float IdKiH;                           //D轴电流环Ki1 新增
    float IdKpL;                           //D轴电流环KP2 新增
    float IdKiL;                           //D轴电流环Ki2 新增

    float IqKpH;                           //Q轴电流环KP1 新增
    float IqKiH;                           //Q轴电流环Ki1 新增
    float IqKpL;                           //Q轴电流环KP2 新增
    float IqKiL;                           //Q轴电流环Ki2 新增
}ACR_PICHG_STRUCT;

extern ACR_PICHG_STRUCT gAcr;

typedef struct MT_STRUCT_DEF{
    float  M;
    float  T;
    float  MFilter;
    float  TFilter;
    float  Alpha;
    float  Beta;
}MT_STRUCT; //MT轴系下的电流、电压结构
extern MT_STRUCT                gIMTSet;        //MT轴系下的设定电流
extern MT_STRUCT                gIMTSetApply;   //MT轴系下的电流指令值
extern MT_STRUCT                gIMT;  //MT轴系下的电流
extern MT_STRUCT                gUMTSet;        //MT轴系下的设定电压

typedef struct ANGLE_STRUCT_DEF {
    float IMPhase;            //M轴角度
    float ICOS;
    float ISIN;
}ANGLE_STRUCT;
extern ANGLE_STRUCT gPhase;

typedef struct IUVW_SAMPLING_STRUCT_DEF{
    float U;
    float V;
    float W;
    float IDC;
    float UErr;
    float VErr;
    float Coff;
    float IdcCoff;
}IUVW_SAMPLING_STRUCT;  //电流采样结构
extern IUVW_SAMPLING_STRUCT gCurSamp;

typedef struct UVW_STRUCT_DEF{
    float     U;
    float     V;
    float     W;
}UVW_STRUCT;    //定子三相坐标轴电流
extern UVW_STRUCT gIUVW;      //定子三相坐标轴电流

typedef struct ALPHABETA_STRUCT_DEF{
    float     Alph;
    float     Beta;
}ALPHABETA_STRUCT;//定子两相坐标轴电流、电压结构
extern ALPHABETA_STRUCT        gIAlphBeta; //定子两相坐标轴电流


typedef struct UALPHABETA_STRUCT_DEF{
    int     Alph;               //Q12
    int     Beta;
}UALPHABETA_STRUCT;
extern UALPHABETA_STRUCT       gUAlphBeta;

typedef struct CURR_DECOUPLE_STRUCT_DEF
{
    float a;
    float b;
    float c;
    float in;
    float inZ;
    float out;
    float outZ;

    float LdivRs;
    float Ts;
}CURR_DECOUPLE_STRUCT;

extern CURR_DECOUPLE_STRUCT gIdDecouple;
extern CURR_DECOUPLE_STRUCT gIqDecouple;

typedef struct LINE_CURRENT_STRUCT_DEF{
    float     Current;
    float     CurrentFilter;
    float     ErrorShow;          //过流时刻记录的线电流有效值
}LINE_CURRENT_STRUCT;//计算过程中使用的线电流表示
extern LINE_CURRENT_STRUCT      gLineCur;


typedef struct MOTOR_POWER_TORQUE_DEF{
    float     TorqueRef;           //执行扭矩
    float     Torque;
    float     TrqOut;             // 输出转矩
}MOTOR_POWER_TORQUE;
extern MOTOR_POWER_TORQUE      gPowerTrq;


extern float gf32IdRefFilter;
extern float gf32IqRefFilter;


extern void CalOutputPhase(void);
extern void TransformCurrent(void);
extern void VCCsrControlEx(void);
extern float RoundArcAngle(float ArcAngle);
extern void ResetCsrPar(void);
extern void CalcActTorque( void );
extern void CalcLineCurr(void);
extern void CalcCsrDecoupleVal( void );
extern void SelectAcrPar(void);


#endif /* SOURCE_MOTOR_INCLUDE_MOTORVC_H_ */
