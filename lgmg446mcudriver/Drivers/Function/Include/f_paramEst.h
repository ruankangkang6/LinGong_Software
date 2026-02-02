/*
 * f_paramEst.h
 *
 *  Created on: 2024年12月2日
 *      Author: pc
 */

#ifndef PARAMEST_H_
#define PARAMEST_H_

#define ROTOR_ZERO_EST_PIOUT_INIT_VAL   (0.05f)
#define TUNE_RUN_OVER_TIME_MAX          180UL      // 运行时的调谐限时时间，_s

#define TUNE_STEP_WAIT      0           // 正在辨识，性能没有任何指令
#define TUNE_STEP_END       100         // 辨识结束。保存结果.
#define TUNE_STEP_STOP      1000        // 停止辨识

typedef struct PM_PARAM_EST_CONTROL_DEF
{
    uint16_t Counter;                             //1时间计数

    uint16_t RunRotorInitPosEst;                  //零位辨识开启标志
    uint16_t RunRsEst;//
    uint16_t RunLdqEst;//
    uint16_t RunBemfEst;//

    float RotorInitPosEst;//1
    float RotorInitPosEstDuty;//
    float RotorInitPosEstMaxCurr;

    float RotorZeroPosFilter1;//1
    float RotorZeroPosFilter2;//1
    float RotorZeroPosFilter3;//1
    float RotorZeroPosFilter4;//1
    uint16_t  RotorInitPosEstStep;               //零位辨识步骤。

    uint16_t  RotorZeroEstOnly; //11
    uint16_t  RotorZeroCompensateEn;
    PIReg   RotorZeroCompensatePIReg;
    float RotorZeroCompensateIqRef;
    float RotorZeroCompensateIqSum;
    uint16_t  RotorZeroCompensateIqCnt;
    float RotorZeroCompensateIdMax;
    float RotorZeroCompensateVal;
    float RotorZeroCompensateValSum;
    uint16_t  RotorZeroCompensateValCnt;

    uint16_t  RsEstStep;//
    float Rs;
    float RsEstCurr1;
    float RsEstCurr2;
    float RsEstDuty;//
    float RsEstDutyMax;
    float Ures1;//
    float Ures2;//
    float Ires1;//
    float Ires2;//
    uint16_t  Rs1Cnt;//
    uint16_t  Rs2Cnt;//
    float Ures1Sum;//
    float Ures2Sum;//
    float Ires1Sum;//
    float Ires2Sum;//

    float Ld;
    float Lq;
    float LdqEstFreqInject;
    float LdqEstWi;
    float LdqEstTheta;
    float VInject;//
    float IpFilter;//
    float InFilter;//

    float Bemf;
    float BemfEstWi;
    float BemfEstUqFilter;
    float BemfEstSpdRef;   //反电势常数辨识的速度给定
    float BemfEstSpdRefTarget;////反电势常数辨识的目标速度
    uint16_t  BemfEstFaultCnt;//
    uint16_t  BemfEstFaultCnt1;//
    uint16_t  RunBemfEstStep;//
    float BemfEstVq1Sum;
    float BemfEstId1Sum;
    float BemfEstVq2Sum;
    float BemfEstId2Sum;
    float BemfEstVq1;
    float BemfEstVq2;
    float BemfEstId1;
    float BemfEstId2;
    uint16_t  BemfEstCnt1;
    uint16_t  BemfEstCnt2;

    float CurrIdDecoupleDiv1;
    float CurrIdDecoupleDiv2;
    float CurrIqDecoupleDiv1;
    float CurrIqDecoupleDiv2;
}PM_PARAM_EST_CONTROL;

typedef struct PN_SEQ_EXTRACT_STRUCT_DEF
{
    float Alpha;
    float Beta;
    float Alpha1;
    float Alpha2;
    float Beta1;
    float Beta2;
    float AlphaP;
    float BetaP;
    float AlphaN;
    float BetaN;
    float dPos;
    float qPos;
    float dNeg;
    float qNeg;
} PN_SEQ_EXTRACT_STRUCT;

extern int16_t g_PMEstOn;   //参数辨识开启状态

extern PN_SEQ_EXTRACT_STRUCT gPNSeqExtract;
extern PM_PARAM_EST_CONTROL gPMEstControl;
extern PIReg gRotorZeroPIReg;


extern void PMEstInit(void);
extern void RunCasePMEst(void);
extern void RunParamEstInIsr( void );

#endif /* PARAMEST_H_ */
