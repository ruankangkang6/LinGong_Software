/*
 * f_paramEst.c
 *
 *  Created on: 2024年12月2日
 *      Author: pc
 */
#include "f_public.h"

//#pragma CODE_SECTION(PNSeqExtrack,      "ramfuncs")

PM_PARAM_EST_CONTROL gPMEstControl;
PIReg gRotorZeroPIReg;
PN_SEQ_EXTRACT_STRUCT gPNSeqExtract;

static uint16_t s_u16IPosSwitch = 0;
static uint16_t gu16MotorCtrlTuneStatus = 0;     // 参数辨识状态字，性能传递
uint16_t g_u16ParamsIdentify2Func[20];
static const uint16_t pmsmParaIndex[] =
{
        3,
        4,
        5,
        6
//    GetCodeIndex(funcCode.code.pmsmLd),                 // C4-00  D轴电感
//    GetCodeIndex(funcCode.code.pmsmLq),                 // C4-01  Q轴电感
//    GetCodeIndex(funcCode.code.pmsmRs),                 // C4-02  电阻
//    GetCodeIndex(funcCode.code.pmsmZeroPosCompVal),     // C4-03  初始位置精确补偿角
//    GetCodeIndex(funcCode.code.pmsmBackBMF),            // C4-04  反电动势
//
//    GetCodeIndex(funcCode.code.disconnectionTime),      // C4-05  断线检测时间
//    GetCodeIndex(funcCode.code.pmsmKPd),                // C4-06  D轴KP
//    GetCodeIndex(funcCode.code.pmsmKId),                // C4-07  D轴KI
//    GetCodeIndex(funcCode.code.pmsmKPq),                // C4-08  Q轴KP
//    GetCodeIndex(funcCode.code.pmsmKIq),                // C4-09  Q轴KI
//
//    GetCodeIndex(funcCode.code.pmsmZeroPosition),       // C4-10  旋变零点位置
//    GetCodeIndex(funcCode.code.pmsmDirection),          // C4-11  旋变方向
};

void PMEstInit(void);
void RunCasePMEst(void);
void RunParamEstInIsr( void );
static void SaveTuneData(void);
static void PNSeqExtrack(void);

uint16_t G_ErrStep = 0;

void PMEstInit(void)
{
    gPMEstControl.Counter = 0;
    gu16MotorCtrlTuneStatus = TUNE_STEP_WAIT;
    gPMEstControl.RunRsEst = 0;
    gPMEstControl.RunLdqEst = 0;
    gPMEstControl.RunBemfEst = 0;
    gPMEstControl.RunRotorInitPosEst = 0;

    gPMEstControl.RotorInitPosEst           =   0;
    gPMEstControl.RotorInitPosEstDuty       =   0.001f;
    gPMEstControl.RotorInitPosEstMaxCurr    =   0.3f;//gCommProVar.InitPosEstCurr
    gPMEstControl.RotorZeroPosFilter1       =   0;
    gPMEstControl.RotorZeroPosFilter2       =   0;
    gPMEstControl.RotorZeroPosFilter3       =   0;
    gPMEstControl.RotorZeroPosFilter4       =   0;
    gPMEstControl.RotorInitPosEstStep       =   0;

    gPMEstControl.RotorZeroCompensatePIReg.Ref      =   0;
    gPMEstControl.RotorZeroCompensatePIReg.Kp       =   1.0e-3;
    gPMEstControl.RotorZeroCompensatePIReg.Ki       =   0.03e-3;
    gPMEstControl.RotorZeroCompensatePIReg.ErrLmt   =   50;
    gPMEstControl.RotorZeroCompensatePIReg.Err      =   0;
    gPMEstControl.RotorZeroCompensatePIReg.ErrZ     =   0;
    gPMEstControl.RotorZeroCompensatePIReg.PIOut    =   0;
    gPMEstControl.RotorZeroCompensatePIReg.MaxPIOut =   Value_Pi*10/180;
    gPMEstControl.RotorZeroCompensatePIReg.MinPIOut =   -gPMEstControl.RotorZeroCompensatePIReg.MaxPIOut;
    gPMEstControl.RotorZeroCompensateIqRef          =   0;
    gPMEstControl.RotorZeroCompensateIqSum          =   0;
    gPMEstControl.RotorZeroCompensateIqCnt          =   0;
    gPMEstControl.RotorZeroCompensateIdMax          =   0.20f*gMotorInfo.Current;
    gPMEstControl.RotorZeroCompensateVal            =   0;
    gPMEstControl.RotorZeroCompensateValSum         =   0;
    gPMEstControl.RotorZeroCompensateValCnt         =   0;

    gPMEstControl.Rs            =   0;
    gPMEstControl.RsEstStep     =   0;
    gPMEstControl.RsEstCurr1    =   0.25f;//gCommProVar.RsEstCurr1
    gPMEstControl.RsEstCurr2    =   0.50f;//gCommProVar.RsEstCurr2
    gPMEstControl.RsEstDuty     =   0.03f;
    gPMEstControl.RsEstDutyMax  =   0.20f;
    gPMEstControl.Ures1         =   0;
    gPMEstControl.Ures2         =   0;
    gPMEstControl.Ires1         =   0;
    gPMEstControl.Ires2         =   0;
    gPMEstControl.Rs1Cnt        =   0;
    gPMEstControl.Rs2Cnt        =   0;
    gPMEstControl.Ures1Sum      =   0;
    gPMEstControl.Ures2Sum      =   0;
    gPMEstControl.Ires1Sum      =   0;
    gPMEstControl.Ires2Sum      =   0;

    gPMEstControl.LdqEstFreqInject = 500;//

    gPMEstControl.Ld            =   0;
    gPMEstControl.Lq            =   0;
    gPMEstControl.VInject       =   0.001f;
    gPMEstControl.IpFilter      =   0;
    gPMEstControl.InFilter      =   0;
    gPMEstControl.LdqEstWi      =   Value_2Pi*gPMEstControl.LdqEstFreqInject;
    gPMEstControl.LdqEstTheta   =   0;

    gPMEstControl.Bemf                  =   0;
    gPMEstControl.BemfEstWi             =   0;
    gPMEstControl.BemfEstUqFilter       =   0;
    gPMEstControl.BemfEstSpdRef         =   1;
    gPMEstControl.BemfEstSpdRefTarget   =   0.333f*gMotorInfo.Frequency;
    gPMEstControl.BemfEstFaultCnt       =   0;
    gPMEstControl.BemfEstFaultCnt1      =   0;
    gPMEstControl.RunBemfEstStep        =   0;
    gPMEstControl.BemfEstVq1Sum         =   0;
    gPMEstControl.BemfEstId1Sum         =   0;
    gPMEstControl.BemfEstVq2Sum         =   0;
    gPMEstControl.BemfEstId2Sum         =   0;
    gPMEstControl.BemfEstVq1            =   0;
    gPMEstControl.BemfEstVq2            =   0;
    gPMEstControl.BemfEstId1            =   0;
    gPMEstControl.BemfEstId2            =   0;
    gPMEstControl.BemfEstCnt1           =   0;
    gPMEstControl.BemfEstCnt2           =   0;

    gPMEstControl.CurrIdDecoupleDiv1    =   0;
    gPMEstControl.CurrIdDecoupleDiv2    =   0;
    gPMEstControl.CurrIqDecoupleDiv1    =   0;
    gPMEstControl.CurrIqDecoupleDiv2    =   0;

    gPNSeqExtract.Alpha         =   0;
    gPNSeqExtract.Beta          =   0;
    gPNSeqExtract.Alpha1        =   0;
    gPNSeqExtract.Alpha2        =   0;
    gPNSeqExtract.Beta1         =   0;
    gPNSeqExtract.Beta2         =   0;
    gPNSeqExtract.AlphaP        =   0;
    gPNSeqExtract.BetaP         =   0;
    gPNSeqExtract.AlphaN        =   0;
    gPNSeqExtract.BetaN         =   0;
    gPNSeqExtract.dPos          =   0;
    gPNSeqExtract.qPos          =   0;
    gPNSeqExtract.dNeg          =   0;
    gPNSeqExtract.qNeg          =   0;

    gRotorZeroPIReg.Ref         =   0;
    gRotorZeroPIReg.Kp          =   1.0e-2;
    gRotorZeroPIReg.Ki          =   3.0e-4;
    gRotorZeroPIReg.ErrLmt      =   50;
    gRotorZeroPIReg.Err         =   0;
    gRotorZeroPIReg.ErrZ        =   0;
    gRotorZeroPIReg.PIOut       =   ROTOR_ZERO_EST_PIOUT_INIT_VAL;
    gRotorZeroPIReg.MaxPIOut    =   0.2;
    gRotorZeroPIReg.MinPIOut    =   0;//-0.2;
}

/************************************************************
    PM Param Estimation
************************************************************/

uint16_t s_PMEstStep = TUNE_STEP_WAIT;       //步骤切换
void RunCasePMEst(void)
{
//    static uint16_t s_PMEstStep = TUNE_STEP_WAIT;       //步骤切换
    static uint32_t runPmEstTimeCnt = 0;                //超时计数
    static uint16_t savePmEstDataFlag = 0;              //保存数据标志

    //辨识失败故障，怎么复位
    if (  (g_RunStatus != STATUS_PARAMEST)
       || (gFault3rd.all != 0)
	   )
    {
        s_PMEstStep = 1;
        savePmEstDataFlag = 0;
        PMEstInit();//执行前数据初始化

        return;
    }

    switch(s_PMEstStep)
    {
        case 1:
            // wait stop  等待
            if( fabs(gRotorSpeed.SpeedApply) < 1 )  //
            {
                if( ++gPMEstControl.Counter > 500*2 )
                {
                    gPMEstControl.Counter = 0;
                    gPMEstControl.RotorInitPosEstStep = 0;
                    gPMEstControl.RotorZeroPosFilter1 = 0;
                    gRotorZeroPIReg.Ref = 0;
                    gRotorZeroPIReg.PIOut = ROTOR_ZERO_EST_PIOUT_INIT_VAL;
                    s_PMEstStep = 2;
                }
            }
            else
            {
                gPMEstControl.Counter = 0;
            }

            break;

        case 2:
            // Rotor Init Position Est  电机初始零位辨识
            ++gPMEstControl.Counter;
            if( gPMEstControl.Counter < (uint16_t)(500*3) )           //采集转子位置(B或C相)，持续3s
            {
                gPMEstControl.RotorInitPosEstStep   =   1;
                gPMEstControl.RunRotorInitPosEst    =   1;          //开启零位辨识的电流给定

                g_PwmEnable = 1;                                  //开波

                gPMEstControl.RotorZeroPosFilter1 = 0.9f*gPMEstControl.RotorZeroPosFilter1 + ( 1 - 0.9f )*gRotorTrans.Pos;//采集位于B或C方向的转子角度
            }
            else if( gPMEstControl.Counter < (uint16_t)(500*3.5f) )   //清零，0.5s后采集转子位置(A相)
            {
                g_PwmEnable = 0;                       //开波
                DisableDrive();//关波

                gPMEstControl.RotorInitPosEstStep   =   2;
                gPMEstControl.RunRotorInitPosEst    =   0;
                gPMEstControl.RotorZeroPosFilter2   =   0;
                gRotorZeroPIReg.Ref = 0;
                gRotorZeroPIReg.PIOut = ROTOR_ZERO_EST_PIOUT_INIT_VAL;
            }

            else if( gPMEstControl.Counter < (uint16_t)(500*15.5f) )//采集转子位置(A相) 持续12s
            {
                gPMEstControl.RotorInitPosEstStep   =   3;
                gPMEstControl.RotorInitPosEst       =   0;
                gPMEstControl.RunRotorInitPosEst    =   1;         //开启零位辨识的电流给定

                g_PwmEnable = 1;                                 //开波

                gPMEstControl.RotorZeroPosFilter2 = 0.9f*gPMEstControl.RotorZeroPosFilter2 + ( 1 - 0.9f )*gRotorTrans.Pos;//采集位于A相方向的转子角度
            }
            else
            {
                gPMEstControl.RotorInitPosEstStep = 0;

                gPMEstControl.RotorZeroPosFilter4 = gPMEstControl.RotorZeroPosFilter2;
                gIPMPos.RotorZero = -0.5f*( gPMEstControl.RotorZeroPosFilter2 + gPMEstControl.RotorZeroPosFilter4 );       //零位取负，用于计算消除角度偏差

                g_PwmEnable = 0;                       //开波
                DisableDrive();

                gPMEstControl.RunRotorInitPosEst = 0;

                if ( 1 == gPMEstControl.RotorZeroEstOnly )
                {
                	s_PMEstStep = 9;     //仅辨识零位跳到9结束

    //                OutputString("AARunPMEST9");
                }
                else
                {
                	s_PMEstStep = 3;    //开始执行定子电阻辨识
                }

                gPMEstControl.Counter = 0;  //计数清零

                if( fabs( gPMEstControl.RotorZeroPosFilter2 - gPMEstControl.RotorZeroPosFilter1 ) < Value_Pi3)  //B或C相方向的转子角度与A相差小于Pi/3报辨识失败，(正常相差值在2PI/3附件)
                {
                	gFault3rd.bit.ErrorParamEst = 1;
                	G_ErrStep = 1;
//                    gMainStatus.ErrFlag.bit.IPosEstErrFlag = 1;
                }

                if( 0 == s_u16IPosSwitch )   //用于零位辨识第二步骤，采集B或C相转子角度的选择
                {
                    s_u16IPosSwitch = 1;
                }
                else
                {
                    s_u16IPosSwitch = 0;
                }


    //            OutputStrAndI32("AAIPOS1:", gPMEstControl.RotorZeroPosFilter1*100);
    //            OutputStrAndI32("AAIPOS2:", gPMEstControl.RotorZeroPosFilter2*100);
    //            OutputStrAndI32("AAIPOS3:", gPMEstControl.RotorZeroPosFilter3*100);
    //            OutputStrAndI32("AAIPOS4:", gPMEstControl.RotorZeroPosFilter4*100);
            }

            break;

        case 3:
            // wait stop 等待
            if( fabs(gRotorSpeed.SpeedApply) < 1 )
            {
                if( ++gPMEstControl.Counter > 500*2 ) //清零，2s后开始定子电阻
                {
                	s_PMEstStep     =   4;

                    gPMEstControl.Counter   =   0;
                    gPMEstControl.Ires1     =   0;
                    gPMEstControl.Ires2     =   0;
                    gPMEstControl.RsEstStep =   0;   //定子电阻辨识步骤

                    gRotorZeroPIReg.Ref     =   0;
                    gRotorZeroPIReg.PIOut   =   ROTOR_ZERO_EST_PIOUT_INIT_VAL;
                }
            }
            else
            {
                gPMEstControl.Counter = 0;
            }
            break;

        case 4:
            // Rs Est
            gPMEstControl.RunRsEst = 1;         //开启电子电阻的电流给定
            ++gPMEstControl.Counter;
            if( gPMEstControl.Counter < (uint16_t)(500*2.0f) ) //开波持续2s
            {
                g_PwmEnable = 1;

                gPMEstControl.RsEstStep = 1;//步骤1
            }
            else if( gPMEstControl.Counter < (uint16_t)(500*2.5f) )//关波0.5s
            {
            	g_PwmEnable = 0;
                DisableDrive();

                gPMEstControl.RsEstStep =   2;

                gRotorZeroPIReg.Ref     =   0;
                gRotorZeroPIReg.PIOut   =   ROTOR_ZERO_EST_PIOUT_INIT_VAL;

            }
            else if( gPMEstControl.Counter < (uint16_t)(500*4.5f) ) //开波2s
            {
                gPMEstControl.RsEstStep = 3;

                g_PwmEnable = 1;
            }
            else
            {
            	g_PwmEnable = 0;
                DisableDrive();

                gPMEstControl.RunRsEst  = 0;
                gPMEstControl.RsEstStep = 0;

                if ( gPMEstControl.Rs1Cnt >= 16000 && gPMEstControl.Rs2Cnt >= 16000 )   //15000计数后，计算平均电压和平均电流, 稳态：us =Rs * is
                {
                    gPMEstControl.Ures1 = gUDC.uDCBigFilter*gPMEstControl.Ures1Sum/ ( gPMEstControl.Rs1Cnt - 15000 );
                    gPMEstControl.Ires1 = gPMEstControl.Ires1Sum/ ( gPMEstControl.Rs1Cnt - 15000 );
                    gPMEstControl.Ures2 = gUDC.uDCBigFilter*gPMEstControl.Ures2Sum/ ( gPMEstControl.Rs2Cnt - 15000 );
                    gPMEstControl.Ires2 = gPMEstControl.Ires2Sum/ ( gPMEstControl.Rs2Cnt - 15000 );
                }

                if( 0 == gPMEstControl.Ires2 - gPMEstControl.Ires1 )
                {
                    gPMEstControl.Rs = 0.003f;//0;

                    gFault3rd.bit.ErrorParamEst = 1;//错误
                    G_ErrStep = 2;
//                    gMainStatus.ErrFlag.bit.RsEstErrFlag = 1;
                }
                else
                {
                    gPMEstControl.Rs = Cnst_2div3*( gPMEstControl.Ures2 - gPMEstControl.Ures1 )/( gPMEstControl.Ires2 - gPMEstControl.Ires1 );
                }

    //            OutputStrAndI32Ex("AARs1:", gPMEstControl.Ures1*100, gPMEstControl.Ires1*100);
    //            OutputStrAndI32Ex("AARs2:", gPMEstControl.Ures2*100, gPMEstControl.Ires2*100);
    //            OutputStrAndI32Ex("AARs:", ( gPMEstControl.Ures2 - gPMEstControl.Ures1 )*10000, gPMEstControl.Rs*10000);

                if ( gPMEstControl.Rs <= 0.001f )
                {
                    gPMEstControl.Rs = 0.003f;

                    gFault3rd.bit.ErrorParamEst = 1;
                    G_ErrStep = 3;
//                    gMainStatus.ErrFlag.bit.RsEstErrFlag = 1;
                }

                gPMEstControl.Counter = 0;
                s_PMEstStep = 5;


            }


            break;

        case 5:
            // wait stop
            if( fabs(gRotorSpeed.SpeedApply) < 1 )  // below 1HZ
            {
                if( ++gPMEstControl.Counter > 500*2 )
                {
                    gPMEstControl.Counter = 0;
                    s_PMEstStep = 6;
                    gPhase.IMPhase = 0;
                    gUMTSet.M = 0.01f;
                }
            }
            else
            {
                gPMEstControl.Counter = 0;
            }
            break;

        case 6:
            // Ld&Lq Est D轴电感和Q轴电感辨识
            ++gPMEstControl.Counter;
            if( gPMEstControl.Counter < 500*2.0f )
            {
                gPMEstControl.RunLdqEst = 1;

                g_PwmEnable = 1;
            }
            else
            {
            	g_PwmEnable = 0;
                DisableDrive();

                gPMEstControl.RunLdqEst = 0;

                s_PMEstStep = 7;

                gPMEstControl.Counter = 0;

                if( 0 == gPMEstControl.IpFilter + gPMEstControl.InFilter )
                {
                    gPMEstControl.Ld = 0;

                    gFault3rd.bit.ErrorParamEst = 1;
                    G_ErrStep = 4;
//                    gMainStatus.ErrFlag.bit.LdEstErrFlag = 1;
                }
                else
                    gPMEstControl.Ld = Cnst_1divsqrt3*gPMEstControl.VInject*g_SampleRealValue.f32Vdc/(gPMEstControl.LdqEstWi*(gPMEstControl.IpFilter + gPMEstControl.InFilter));

                if( 0 == gPMEstControl.IpFilter - gPMEstControl.InFilter )
                {
                    gPMEstControl.Lq = 0;

                    gFault3rd.bit.ErrorParamEst = 1;
                    G_ErrStep = 5;
//                    gMainStatus.ErrFlag.bit.LqEstErrFlag = 1;
                }
                else
                {
                    gPMEstControl.Lq = Cnst_1divsqrt3*gPMEstControl.VInject*g_SampleRealValue.f32Vdc/(gPMEstControl.LdqEstWi*(gPMEstControl.IpFilter - gPMEstControl.InFilter));
                }

    //            OutputStrAndI32("AALd:", gPMEstControl.Ld*1000000);
    //            OutputStrAndI32("AALq:", gPMEstControl.Lq*1000000);

            }
            break;

        case 7:
            // wait stop
            if( fabs(gRotorSpeed.SpeedApply) < 5 )  // below 5HZ
            {
                if( ++gPMEstControl.Counter > 500*2 )
                {
                	s_PMEstStep = 8;

                    gPMEstControl.Counter           =   0;
                    gPMEstControl.RunBemfEstStep    =   0;
                    gPMEstControl.BemfEstFaultCnt   =   0;
                    gPMEstControl.BemfEstFaultCnt1  =   0;

                    gPMEstControl.RotorZeroCompensateIqRef = 0;
                }
            }
            else
            {
                gPMEstControl.Counter = 0;
            }
            break;

        case 8:
            // Psi_f Est 反电势常数辨识
            if( ++gPMEstControl.Counter < 500*7.0f )
            {
                gPMEstControl.RunBemfEst = 1;

                if ( gPMEstControl.Counter < (uint16_t)(500*1.5f) )
                {
                    gPMEstControl.RunBemfEstStep = 1;
                }
                else if ( gPMEstControl.Counter >= (uint16_t)(500*1.5f) && gPMEstControl.Counter < (uint16_t)(500*3.0f) )
                {
                    gPMEstControl.RunBemfEstStep = 2;
                }
                else if ( gPMEstControl.Counter == (uint16_t)(500*3.0f) )
                {
                    gPMEstControl.RunBemfEstStep = 3;
                }
                else if ( gPMEstControl.Counter > (uint16_t)(500*3.0f) && gPMEstControl.Counter < (uint16_t)(500*4.5f) )
                {
                    gPMEstControl.RunBemfEstStep = 4;
                }
                else if ( gPMEstControl.Counter >= (uint16_t)(500*4.5f) && gPMEstControl.Counter < (uint16_t)(500*6.0f) )
                {
                    gPMEstControl.RunBemfEstStep = 5;
                }
                else if ( gPMEstControl.Counter == (uint16_t)(500*6.0f) )
                {
                    gPMEstControl.RunBemfEstStep = 6;
                }
                else
                {
                    gPMEstControl.RunBemfEstStep = 7;
                }

                if ( gPMEstControl.BemfEstSpdRef < gPMEstControl.BemfEstSpdRefTarget )  //gPMEstControl.BemfEstSpdRef
                {
                    gPMEstControl.BemfEstSpdRef += 1;
                }
                else
                {
                    gPMEstControl.BemfEstSpdRef = gPMEstControl.BemfEstSpdRefTarget;
                }

                if( fabs( gRotorSpeed.SpeedApply ) < 5 )
                {
                    ++gPMEstControl.BemfEstFaultCnt;

                    if( gPMEstControl.BemfEstFaultCnt > 750 )
                    {
                    	gFault3rd.bit.ErrorParamEst = 1;
                    	G_ErrStep = 6;
//                        gMainStatus.ErrFlag.bit.BemfEstErrFlag = 1;
                    }
                }

                if( fabs( gRotorSpeed.SpeedApply ) > gPMEstControl.BemfEstSpdRefTarget*1.5f )
                {
                    ++gPMEstControl.BemfEstFaultCnt1;

                    if( gPMEstControl.BemfEstFaultCnt1 > 50 )
                    {
                    	gFault3rd.bit.ErrorParamEst = 1;
                    	G_ErrStep = 7;
//                        gMainStatus.ErrFlag.bit.BemfEstErrFlag = 1;
                    }
                }

                VCSpeedControl();

                g_PwmEnable = 1;
            }
            else
            {
                float Ts = 0;
                float f32Temp = 0;

                g_PwmEnable = 0;
                DisableDrive();

                gPMEstControl.RunBemfEst = 0;

                s_PMEstStep = 10;

                gPMEstControl.Counter = 0;

                if ( gPMEstControl.Rs > 0 && gPMEstControl.Ld > 0 && gPMEstControl.Lq > 0 )
                {
                    Ts = 1.0f/gSynPWM.FcApply;
                    gPMEstControl.CurrIdDecoupleDiv1 = 1.0f/( gPMEstControl.Rs*Ts + 2*gPMEstControl.Lq );
                    gPMEstControl.CurrIdDecoupleDiv2 = Ts*gPMEstControl.CurrIdDecoupleDiv1;
                    gPMEstControl.CurrIqDecoupleDiv1 = 1.0f/( gPMEstControl.Rs*Ts + 2*gPMEstControl.Ld );
                    gPMEstControl.CurrIqDecoupleDiv2 = Ts*gPMEstControl.CurrIqDecoupleDiv1;
                }

                g_u16ParamsIdentify2Func[0]     =   gPMEstControl.Ld*1000000;//gPMEstControl.u16Ld;
                g_u16ParamsIdentify2Func[1]     =   gPMEstControl.Lq*1000000;//gPMEstControl.u16Lq;
                g_u16ParamsIdentify2Func[2]     =   gPMEstControl.Rs*1000;//gPMEstControl.u16Rs;

                if( gPMEstControl.RotorZeroCompensateEn )
                {
                    f32Temp = -gPMEstControl.RotorZeroCompensateVal;
                    if ( f32Temp < 0 )
                    {
                        f32Temp += Value_2Pi;
                    }

                    g_u16ParamsIdentify2Func[3] = f32Temp*1000;
    //                OutputStrAndI32("AAID3:", g_u16ParamsIdentify2Func[3]);
                }
                else
                {
                    f32Temp = 0;
                }

                g_u16ParamsIdentify2Func[4]     =   gPMEstControl.Bemf*1000;//gPMEstControl.u16Bemf;
                g_u16ParamsIdentify2Func[10]    =   0.5f*( gPMEstControl.RotorZeroPosFilter2 + gPMEstControl.RotorZeroPosFilter4 )*1000;

                gu16MotorCtrlTuneStatus = TUNE_STEP_END;

    //            OutputStrAndI32("AABemf:", gPMEstControl.Bemf*1000);
    //            OutputStrAndI32("AAPosC:", gPMEstControl.RotorZeroCompensateVal*10000);
            }
            break;

        case 9:
        	g_PwmEnable = 0;
            DisableDrive();

            s_PMEstStep = 10;

            g_u16ParamsIdentify2Func[0]     =   gCommProVar.MotorLd;//funcCode.code.pmsmLd;//gEstPara2Motor[0];  //Ld;
            g_u16ParamsIdentify2Func[1]     =   gCommProVar.MotorLq;//funcCode.code.pmsmLq;//gEstPara2Motor[1];  //Lq;
            g_u16ParamsIdentify2Func[2]     =   gCommProVar.MotorRs;//funcCode.code.pmsmRs;//gEstPara2Motor[2];  //Rs;
            g_u16ParamsIdentify2Func[4]     =   gCommProVar.MotorBemf;//funcCode.code.pmsmBackBMF;//gEstPara2Motor[4];  //Psi_f;
            g_u16ParamsIdentify2Func[10]    =   0.5f*( gPMEstControl.RotorZeroPosFilter2 + gPMEstControl.RotorZeroPosFilter4 )*1000;

            gu16MotorCtrlTuneStatus = TUNE_STEP_END;

    //        OutputString("AAParEIposE");
            break;

        default:

        	g_PwmEnable = 0;
            DisableDrive();
//            DisableSlavePWM();//为什么要延迟下？
            s_PMEstStep = 1;
            gDmdCanRe.ModeReq = 0;                //参数辨识命令清除
            g_RunStatus = STATUS_READY;
            break;
    }

    if (gu16MotorCtrlTuneStatus == TUNE_STEP_END)
    {

        if (savePmEstDataFlag == 0)
        {
            SaveTuneData(); //数据保存
            savePmEstDataFlag = 1;
        }
    }

    if ( ++runPmEstTimeCnt >= TUNE_RUN_OVER_TIME_MAX * 500 ) // 超时进行提醒？
    {
        runPmEstTimeCnt = TUNE_RUN_OVER_TIME_MAX * 500;
//        OutputString("EETuneTimeOut");
//        errorOther = ERROR_TUNE;
//        return;
    }
}

void RunParamEstInIsr( void )
{
    if( gPMEstControl.RunRotorInitPosEst )
    {
        if ( 1 == gPMEstControl.RotorInitPosEstStep )
        {
            if( gRotorZeroPIReg.Ref < gPMEstControl.RotorInitPosEstMaxCurr*gMotorInfo.Current)
                gRotorZeroPIReg.Ref += 0.01f;
            else
                gRotorZeroPIReg.Ref = gPMEstControl.RotorInitPosEstMaxCurr*gMotorInfo.Current;

            if( 0 == s_u16IPosSwitch )
            {
                gRotorZeroPIReg.Fb = g_SampleRealValue.f32Ib;
            }
            else
            {
                gRotorZeroPIReg.Fb = g_SampleRealValue.f32Ic;
            }


            PIRegulator(&gRotorZeroPIReg);

            gPMEstControl.RotorInitPosEstDuty = gRotorZeroPIReg.PIOut;

            gPWM.U = gPWM.gPWMPrdApply;

            if( 0 == s_u16IPosSwitch )
            {
                gPWM.V = ( 1 - gPMEstControl.RotorInitPosEstDuty )*gPWM.gPWMPrdApply;
                gPWM.W = gPWM.gPWMPrdApply;
            }
            else
            {
                gPWM.V = gPWM.gPWMPrdApply;
                gPWM.W = ( 1 - gPMEstControl.RotorInitPosEstDuty )*gPWM.gPWMPrdApply;
            }
        }
        else if ( 3 == gPMEstControl.RotorInitPosEstStep )
        {
            if( gRotorZeroPIReg.Ref < gPMEstControl.RotorInitPosEstMaxCurr*gMotorInfo.Current )
                gRotorZeroPIReg.Ref += 0.1f*gPMEstControl.RotorInitPosEstMaxCurr*gMotorInfo.Current*1.0e-4;
            else
                gRotorZeroPIReg.Ref = gPMEstControl.RotorInitPosEstMaxCurr*gMotorInfo.Current;

            gRotorZeroPIReg.Fb  = g_SampleRealValue.f32Ia;
            PIRegulator(&gRotorZeroPIReg);

            gPMEstControl.RotorInitPosEstDuty = gRotorZeroPIReg.PIOut;

            gPWM.U = ( 1 - gPMEstControl.RotorInitPosEstDuty )*gPWM.gPWMPrdApply;
            gPWM.V = gPWM.gPWMPrdApply;
            gPWM.W = gPWM.gPWMPrdApply;
        }
        else
        {
            gPWM.U = gPWM.gPWMPrdApply;
            gPWM.V = gPWM.gPWMPrdApply;
            gPWM.W = gPWM.gPWMPrdApply;

            gRotorZeroPIReg.PIOut = ROTOR_ZERO_EST_PIOUT_INIT_VAL;
            gRotorZeroPIReg.Err = 0;
        }

        SendPWM();
    }
    else if ( gPMEstControl.RunRsEst )
    {
        if( 1 == gPMEstControl.RsEstStep )
        {
            ++gPMEstControl.Rs1Cnt;

            if( gRotorZeroPIReg.Ref < gPMEstControl.RsEstCurr1*gMotorInfo.Current )
                gRotorZeroPIReg.Ref += 0.02f;//0.01f;
            else
                gRotorZeroPIReg.Ref = gPMEstControl.RsEstCurr1*gMotorInfo.Current;

            gRotorZeroPIReg.Fb = g_SampleRealValue.f32Ia;
            PIRegulator(&gRotorZeroPIReg);

            gPMEstControl.RsEstDuty = gRotorZeroPIReg.PIOut;

            gPWM.U = ( 1 - gPMEstControl.RsEstDuty )*gPWM.gPWMPrdApply;
            gPWM.V = gPWM.gPWMPrdApply;
            gPWM.W = gPWM.gPWMPrdApply;

            if ( gPMEstControl.Rs1Cnt >= 15000 )
            {
                gPMEstControl.Ures1Sum += gPMEstControl.RsEstDuty;//*g_SampleRealValue.f32Vdc;
                gPMEstControl.Ires1Sum += g_SampleRealValue.f32Ia;
            }
        }
        else if( 3 == gPMEstControl.RsEstStep )
        {
            ++gPMEstControl.Rs2Cnt;

            if( gRotorZeroPIReg.Ref < gPMEstControl.RsEstCurr2*gMotorInfo.Current )
                gRotorZeroPIReg.Ref += 0.02f;//0.01f;
            else
                gRotorZeroPIReg.Ref = gPMEstControl.RsEstCurr2*gMotorInfo.Current;

            gRotorZeroPIReg.Fb = g_SampleRealValue.f32Ia;
            PIRegulator(&gRotorZeroPIReg);

            gPMEstControl.RsEstDuty = gRotorZeroPIReg.PIOut;

            gPWM.U = ( 1 - gPMEstControl.RsEstDuty )*gPWM.gPWMPrdApply;
            gPWM.V = gPWM.gPWMPrdApply;
            gPWM.W = gPWM.gPWMPrdApply;

            if ( gPMEstControl.Rs2Cnt >= 15000 )
            {
                gPMEstControl.Ures2Sum += gPMEstControl.RsEstDuty;//*g_SampleRealValue.f32Vdc;
                gPMEstControl.Ires2Sum += g_SampleRealValue.f32Ia;
            }
        }
        else
        {
            gPWM.U = gPWM.gPWMPrdApply;
            gPWM.V = gPWM.gPWMPrdApply;
            gPWM.W = gPWM.gPWMPrdApply;

            gRotorZeroPIReg.PIOut = ROTOR_ZERO_EST_PIOUT_INIT_VAL;
            gRotorZeroPIReg.Err = 0;
        }

        SendPWM();
    }
    else if( gPMEstControl.RunLdqEst )
    {
        if( gPMEstControl.VInject < 0.5f && gPMEstControl.IpFilter < 0.50f*gMotorInfo.Current )
            gPMEstControl.VInject += 0.0005f;

        if( gPMEstControl.VInject > 0.5f ) gPMEstControl.VInject = 0.5f;

        gUMTSet.M = gPMEstControl.VInject*g_SampleRealValue.f32Vdc;
        gUMTSet.T = 0;

        OutPutPWMVF();

        PNSeqExtrack();

        CalOutputPhase();

    }
    else if( gPMEstControl.RunBemfEst )
    {
        CalOutputPhase();
        TransformCurrent();
        VCCsrControlEx();
        OutPutPWMVF();

        if( 2 == gPMEstControl.RunBemfEstStep )
        {
            gPMEstControl.BemfEstVq1Sum += gUMTSet.T*Cnst_1divsqrt3;
            gPMEstControl.BemfEstId1Sum += gIMT.M;
            gPMEstControl.BemfEstCnt1++;

            gPMEstControl.RotorZeroCompensateIqSum += gIMT.T;
            gPMEstControl.RotorZeroCompensateIqCnt++;
        }
        else if( 3 == gPMEstControl.RunBemfEstStep )
        {
            if( 0 != gPMEstControl.BemfEstCnt1 )
            {
                gPMEstControl.BemfEstVq1 = gPMEstControl.BemfEstVq1Sum/gPMEstControl.BemfEstCnt1;
                gPMEstControl.BemfEstId1 = gPMEstControl.BemfEstId1Sum/gPMEstControl.BemfEstCnt1;
            }
            else
            {
                gPMEstControl.BemfEstVq1 = 0;
                gPMEstControl.BemfEstId1 = 0;
            }

            if( 0 != gPMEstControl.RotorZeroCompensateIqCnt )
            {
                gPMEstControl.RotorZeroCompensateIqRef = gPMEstControl.RotorZeroCompensateIqSum/gPMEstControl.RotorZeroCompensateIqCnt;
            }
            else
            {
                gPMEstControl.RotorZeroCompensateIqRef = 0;
            }

        }
        else if( 5 == gPMEstControl.RunBemfEstStep )
        {
            gPMEstControl.BemfEstVq2Sum += gUMTSet.T*Cnst_1divsqrt3;
            gPMEstControl.BemfEstId2Sum += gIMT.M;
            gPMEstControl.BemfEstCnt2++;

            gPMEstControl.RotorZeroCompensateValSum += gPMEstControl.RotorZeroCompensatePIReg.PIOut;
            gPMEstControl.RotorZeroCompensateValCnt++;
        }
        else if( 6 == gPMEstControl.RunBemfEstStep )
        {
            if( 0 != gPMEstControl.BemfEstCnt2 )
            {
                gPMEstControl.BemfEstVq2 = gPMEstControl.BemfEstVq2Sum/gPMEstControl.BemfEstCnt2;
                gPMEstControl.BemfEstId2 = gPMEstControl.BemfEstId2Sum/gPMEstControl.BemfEstCnt2;
            }
            else
            {
                gPMEstControl.BemfEstVq2 = 0;
                gPMEstControl.BemfEstId2 = 0;
            }

            if( 0 != gPMEstControl.BemfEstId2 - gPMEstControl.BemfEstId1 && 0 != gPMEstControl.BemfEstSpdRef )
            {
                gPMEstControl.BemfEstWi =   Value_2Pi*gPMEstControl.BemfEstSpdRef;
                gPMEstControl.Bemf      =   ( gPMEstControl.BemfEstVq1*gPMEstControl.BemfEstId2 - gPMEstControl.BemfEstVq2*gPMEstControl.BemfEstId1 );
                gPMEstControl.Bemf      /=  ( ( gPMEstControl.BemfEstId2 - gPMEstControl.BemfEstId1 )*gPMEstControl.BemfEstWi );
            }
            else
            {
                gPMEstControl.Bemf = 0;
            }

            if( 0 != gPMEstControl.RotorZeroCompensateValCnt )
            {
                gPMEstControl.RotorZeroCompensateVal = gPMEstControl.RotorZeroCompensateValSum/gPMEstControl.RotorZeroCompensateValCnt;
            }
            else
            {
                gPMEstControl.RotorZeroCompensateVal = 0;
            }

        }

        if( gPMEstControl.RunBemfEstStep >= 4 )
        {
            gPMEstControl.RotorZeroCompensatePIReg.Ref  =   gPMEstControl.RotorZeroCompensateIqRef;
            gPMEstControl.RotorZeroCompensatePIReg.Fb   =   gIMT.T;
            PIRegulator(&(gPMEstControl.RotorZeroCompensatePIReg));

        }
    }
    else
    {
    	g_PwmEnable = 0;
        DisableDrive();
    }
}


static void SaveTuneData(void)
{
    int16_t i;
    uint16_t index;               // 需要保存的参数index //funcCode.code.pmsmLd传递给gEstPara2Motor[0]，程序使用gEstPara2Motor(扩大了1000倍)进行计算零位等

//    funcCode.code.pmsmLd            =   g_u16ParamsIdentify2Func[0];
//    funcCode.code.pmsmLq            =   g_u16ParamsIdentify2Func[1];    // 29  C4-01    Q Axis L
//    funcCode.code.pmsmRs            =   g_u16ParamsIdentify2Func[2];    // 30  C4-02    Stator Resistor
//    funcCode.code.pmsmZeroPosCompVal=   g_u16ParamsIdentify2Func[3];    // 31  C4-03
//    funcCode.code.pmsmBackBMF       =   g_u16ParamsIdentify2Func[4];    // 32  C4-04   反电动势
//    funcCode.code.disconnectionTime =   g_u16ParamsIdentify2Func[5];    // 33  C4-05   断线检测时间
//    funcCode.code.pmsmKPd           =   g_u16ParamsIdentify2Func[6];    // 34  C4-06    D Axis KP
//    funcCode.code.pmsmKId           =   g_u16ParamsIdentify2Func[7];    // 35  C4-07    D Axis KI
//    funcCode.code.pmsmKPq           =   g_u16ParamsIdentify2Func[8];    // 36  C4-08    Q Axis KP
//    funcCode.code.pmsmKIq           =   g_u16ParamsIdentify2Func[9];    // 37  C4-09    Q Axis KI
//    funcCode.code.pmsmZeroPosition  =   g_u16ParamsIdentify2Func[10];   // 38  C4-10   旋变零点位置
//    funcCode.code.pmsmDirection     =   g_u16ParamsIdentify2Func[11];   // 39  C4-11   旋变方向

//    OutputStrAndI32("AAEst0:", funcCode.code.pmsmLd);
//    OutputStrAndI32("AAEst1:", funcCode.code.pmsmLq);
//    OutputStrAndI32("AAEst2:", funcCode.code.pmsmBackBMF);
//    OutputStrAndI32("AAEst3:", funcCode.code.pmsmZeroPosition);

    for (i = 0; i <= sizeof(pmsmParaIndex) - 1; i++)
    {
        index = *(pmsmParaIndex + i);   // 调谐保存参数从同步机D轴电感开始
        SaveOneFuncCode(index);
    }

}

static void PNSeqExtrack( void )
{
    float f32temp = 0;

    gPNSeqExtract.Alpha =   Cnst_2div3*g_SampleRealValue.f32Ia - Cnst_1div3*g_SampleRealValue.f32Ib - Cnst_1div3*g_SampleRealValue.f32Ic;
    gPNSeqExtract.Beta  =   ( g_SampleRealValue.f32Ib - g_SampleRealValue.f32Ic )*Cnst_1divsqrt3;

    f32temp = (gPNSeqExtract.Alpha - gPNSeqExtract.Alpha1)*sqrt2 - gPNSeqExtract.Alpha2;
    gPNSeqExtract.Alpha1 += (2*Value_Pi*gPMEstControl.LdqEstFreqInject/gSynPWM.FcApply*f32temp);
    gPNSeqExtract.Alpha2 += (2*Value_Pi*gPMEstControl.LdqEstFreqInject/gSynPWM.FcApply*gPNSeqExtract.Alpha1);

    f32temp = (gPNSeqExtract.Beta - gPNSeqExtract.Beta1)*sqrt2 - gPNSeqExtract.Beta2;
    gPNSeqExtract.Beta1 += (2*Value_Pi*gPMEstControl.LdqEstFreqInject/gSynPWM.FcApply*f32temp);
    gPNSeqExtract.Beta2 += (2*Value_Pi*gPMEstControl.LdqEstFreqInject/gSynPWM.FcApply*gPNSeqExtract.Beta1);

    gPNSeqExtract.AlphaP = 0.5 * (gPNSeqExtract.Alpha1 - gPNSeqExtract.Beta2);
    gPNSeqExtract.BetaP = 0.5 * (gPNSeqExtract.Beta1 + gPNSeqExtract.Alpha2);

    gPNSeqExtract.AlphaN = 0.5 * (gPNSeqExtract.Alpha1 + gPNSeqExtract.Beta2);
    gPNSeqExtract.BetaN = 0.5 * (gPNSeqExtract.Beta1 - gPNSeqExtract.Alpha2);

//  gPMEstControl.IpFilter = 0.98f*gPMEstControl.IpFilter + 0.02f*sqrt(gPNSeqExtract.AlphaP*gPNSeqExtract.AlphaP + gPNSeqExtract.BetaP*gPNSeqExtract.BetaP);
//  gPMEstControl.InFilter = 0.98f*gPMEstControl.InFilter + 0.02f*sqrt(gPNSeqExtract.AlphaN*gPNSeqExtract.AlphaN + gPNSeqExtract.BetaN*gPNSeqExtract.BetaN);
    gPMEstControl.IpFilter = 0.999f*gPMEstControl.IpFilter + ( 1 - 0.999f )*sqrt(gPNSeqExtract.AlphaP*gPNSeqExtract.AlphaP + gPNSeqExtract.BetaP*gPNSeqExtract.BetaP);
    gPMEstControl.InFilter = 0.999f*gPMEstControl.InFilter + ( 1 - 0.999f )*sqrt(gPNSeqExtract.AlphaN*gPNSeqExtract.AlphaN + gPNSeqExtract.BetaN*gPNSeqExtract.BetaN);
}
