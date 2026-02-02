/*
 * MotorVar.c
 *
 *  Created on: 2024年12月4日
 *      Author: pc
 */
#include "f_public.h"
#include "MotorCalibration.h"

extern float gIdTSetM;
extern float gIqTSetM;
//#pragma CODE_SECTION(sqrt,                  "ramfuncs")
//#pragma CODE_SECTION(sincos,                "ramfuncs")
//#pragma CODE_SECTION(CalOutputPhase,        "ramfuncs")
//#pragma CODE_SECTION(VCCsrControlEx,            "ramfuncs")
//#pragma CODE_SECTION(ModulateIndexSearch,       "ramfuncs")
//#pragma CODE_SECTION(CalcModifiedModulateIndex, "ramfuncs")
//#pragma CODE_SECTION(RoundArcAngle,         "ramfuncs")
//#pragma CODE_SECTION(TransformCurrent,      "ramfuncs")
//#pragma CODE_SECTION(CalcPowerAndTorque,    "ramfuncs")
//#pragma CODE_SECTION(CalcLineCurr,          "ramfuncs")

IUVW_SAMPLING_STRUCT gCurSamp    = {0, 0, 0, 0, 0, 0, 0, 0};
UVW_STRUCT gIUVW                 = {0, 0, 0};      //定子三相坐标轴电流
RotorSpeedStruct gRotorSpeed     = {0, 0, 0};
MT_STRUCT  gIMTSet               = {0, 0, 0, 0, 0, 0};   //MT轴系下的设定电流
MT_STRUCT  gIMTSetApply          = {0, 0, 0, 0, 0, 0};   //MT轴系下的电流指令值
MT_STRUCT  gIMT                  = {0, 0, 0, 0, 0, 0};  //MT轴系下的电流
MT_STRUCT  gUMTSet               = {0, 0, 0, 0, 0, 0};   //MT轴系下的设定电压
ANGLE_STRUCT gPhase              = {0, 0, 0};     //角度结构
SYN_PWM_STRUCT gSynPWM           = {0, 0, 0, 0};
ALPHABETA_STRUCT  gIAlphBeta     = {0, 0}; //定子两相坐标轴电流
UALPHABETA_STRUCT gUAlphBeta     = {0, 0};
CURR_DECOUPLE_STRUCT gIdDecouple = {0, 0, 0, 0, 0, 0, 0, 0, 0};
CURR_DECOUPLE_STRUCT gIqDecouple = {0, 0, 0, 0, 0, 0, 0, 0, 0};
PIReg gIMAcr                     = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
PIReg gITAcr                     = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
ACR_PICHG_STRUCT gAcr            = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

LINE_CURRENT_STRUCT gLineCur     = {0, 0, 0};
MOTOR_POWER_TORQUE gPowerTrq     = {0, 0, 0};

uint16_t gu16OverModuleFlag        = 0; //过调标志
float gf32IdRefFilter          = 0;
float gf32IqRefFilter          = 0;
static float ModulateIndex     = 1;
static float ModulateIndexModify = 1;
static float s_MaxVolt         = 0;
static float s_MaxVd           = 0;
static float s_MaxOutVolt      = 0;
static float s_UAmp            = 0;
static float s_W0              = 0;
static float s_f32IdComp       = 1;
static float s_f32IqComp       = 1;

void VCCsrControlEx(void);
void CalOutputPhase(void);
void TransformCurrent(void);
float RoundArcAngle(float ArcAngle);
void CalcActTorque( void );
void CalcLineCurr(void);
void CalcCsrDecoupleVal( void );
void SelectAcrPar(void);
void ResetCsrPar(void);
static void ModulateIndexSearch( float M, int16_t *i0, int16_t *i1 );
static float CalcModifiedModulateIndex( float M );

#define MODULATE_LIMIT_INDEX 150
#define MAX_OVR_MOD_LEN    201
static float OVR_MOD[MAX_OVR_MOD_LEN] =
{
    1,1.0043642,1.0082038,1.0117043,1.0149323,1.0179264,1.0207129,1.0233112,1.0257364,1.0280007,
    1.0301143,1.0320858,1.0339226,1.0356311,1.0372173,1.0386862,1.0400425,1.0412903,1.0424337,1.0434759,
    1.0444205,1.0452702,1.0460281,1.0466967,1.0472784,1.0477756,1.0481904,1.0485249,1.0487811,1.0489607,
    1.0490656,1.0490975,1.0491321,1.0492317,1.04939,1.0496015,1.0498612,1.0501646,1.0505074,1.0508859,
    1.0512966,1.0517364,1.0522023,1.0526917,1.0532022,1.0537315,1.0542776,1.0548386,1.0554128,1.0559986,
    1.0565944,1.057199,1.0578111,1.0584295,1.0590532,1.0596811,1.0603124,1.0609462,1.0615817,1.0622183,
    1.0628552,1.0634919,1.0641278,1.0647623,1.065395,1.0660254,1.0666531,1.0672778,1.067899,1.0685165,
    1.06913,1.0697392,1.0703439,1.0709437,1.0715386,1.0721283,1.0727127,1.0732915,1.0738647,1.0744321,
    1.0749936,1.0755491,1.0760985,1.0766417,1.0771787,1.0777094,1.0782337,1.0787515,1.0792629,1.0797678,
    1.0802662,1.080758,1.0812433,1.081722,1.0821942,1.0826597,1.0831187,1.0835711,1.084017,1.0844563,
    1.0848892,1.0853155,1.0857353,1.0861487,1.0865557,1.0869562,1.0873505,1.0877384,1.08812,1.0884953,
    1.0888645,1.0892275,1.0895843,1.0899351,1.0902799,1.0906186,1.0909514,1.0912784,1.0915994,1.0919147,
    1.0922242,1.092528,1.0928262,1.0931187,1.0934057,1.0936872,1.0939633,1.0942339,1.0944992,1.0947592,
    1.095014,1.0952635,1.095508,1.0957473,1.0959816,1.0962109,1.0964353,1.0966548,1.0968695,1.0970794,
    1.0972845,1.097485,1.0976808,1.0978721,1.0980588,1.098241,1.0984188,1.0985923,1.0987614,1.0989262,
    1.0990867,1.0992431,1.0993953,1.0995435,1.0996875,1.0998276,1.0999637,1.1000959,1.1002243,1.1003488,
    1.1004695,1.1005865,1.1006998,1.1008094,1.1009155,1.1010179,1.1011169,1.1012123,1.1013044,1.101393,
    1.1014783,1.1015602,1.1016389,1.1017143,1.1017865,1.1018556,1.1019215,1.1019843,1.1020441,1.1021009,
    1.1021547,1.1022055,1.1022535,1.1022986,1.1023408,1.1023802,1.1024169,1.1024508,1.102482,1.1025106,
    1.1025365,1.1025599,1.1025806,1.1025988,1.1026145,1.1026278,1.1026385,1.1026469,1.1026529,1.1026565,
    1.1026578
};

/*************************************************************
    电角度计算
*************************************************************/
uint16_t G_dgghdhw = 0;
void CalOutputPhase(void)
{
	static float32 s_IMPhase = 0;
	static uint16_t s_Cnt = 0;

	s_Cnt++;
	if (s_Cnt >= 4000)
	{
		G_dgghdhw++;
		s_Cnt = 0;
	}

//	s_IMPhase += 0.005*Value_2Pi; //50HZ
//
////	s_IMPhase = 0.03*Value_2Pi; //50HZ
//
//	if (s_IMPhase > Value_2Pi )
//	{
//		s_IMPhase -= Value_2Pi;
//	}
//
//
//    gIPMPos.RotorPos = RoundArcAngle(s_IMPhase);
//
//    gPhase.IMPhase = gIPMPos.RotorPos;
//
//    gPhase.ISIN = sinf(gPhase.IMPhase);
//    gPhase.ICOS = cosf(gPhase.IMPhase);


    if( g_RunStatus != STATUS_PARAMEST)
    {
        gIPMPos.RotorPos = gRotorTrans.Pos + gIPMPos.RotorZero + gRotorTrans.PosComp;
        gIPMPos.RotorPos = RoundArcAngle(gIPMPos.RotorPos);

        gPhase.IMPhase = gIPMPos.RotorPos;

        gPhase.ISIN = sinf(gPhase.IMPhase);
        gPhase.ICOS = cosf(gPhase.IMPhase);
//        sincos(gPhase.IMPhase, &(gPhase.ISIN), &(gPhase.ICOS));
    }
    else
    {
        if ( gPMEstControl.RunRotorInitPosEst )
        {
            gPhase.IMPhase = gPMEstControl.RotorInitPosEst;       //pz modify

            gPhase.ISIN = sinf(gPMEstControl.RotorInitPosEst);
            gPhase.ICOS = cosf(gPMEstControl.RotorInitPosEst);
//            sincos(gPMEstControl.RotorInitPosEst, &(gPhase.ISIN), &(gPhase.ICOS));
        }
        else if ( gPMEstControl.RunLdqEst )
        {
            gPMEstControl.LdqEstTheta += gPMEstControl.LdqEstWi/gSynPWM.FcApply;  //gPMEstControl.LdqEstWi 2pi/500
            gPMEstControl.LdqEstTheta = RoundArcAngle(gPMEstControl.LdqEstTheta);
            gIPMPos.RotorPos = gRotorTrans.Pos - 0.5f*( gPMEstControl.RotorZeroPosFilter2 + gPMEstControl.RotorZeroPosFilter4 );
            gPhase.IMPhase = gIPMPos.RotorPos + gPMEstControl.LdqEstTheta;
            gPhase.IMPhase = RoundArcAngle(gPhase.IMPhase);

            gPhase.ISIN = sinf(gPhase.IMPhase);
            gPhase.ICOS = cosf(gPhase.IMPhase);
//            sincos(gPhase.IMPhase, &(gPhase.ISIN), &(gPhase.ICOS));
        }
        else if ( gPMEstControl.RunBemfEst )
        {
            if( gPMEstControl.RunBemfEstStep >= 4 )
            {
                gPhase.IMPhase = gRotorTrans.Pos - 0.5f*( gPMEstControl.RotorZeroPosFilter2 + gPMEstControl.RotorZeroPosFilter4 ) \
                    + gPMEstControl.RotorZeroCompensatePIReg.PIOut + gRotorTrans.PosComp;
            }
            else
            {
                gPhase.IMPhase = gRotorTrans.Pos - 0.5f*( gPMEstControl.RotorZeroPosFilter2 + gPMEstControl.RotorZeroPosFilter4 ) + gRotorTrans.PosComp;
            }

            gPhase.IMPhase = RoundArcAngle(gPhase.IMPhase);

            gPhase.ISIN = sinf(gPhase.IMPhase);
            gPhase.ICOS = cosf(gPhase.IMPhase);
//            sincos(gPhase.IMPhase, &(gPhase.ISIN), &(gPhase.ICOS));
        }
    }
}

/*************************************************************
       电流变换，IdIq反馈
*************************************************************/
void TransformCurrent(void)
{
    float m_PhaseISIN = 0;
    float m_PhaseICOS = 0;
    float m_PRotorPos = 0;
    float m_IMPhase = 0;

    gIUVW.U = gCurSamp.U;
    gIUVW.W = gCurSamp.W;
    gIUVW.V = gCurSamp.V;


    m_PRotorPos = gRotorTrans.Pos + gIPMPos.RotorZero + gRotorTrans.ParkPosComp;
    m_PRotorPos = RoundArcAngle(m_PRotorPos);

    m_PhaseISIN = sinf(m_PRotorPos);
    m_PhaseICOS = cosf(m_PRotorPos);

    //Clark变换
    gIAlphBeta.Alph = ( gIUVW.U*2 - gIUVW.V - gIUVW.W )*Cnst_1div3;
    gIAlphBeta.Beta  = ( gIUVW.V - gIUVW.W )*Cnst_1divsqrt3;

    //Park变换
    gIMT.M = m_PhaseICOS * gIAlphBeta.Alph + m_PhaseISIN * gIAlphBeta.Beta;
    gIMT.T = -m_PhaseISIN * gIAlphBeta.Alph + m_PhaseICOS * gIAlphBeta.Beta;

}

/*************************************************************
       电流环计算，UqUq给定
*************************************************************/
void VCCsrControlEx(void)
{
    float f32Temp = 0;
    float f32AbsIqRef = 0;

    s_MaxVolt = gUDC.uDCBigFilter;//gUDC.uDCFilter;
    s_MaxOutVolt = 1.020f*gUDC.uDCBigFilter;

    gu16OverModuleFlag = 0;

    gIMAcr.MaxPIOut = 1.00f*gUDC.uDCBigFilter;
    gIMAcr.MinPIOut = -gIMAcr.MaxPIOut;

    gITAcr.MaxPIOut = 1.00f*gUDC.uDCBigFilter;
    gITAcr.MinPIOut = -gITAcr.MaxPIOut;

    //标定程序对IdIq的处理暂时屏蔽
    if( 0 == IS_CALIBRATED && 1 == g_TorqueCtlFlag)//1 == gComPar.SubCommand.bit.TorqueCtl
    {
        gIMTSetApply.M = gIMTSet.M;

        if ( gIMTSet.T >= 0 )
        {
            if ( gIMTSetApply.T < gIMTSet.T )
            {
                if( 0 == gPMFwCtrl.StopIncIqByIdRef && 0 == gPMFwCtrl.StopIncIqByVout )
                    gIMTSetApply.T += 0.01f;
            }
            else
            {
                gIMTSetApply.T = gIMTSet.T;
            }
        }
        else
        {
            if ( gIMTSetApply.T > gIMTSet.T )
            {
                if( 0 == gPMFwCtrl.StopIncIqByIdRef && 0 == gPMFwCtrl.StopIncIqByVout )
                    gIMTSetApply.T -= 0.01f;
            }
            else
            {
                gIMTSetApply.T = gIMTSet.T;
            }
        }

    }
    else
    {
        gIMTSetApply.M = gIMTSet.M;
        gIMTSetApply.T = gIMTSet.T;
    }

//    gIMTSetApply.M = gIdTSetM;
//    gIMTSetApply.T = gIqTSetM;

    if ( 0 == IS_CALIBRATED )
    {
        if( 1 == g_TorqueCtlFlag )//1 == gComPar.SubCommand.bit.TorqueCtl
        {
            s_f32IdComp = 1;
            s_f32IqComp = 1;    // just derate iqref
        }
        else
        {
            s_f32IdComp = 1;
            s_f32IqComp = 1;
        }
    }
    else
    {
        if( gIMTSetApply.M < -gPMFwCtrl.IdMax1*0.88f )  //gPMFwCtrl.IdMax1 = 0.0001f*gu16MaxFwIdSetting*gMotorInfo.Current;弱磁电流限制
        {
            s_f32IdComp = 1;
        }
    }

    gIMAcr.Ref = s_f32IdComp*gIMTSetApply.M;
    Saturatefloat( &gIMAcr.Ref, 0, -gPMFwCtrl.IdMax1 );  //标定弱磁限制
    gIMAcr.Fb = gIMT.M;
    PIRegulatorEx(&gIMAcr);

    gITAcr.Ref = s_f32IqComp*gIMTSetApply.T;
    f32AbsIqRef = fabs( gITAcr.Ref );
    gITAcr.Fb = gIMT.T;
    PIRegulatorEx(&gITAcr);

    gUMTSet.M = gIMAcr.PIOut + gPMFwOut.UdComp; //Pi输出加补偿
    gUMTSet.T = gITAcr.PIOut + gPMFwOut.UqComp;

    s_UAmp = sqrt( gUMTSet.M*gUMTSet.M + gUMTSet.T*gUMTSet.T );

    if( s_UAmp > s_MaxVolt && s_MaxVolt > 10 )
    {
        if ( s_UAmp > s_MaxOutVolt )
        {
            s_MaxVd = 0.998f*s_MaxOutVolt;

            if ( gUMTSet.M >= 0 && gUMTSet.M >= s_MaxVd )
            {
                gUMTSet.M = s_MaxVd;
            }
            else if ( gUMTSet.M < 0 && gUMTSet.M < -s_MaxVd )
            {
                gUMTSet.M = -s_MaxVd;
            }

            f32Temp = sqrt( ( s_MaxVd + 15 )*( s_MaxVd + 15 ) - gUMTSet.M*gUMTSet.M );
            if ( gUMTSet.T < -f32Temp )
            {
                gUMTSet.T = -f32Temp;
                s_UAmp = s_MaxVd + 15;
            }
            else if ( gUMTSet.T > f32Temp )
            {
                gUMTSet.T = f32Temp;
                s_UAmp = s_MaxVd + 15;
            }

            gPMFwCtrl.StopIncIqByVout = 1;
            gPMFwCtrl.StopIncIqByVoutCnt = 1000;

            if( s_f32IqComp > 0.5f )
            {
                s_f32IqComp *= 0.997f;
                if ( s_f32IqComp < 0.5f )
                {
                    s_f32IqComp = 0.5f;
                }
            }

            if ( f32AbsIqRef <= 20 )
            {
                if( s_f32IdComp < 1.2f )
                {
                    s_f32IdComp *= 1.003f;
                }

                if( s_f32IdComp > 1.2f )
                {
                    s_f32IdComp = 1.2f;
                }
            }
        }
        else
        {
            if ( gPMFwCtrl.StopIncIqByVoutCnt > 0 )
            {
                --gPMFwCtrl.StopIncIqByVoutCnt;
            }
            else
            {
                gPMFwCtrl.StopIncIqByVout = 0;
            }

            if( s_f32IdComp > 1 )
            {
                s_f32IdComp *= 0.99995f;
                if ( s_f32IdComp < 1 )
                {
                    s_f32IdComp = 1;
                }
            }

            if( s_f32IqComp < 1 )
            {
                s_f32IqComp *= 1.00005f;
                if ( s_f32IqComp > 1 )
                {
                    s_f32IqComp = 1;
                }
            }
        }

        ModulateIndex = s_UAmp/gUDC.uDCBigFilter;

        if( ModulateIndex >= OVR_MOD[ MAX_OVR_MOD_LEN - 1 - MODULATE_LIMIT_INDEX] )
        {
            ModulateIndexModify = 2 - 0.005f*MODULATE_LIMIT_INDEX;
        }
        else
        {
            ModulateIndexModify = CalcModifiedModulateIndex( ModulateIndex );
        }

        f32Temp = ModulateIndexModify/ModulateIndex;
        gUMTSet.M *= f32Temp;
        gUMTSet.T *= f32Temp;

        gu16OverModuleFlag = 1;

    }
    else
    {
        gPMFwCtrl.StopIncIqByVout = 0;
        gPMFwCtrl.StopIncIqByVoutCnt = 0;

        if( s_f32IdComp > 1 )
        {
            s_f32IdComp *= 0.99990f;
            if ( s_f32IdComp < 1 )
            {
                s_f32IdComp = 1;
            }
        }

        if( s_f32IqComp < 1 )
        {
            s_f32IqComp *= 1.0005f;
            if ( s_f32IqComp > 1 )
            {
                s_f32IqComp = 1;
            }
        }
    }

    gCsrUAmp = s_UAmp;   // used for flux weak 弱磁

    CalDeadBandComp(gIMAcr.Ref, gITAcr.Ref);//死区补偿
}

/*************************************************************
       角度限制2PI
*************************************************************/
float RoundArcAngle(float ArcAngle)
{
    while( ArcAngle > Value_2Pi ) ArcAngle -= Value_2Pi;
    while( ArcAngle < 0 ) ArcAngle += Value_2Pi;

    return ArcAngle;
}

/*************************************************************
       电压UdUq给定补偿
*************************************************************/
void CalcCsrDecoupleVal(void)
{
    if( 0 == gPMEstControl.RunBemfEst && gIdDecouple.LdivRs > 0 && gIqDecouple.LdivRs > 0 )
    {
        s_W0 = 2*Value_Pi*gRotorSpeed.SpeedApply;

        gIqDecouple.a       =   0.30f*s_W0*gIqDecouple.LdivRs;  // 0.30 is not needed in theory
        gIqDecouple.b       =   gIqDecouple.LdivRs;
        gIqDecouple.c       =   gIqDecouple.b/( gIqDecouple.b + gIqDecouple.Ts );
        gIqDecouple.out     =   gIqDecouple.c*gIqDecouple.out + ( 1 - gIqDecouple.c )*gIMAcr.PIOut;

        gIdDecouple.a       =   0.30f*s_W0*gIdDecouple.LdivRs;  // 0.30 is not needed in theory
        gIdDecouple.b       =   gIdDecouple.LdivRs;
        gIdDecouple.c       =   gIdDecouple.b/( gIdDecouple.b + gIdDecouple.Ts );
        gIdDecouple.out     =   gIdDecouple.c*gIdDecouple.out + ( 1 - gIdDecouple.c )*gITAcr.PIOut;

        gPMFwOut.UqComp     =   gIqDecouple.a*gIqDecouple.out + s_W0*gMotorInfo.VsCoe;//gMotorInfo.VsCoe反电势常数
        gPMFwOut.UdComp     =   -gIdDecouple.a*gIdDecouple.out;

        Saturatefloat(&gPMFwOut.UqComp, gUDC.uDCBigFilter, -gUDC.uDCBigFilter);
        Saturatefloat(&gPMFwOut.UdComp, gUDC.uDCBigFilter, -gUDC.uDCBigFilter);
    }
    else
    {
        gPMFwOut.UdComp = 0;
        gPMFwOut.UqComp = 0;
    }
}

/*************************************************************
       调制比计算
*************************************************************/
static float CalcModifiedModulateIndex( float M )
{
    float newM;
    int16_t i0, i1;
    float M0, M1;

    if( M <= 1 ) return M;
    if( M >= OVR_MOD[MAX_OVR_MOD_LEN - 1] ) return 2;

    ModulateIndexSearch( M,  &i0, &i1 );

    M0 = OVR_MOD[i0];
    M1 = OVR_MOD[i1];

    newM = 1 + 0.005f*i0;
    newM += 0.005f*(M - M0)/(M1 - M0);

    return newM;
}

static void ModulateIndexSearch( float M, int16_t *i0, int16_t *i1 )
{
    int16_t exactFind = 0;
    int16_t low = 0;
    int16_t high = MAX_OVR_MOD_LEN - 1;
    int16_t mid;
    float diff;

    if ( M >= OVR_MOD[MAX_OVR_MOD_LEN - 1] )
    {
        *i0 = MAX_OVR_MOD_LEN - 2;
        *i1 = MAX_OVR_MOD_LEN - 1;
    }
    else if ( M < OVR_MOD[0] )
    {
        *i0 = 0;
        *i1 = 1;
    }
    else
    {
        while( low < high - 1  && 1 != exactFind )
        {
            mid = (( low + high )>>1);

            diff = M - OVR_MOD[mid];

            if( 0 == diff )
            {
                exactFind = 1;
            }
            else if( diff < 0 )
            {
                high = mid;
                mid  = high;
            }
            else
            {
                low = mid;
                mid = high;
            }
        }

        if( 0 == mid )
        {
            low  = 0;
            high = 1;
        }
        else if( MAX_OVR_MOD_LEN - 1 == mid )
        {
            low     =   MAX_OVR_MOD_LEN - 2;
            high    =   MAX_OVR_MOD_LEN - 1;
        }
        else
        {
            if( M > OVR_MOD[mid] )
            {
                low     =   mid;
                high    =   mid + 1;
            }
            else
            {
                low     =   mid - 1;
                high    =   mid;
            }
        }

        *i0 = low;
        *i1 = high;
    }
}


/*************************************************************
      相电流计算
*************************************************************/
void CalcLineCurr(void)
{
    static float gIAmpTheta = 0;

    gIAmpTheta = sqrt( gIMT.M*gIMT.M + gIMT.T*gIMT.T ); //计算相电流峰值

    //...................................计算线电流电流
    gLineCur.Current = gIAmpTheta*0.5f + gLineCur.Current*0.5f;
    gLineCur.CurrentFilter = ( 1 - 0.0625f)*gLineCur.CurrentFilter + 0.0625f*gLineCur.Current;

    gIMT.MFilter = 0.9*gIMT.MFilter + ( 1 - 0.9 )*gIMT.M;
    gIMT.TFilter = 0.9*gIMT.TFilter + ( 1 - 0.9 )*gIMT.T;

    gf32IdRefFilter = 0.99f*gf32IdRefFilter + ( 1 - 0.99f )*gIMAcr.Ref;
    gf32IqRefFilter = 0.99f*gf32IqRefFilter + ( 1 - 0.99f )*gITAcr.Ref;
}

/*************************************************************
      反馈扭矩计算
*************************************************************/
void CalcActTorque( void )
{
    if(  (STATUS_RUNNING != g_RunStatus )
      || (MODE_ACTDISCHARGE == g_McuStMode)
      )
    {
        gPowerTrq.TorqueRef =   0;
        gPowerTrq.Torque    =   0;
        gPowerTrq.TrqOut    =   0;

        return;
    }

    gPowerTrq.Torque    =   GetCalibTorque( gRotorSpeed.SpeedApply, gIMT.MFilter, gIMT.TFilter );
    gPowerTrq.TrqOut    =   0.90*gPowerTrq.TrqOut + ( 1 - 0.90 )*gPowerTrq.Torque;
}


/*************************************************************
      电流环PI参数切换
*************************************************************/
void SelectAcrPar(void)
{
//    float m_DetaIdKP, m_DetaIdKI, m_DetaIqKP, m_DetaIqKI;
//    float m_AbsChgSrc, m_AcrChgVal, m_ChgValUp;
//
//    //切换时是否关中断？或放在中断执行？
//
//	//电流环PI参数设置
//	if (gAcr.GainChgSel == 1) //根据转速切换
//	{
//		m_AbsChgSrc = fabs(gRotorSpeed.SpeedApplyFilter);
//		m_AcrChgVal = gAcr.ChgSpd;
//	}
//	else if (gAcr.GainChgSel == 2) //根据电流点切换
//	{
//		m_AbsChgSrc = gLineCur.CurrentFilter;
//		m_AcrChgVal = gAcr.ChgCurr;
//	}
//
//    //Id环路参数切换
//    m_DetaIdKP = gAcr.IdKpH - gAcr.IdKpL;
//    m_DetaIdKI = gAcr.IdKiH - gAcr.IdKiL;
//
//    if( m_AbsChgSrc <= (m_AcrChgVal - gAcr.ChgHysterVal))
//    {
//    	gIMAcr.Kp = gAcr.IdKpL;
//    	gIMAcr.Ki = gAcr.IdKiL;
//    }
//    else if( m_AbsChgSrc >= m_AcrChgVal )
//    {
//    	gIMAcr.Kp = gAcr.IdKpH;
//    	gIMAcr.Ki = gAcr.IdKiH;
//    }
//    else
//    {
//    	m_ChgValUp    = m_AbsChgSrc - (m_AcrChgVal - gAcr.ChgHysterVal);
//        gIMAcr.Kp = m_DetaIdKP*m_ChgValUp/gAcr.ChgHysterVal + gAcr.IdKpL;
//        gIMAcr.Ki = m_DetaIdKI*m_ChgValUp/gAcr.ChgHysterVal + gAcr.IdKiL;
//    }
//
//    //Iq环路参数切换
//    m_DetaIqKP = gAcr.IqKpH - gAcr.IqKpL;
//    m_DetaIqKI = gAcr.IqKiH - gAcr.IqKiL;
//
//    if( m_AbsChgSrc <= (m_AcrChgVal - gAcr.ChgHysterVal))
//    {
//    	gITAcr.Kp = gAcr.IqKpL;
//    	gITAcr.Ki = gAcr.IqKiL;
//    }
//    else if( m_AbsChgSrc >= m_AcrChgVal )
//    {
//    	gITAcr.Kp = gAcr.IqKpH;
//    	gITAcr.Ki = gAcr.IqKiH;
//    }
//    else
//    {
//    	m_ChgValUp    = m_AbsChgSrc - (m_AcrChgVal - gAcr.ChgHysterVal);
//        gITAcr.Kp = m_DetaIqKP*m_ChgValUp/gAcr.ChgHysterVal + gAcr.IqKpL;
//        gITAcr.Ki = m_DetaIqKI*m_ChgValUp/gAcr.ChgHysterVal + gAcr.IqKiL;
//    }

    gIMAcr.Kp = 0.2f;  //0.5f;    //0.5f;    //1.00f  //0.2f;
    gIMAcr.Ki = 0.002f;//0.0025f; //0.0025f; //0.04f; //0.005f;

    gITAcr.Kp = 0.2f;//
    gITAcr.Ki = 0.002f;//0.0025f;

}

/*************************************************************
      电流环参数复位
*************************************************************/
void ResetCsrPar(void)
{
    gIMTSet.M          = 0;
    gIMTSet.T          = 0;
    gIMTSetApply.M     = 0;
    gIMTSetApply.T     = 0;
    gUMTSet.M          = 0;
    gUMTSet.T          = 0;

    gIMAcr.Err         = 0;
    gIMAcr.PIOut       = 0;
    gITAcr.Err         = 0;
    gITAcr.PIOut       = 0;

    s_f32IdComp = 1;
    s_f32IqComp = 1;
}
