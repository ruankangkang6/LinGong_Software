/*=============================================================================*
 *         Copyright(c) 2016, HT Renewable Energy
 *                          ALL RIGHTS RESERVED
 *
 *  PRODUCT  : EV Motor Controller
 *
 *  FILENAME : FluxWeak.c 
 *  PURPOSE  :
 *  
 *  HISTORY  :
 *    DATE            VERSION        AUTHOR            NOTE
 *    2016-06-23      V1.0           king       Created.
 *
 *    
 *============================================================================*/
#include "f_public.h"

//#pragma CODE_SECTION(PMFluxWeakDealEx,        "ramfuncs")
//#pragma CODE_SECTION(PMSetFwPara1,          "ramfuncs")

float gCsrUAmp = 0;
PM_FW gPMFwCtrl = {0, 0, 0, 0, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0};
PM_FW_OUT gPMFwOut = {0, 0, 0, 0};
MTPA gMTPA = {0, 0, 0, 0, 0, 0, 0, 0};

uint16_t gu16MaxFwIdSetting = 226*100;

void PMFwInit(void);
void PMMaxTorqCtrl( float iqSet );
void PMFluxWeakDealEx( void );
void PMSetFwPara1(void);


void PMFwInit(void)
{
    gCsrUAmp = 0;

	gPMFwCtrl.IdForTorqFilter =	0;

	gPMFwCtrl.IdForTorq =	0;
	gPMFwOut.IdSet		=	0;
	gPMFwOut.IqSet		=	0;
	gPMFwCtrl.AdId		=	0;
	gPMFwCtrl.VoutAmpLpf	=	0;

	gPMFwCtrl.IqLpf		=	0;

	gPMFwCtrl.FwPI.ErrLmt	=	50;
	gPMFwCtrl.FwPI.Err		=	0;
	gPMFwCtrl.FwPI.ErrZ		=	0;
	gPMFwCtrl.FwPI.PIOut	=	0;
	gPMFwCtrl.FwPI.MaxPIOut =	0;
	gPMFwCtrl.FwPI.MinPIOut =	0;

	gPMFwCtrl.StopIncIqByIdRef = 0;
	gPMFwCtrl.StopIncIqByVout  = 0;
	gPMFwCtrl.StopIncIqByVoutCnt  = 0;
}

void PMMaxTorqCtrl( float iqSet )
{
	float imSet, data;
	float absIqSet;

    if( gMotorInfo.LD < gMotorInfo.LQ )
    {
        absIqSet = fabs(iqSet);
        data	=	( gMotorInfo.LD - gMotorInfo.LQ )*absIqSet;
        data	=	( -gMotorInfo.VsCoe + sqrt( gMotorInfo.VsCoe*gMotorInfo.VsCoe + 4*data*data ) )/( ( gMotorInfo.LD - gMotorInfo.LQ )*2 );
        Saturatefloat(&data, 0, -sqrt3*absIqSet);
        imSet = data;
    }
    else
    {
        imSet = 0;
    }

    if( gPMFwCtrl.StopIncIqByIdRef || gPMFwCtrl.StopIncIqByVout )
    {
        if( gPMFwCtrl.IdForTorq >= imSet )
        {
            return;
        }
    }

    gPMFwCtrl.IdForTorq = imSet;
}

void PMFluxWeakDealEx( void )
{
	float Vd, Vq, Vq2;
	float absSpdFreq;
	float We;
	float temp = 0;
	float coefAdj;
	float m_ImSet = 0;

    //速度小于3000rpm不计算id给定
    if ( gMotorInfo.LD <= 0 || gMotorInfo.LQ <= 0 || gMotorInfo.VsCoe <= 0 )
    {
        gPMFwOut.IdSet = 0;

        return;
    }

    gPMFwCtrl.IdMax1 = 600;//0.0001f*gu16MaxFwIdSetting*gMotorInfo.Current;

    gPMFwCtrl.IqLpf = 0.99f*gPMFwCtrl.IqLpf + ( 1 - 0.99f )*gIMTSetApply.T;
    gPMFwCtrl.IdForTorqFilter = 0.99f*gPMFwCtrl.IdForTorqFilter + ( 1 - 0.99f )*gPMFwCtrl.IdForTorq;

    absSpdFreq = fabs(gRotorSpeed.SpeedApplyFilter);
    We = Value_2Pi*absSpdFreq;

    Vd = We*gMotorInfo.LQ*gPMFwCtrl.IqLpf;

    if(1 == g_TorqueCtlFlag/* 1 == gComPar.SubCommand.bit.TorqueCtl*/ )
    {
#if ( CODE_MODE_UNCALIBRATED_ON_ROAD_RUNNING == CODE_MODE )
        temp = gUDC.uDCBigFilter*Cnst_1divsqrt3;
#else
        temp = gUDC.uDCBigFilter;
#endif
    }
    else
    {
        temp = gUDC.uDCBigFilter*Cnst_1divsqrt3;
    }

    Vq2 = temp*temp - Vd*Vd;
    if ( Vq2 > 0 )
    {
        Vq = sqrt(Vq2);
    }
    else
    {
        Vq = 0;
    }

    temp = (Vq/We - gMotorInfo.VsCoe)/gMotorInfo.LD; //得到最大id？

    if(1 == g_TorqueCtlFlag/* 1 == gComPar.SubCommand.bit.TorqueCtl*/ )
    {
#if ( CODE_MODE_UNCALIBRATED_ON_ROAD_RUNNING == CODE_MODE )
        if( temp > gPMFwCtrl.IdForTorqFilter )
        {
            temp = gPMFwCtrl.IdForTorqFilter;
        }

        Saturatefloat( &temp, 0, -0.5f*gPMFwCtrl.IdMax1 );
        gPMFwOut.IdSet = temp;
#else
        if ( absSpdFreq < gMotorInfo.Frequency*1.5f )
        {
            temp *= 0.30f;
        }
        else if ( absSpdFreq > gMotorInfo.Frequency*2.0f )
        {
            temp *= 0.60f;
        }
        else
        {
            temp *= 0.30f*( 1 + ( absSpdFreq - gMotorInfo.Frequency*1.5f )/( gMotorInfo.Frequency*0.5f ) );
        }

        if( temp > gPMFwCtrl.IdForTorqFilter )
        {
            temp = gPMFwCtrl.IdForTorqFilter;  //Id限制
        }

        Saturatefloat( &temp, 0, -gPMFwCtrl.IdMax1 );
        gPMFwOut.IdSet = temp;
#endif
    }
    else
    {
        temp = gPMFwCtrl.IdForTorqFilter;

        Saturatefloat( &temp, 0, -0.8f*gPMFwCtrl.IdMax1 );
        gPMFwOut.IdSet = temp;
    }

    /************************************************
    电压饱和弱磁电流积分计算，直接使用最简单的累加
    *************************************************/
    // king comment
    // 现有资料表明，这里的电压参考取值为 K*Vdc/sqrt(3)
    // K取值范围为 0.85~1.00之间，视保守程度而定
    // I CONTROL

    coefAdj = 1.0;
    temp = ( gUDC.uDCBigFilter*0.972f - gPMFwCtrl.VoutAmpLpf )*gPMFwCtrl.FwPI.Ki;
    if( temp > 0 ) temp *= 0.5f;
    Saturatefloat(&temp, 5, -10);
    gPMFwCtrl.AdId += temp;
    Saturatefloat(&gPMFwCtrl.AdId, 0, -gPMFwCtrl.IdMax1 - gPMFwOut.IdSet );
    gPMFwOut.IdSet += gPMFwCtrl.AdId*coefAdj;
    Saturatefloat(&gPMFwOut.IdSet, 0, -gPMFwCtrl.IdMax1 );

    if (1 == g_TorqueCtlFlag/* 1 == gComPar.SubCommand.bit.TorqueCtl*/ )
    {
        if (  0 == gPMFwCtrl.StopIncIqByIdRef )
        {
            if ( gPMFwOut.IdSet <= -gPMFwCtrl.IdMax1 + 10 )
            {
                gPMFwCtrl.StopIncIqByIdRef = 1;
            }
        }
        else
        {
            if ( gPMFwOut.IdSet >= -gPMFwCtrl.IdMax1 + 20 )
            {
                gPMFwCtrl.StopIncIqByIdRef = 0;
            }
        }

        if ( gPMFwOut.IdSet < fabs(gMTPA.Curr)*gMTPA.CosVal )
        {
            m_ImSet = gPMFwOut.IdSet;
        }
        else
        {
            m_ImSet = fabs(gMTPA.Curr)*gMTPA.CosVal;
        }

        Saturatefloat(&m_ImSet, 0, -gPMFwCtrl.IdMax1);

        gIMTSet.M = m_ImSet;
    }
    else
    {
        if (  0 == gPMFwCtrl.StopIncIqByIdRef )
        {
            if ( gPMFwOut.IdSet <= -gPMFwCtrl.IdMax1 + 10 )
            {
                gPMFwCtrl.StopIncIqByIdRef = 1;
            }
        }
        else
        {
            if ( gPMFwOut.IdSet >= -gPMFwCtrl.IdMax1 + 20 )
            {
                gPMFwCtrl.StopIncIqByIdRef = 0;
            }
        }

        gIMTSet.M = gPMFwOut.IdSet;
    }
}

void PMSetFwPara1(void)
{
    gPMFwCtrl.VoutAmpLpf = 0.9921875f*gPMFwCtrl.VoutAmpLpf + ( 1 - 0.9921875f )*gCsrUAmp;

}
