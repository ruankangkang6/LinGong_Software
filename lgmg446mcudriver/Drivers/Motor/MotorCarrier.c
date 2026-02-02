/*
 * MotorCarrier.c
 *
 *  Created on: 2024年12月20日
 *      Author: pc
 */

#include "f_public.h"
#include "main.h"

//#pragma CODE_SECTION(CalCarrierWaveFreq,    "ramfuncs")
//#pragma CODE_SECTION(AsynPWMAngleCal,       "ramfuncs")

FC_CAL_STRUCT gFcCal = {0, 0, 0, 0};
BASE_PAR_STRUCT gBasePar = {0, 0, 0};

#define SPEED_FREQ      70      //  3k
#define SPEED_FREQ_1    120     //  5k
#define SPEED_FREQ_2    250     //  7k
#define SPEED_FREQ_3    400     //  8.5k
#define SPEED_FREQ_DELT 10      //  转速回差


void AsynPWMAngleCal(uint32_t FcApply);
void CalCarrierWaveFreq(void);

void CalCarrierWaveFreq(void)
{
    uint16_t  m_minFc;
    uint16_t  m_maxFc;
    float m_AbsFreq;

    if( 1 == g_TorqueCtlFlag/*1 == gComPar.SubCommand.bit.TorqueCtl*/ )
    {
        m_AbsFreq = fabs(gRotorSpeed.SpeedApplyFilter);

        if( IS_CALIBRATED )
        {
            //根据转速分段载频
            if ( m_AbsFreq < SPEED_FREQ - SPEED_FREQ_DELT )
            {
                gFcCal.FcBak = 2000;;
            }
            else if ( m_AbsFreq < SPEED_FREQ )
            {
                // keep stable
            }
            else if ( m_AbsFreq < SPEED_FREQ_1 - SPEED_FREQ_DELT )
            {
                gFcCal.FcBak = 5000;

            }
            else if ( m_AbsFreq < SPEED_FREQ_1 )
            {
                // keep stable
            }
            else if ( m_AbsFreq < SPEED_FREQ_2 - SPEED_FREQ_DELT )
            {
                gFcCal.FcBak = 7000;
            }
            else if ( m_AbsFreq < SPEED_FREQ_2 )
            {
                // keep stable
            }
            else if ( m_AbsFreq < SPEED_FREQ_3 - SPEED_FREQ_DELT )
            {
                gFcCal.FcBak = 8500;
            }
            else if ( m_AbsFreq < SPEED_FREQ_3 )
            {
                // keep stable
            }
            else
            {
                gFcCal.FcBak = 10000;
            }
        }
        else
        {
            gFcCal.FcBak = gBasePar.FcSet;   //扭矩模式下，标定程序使用上位机传递的载波频率，由funcCode.code.carrierFrq传递
        }
    }
    else
    {
        gFcCal.FcBak = 10000;//12000;
    }

    m_minFc = 1500;
    m_maxFc = 10000;

    //载频限制
    Saturateuint16_t(&gFcCal.FcBak, m_maxFc, m_minFc);


    //扭矩模式下，载频变化平滑
    if ( gBasePar.FcSetApply == gFcCal.FcBak )
    {
        // do nothing
    }
    else if ( gBasePar.FcSetApply + 100 <= gFcCal.FcBak )
    {
        gBasePar.FcSetApply += 100;
    }
    else if ( gBasePar.FcSetApply < gFcCal.FcBak )
    {
        gBasePar.FcSetApply = gFcCal.FcBak;
    }
    else if ( gBasePar.FcSetApply >= gFcCal.FcBak + 1 )
    {
        gBasePar.FcSetApply -= 1;
    }
    else if ( gBasePar.FcSetApply > gFcCal.FcBak )
    {
        gBasePar.FcSetApply = gFcCal.FcBak;
    }

    if(0 == g_TorqueCtlFlag/*0 == gComPar.SubCommand.bit.TorqueCtl*/ )  //扭矩标志整理思路
    {
        gBasePar.FcSetApply = 10000;//7000;//gBasePar.FcSet;
    }

    if( 0/*STATUS_GET_PAR_PMSM == gMainStatus.RunStep*/ ) //参数辨识整理思路
    {
        gBasePar.FcSetApply = 10000;
    }

    gBasePar.FcSetApply = 12000;//10000;

    Saturateuint16_t(&(gBasePar.FcSetApply), 12000, 1500);

    AsynPWMAngleCal(gBasePar.FcSetApply);
}


/************************************************************
    异步调制情况下根据当前载波频率和运行频率计算载波周期内相位变化步长
************************************************************/
void AsynPWMAngleCal(uint32_t FcApply)
{
    uint16_t m_HalfTc, m_Fc;
    float m_Ts;

    uint32_t m_LData = 450*200000UL;

    if( FcApply > 12000 )
    {
        m_Fc = FcApply>>1;
        m_LData = m_LData>>1;
    }
    else
    {
        m_Fc = FcApply;
    }

    m_Ts        =   1.0e3f/m_Fc;  //载波周期 *1000

    m_HalfTc    =   (uint16_t)( m_LData*m_Ts*1.0e-3f + 0.5f );

//    DINT;  //关中断
    gBasePar.Ts         =   m_Ts*1.0e-3f;  //载波周期，单位s  用于计算零位转速补偿和电压给定补偿

    gSynPWM.FcApply     =   m_Fc;          //载波频率，单位hz

    gPWM.gPWMPrd        =   m_HalfTc;      //计算后的载波频率   用于PWM波计算，和死区补偿计算

    g_f32DeadZoneComp   =   gSynPWM.FcApply*3.0e-6f*gPWM.gPWMPrd;  //死区补偿时间 270 3us

    gIdDecouple.Ts      =   gBasePar.Ts;   //用于电压给定补偿计算
    gIqDecouple.Ts      =   gIdDecouple.Ts;

//    gIMAcr.Kp           =   0.001f*gCommProVar.AcrKp*gSynPWM.FcApply*1.0e-4f;
//    gIMAcr.Ki           =   0.001f*gCommProVar.AcrKi*gSynPWM.FcApply*1.0e-4f;
//    gITAcr.Kp           =   gIMAcr.Kp;
//    gITAcr.Ki           =   gIMAcr.Ki;

    pwm_freq_set(FcApply);
//    EINT; //开中断

//
}
