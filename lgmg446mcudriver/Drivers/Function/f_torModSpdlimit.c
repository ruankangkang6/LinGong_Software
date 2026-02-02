/*
 * f_torqueSpdlimit.c
 *
 *  Created on: 2024年12月16日
 *      Author: pc
 */
#include "f_public.h"

uint16_t  g_u16HighSpeedRegulator =   0;
float g_f32HighSpeedRef       =   0;
float g_f32HighSpeedRefTarget =   0;
uint16_t g_u16AhsExitCnt          =   0;

uint16_t ReadyToEnterHighSpeedRegulator( void );
void EnterHighSpeedRegulator( void );
void RunHighSpeedRegulator( void );
static int16_t IsSpeedDecreasingEx( void );

uint16_t ReadyToEnterHighSpeedRegulator( void )
{
    float m_Speed = 0;

    if (IS_AHS_EN)  //高速限速功能是否开启
    {
        return 0;
    }

    if (0 == g_u16HighSpeedRegulator)
    {
        if ( 0 == IS_POSITIVE_ROTATE_WITH_SPEED_N )
        {
            m_Speed = gRotorSpeed.SpeedApply;
        }
        else
        {
            m_Speed = -gRotorSpeed.SpeedApply;
        }

        if ( m_Speed >= 0 )
        {
            if( m_Speed >= 0.97f*gDmdCanRe.FwdSpdLimit/*gVCPar.torqCtrlMaxFwdFreq*/ )
                return 1;
        }
        else
        {
            if( m_Speed <= -0.97f*gDmdCanRe.RevSpdLimit/*gVCPar.torqCtrlMaxRevFreq*/ )
                return 1;
        }
    }

    return 0;
}

void EnterHighSpeedRegulator( void )
{
    g_u16HighSpeedRegulator = 1;
    g_f32HighSpeedRef = gRotorSpeed.SpeedApply;

    gAsr.Asr.Err = 0;
    gAsr.Asr.PIOut = g_torquecmd;
    gAsr.AsrOutFilter = gAsr.Asr.PIOut;

//    OutputString("AAEnterHSL");
}

void RunHighSpeedRegulator( void )
{
	uint16_t u16Exit = 0;


    if ( gRotorSpeed.SpeedApply >= 0 )
    {
        if ( 0 == IS_POSITIVE_ROTATE_WITH_SPEED_N )
        {
            g_f32HighSpeedRefTarget = gDmdCanRe.FwdSpdLimit;//gVCPar.torqCtrlMaxFwdFreq;//程序正转车辆前进使用前进转速限制gVCPar.torqCtrlMaxFwdFreq
        }
        else
        {
            g_f32HighSpeedRefTarget = gDmdCanRe.RevSpdLimit;//gVCPar.torqCtrlMaxRevFreq;//程序正转车辆后退使用后退转速限制gVCPar.torqCtrlMaxFwdFreq
        }
    }
    else
    {
        if ( 0 == IS_POSITIVE_ROTATE_WITH_SPEED_N )
        {
            g_f32HighSpeedRefTarget = -gDmdCanRe.RevSpdLimit;//gVCPar.torqCtrlMaxRevFreq;
        }
        else
        {
            g_f32HighSpeedRefTarget = -gDmdCanRe.FwdSpdLimit;//gVCPar.torqCtrlMaxFwdFreq;
        }
    }

    if ( g_f32HighSpeedRefTarget > 0 )        //程序转速正转情况下，判断是否执行高速限速
    {
        if ( gRotorSpeed.SpeedApplyFilter < 0.85f*g_f32HighSpeedRefTarget )
        {
            u16Exit = 1;
        }
        else if ( gRotorSpeed.SpeedApplyFilter < 0.90f*g_f32HighSpeedRefTarget )
        {
            if ( IsSpeedDecreasingEx() ) //如果减速则退出高速限速
            {
                u16Exit = 1;
            }
        }
    }
    else if ( g_f32HighSpeedRefTarget < 0 )  //程序转速负转，判断是否执行高速限速
    {
        if ( gRotorSpeed.SpeedApplyFilter > 0.85f*g_f32HighSpeedRefTarget )
        {
            u16Exit = 1;
        }
        else if ( gRotorSpeed.SpeedApplyFilter > 0.90f*g_f32HighSpeedRefTarget )
        {
            if ( IsSpeedDecreasingEx() )//如果减速则退出高速限速
            {
                u16Exit = 1;
            }
        }
    }

    if ( 0 == u16Exit )//启动
    {
        if( g_f32HighSpeedRef < g_f32HighSpeedRefTarget ) //程序正转情况下，转速接近限值，则切为转速模式，目标转速值逐渐趋近并等于限值
        {
            g_f32HighSpeedRef += 0.1;

            if( g_f32HighSpeedRef > g_f32HighSpeedRefTarget )
                g_f32HighSpeedRef = g_f32HighSpeedRefTarget;
        }
        else if( g_f32HighSpeedRef > g_f32HighSpeedRefTarget )//程序负转情况下，转速接近限值，则切为转速模式，目标转速值逐渐趋近并等于限值
        {
            g_f32HighSpeedRef -= 0.1;

            if( g_f32HighSpeedRef < g_f32HighSpeedRefTarget )
                g_f32HighSpeedRef = g_f32HighSpeedRefTarget;
        }


//        gMainCmd.FreqSet = g_f32HighSpeedRef;//目标值输入

        g_freqSet = g_f32HighSpeedRef;//

        VCSpeedControl();

        if( gRotorSpeed.SpeedApply*gAsr.AsrOutFilter >= 0 )  //转速和扭矩同方向下，转速环路输出超过扭矩指令值，限制到扭矩指令值
        {
            if( gRotorSpeed.SpeedApply >= 0 && g_torquecmd < gAsr.Asr.PIOut )    // brake or light fuel
            {
                gAsr.Asr.PIOut = g_torquecmd;
            }
            else if( gRotorSpeed.SpeedApply < 0 && g_torquecmd > gAsr.Asr.PIOut )    // brake or light fuel
            {
                gAsr.Asr.PIOut = g_torquecmd;
            }
        }
        else                                                 //转速和扭矩不同方向下，扭矩指令值输出超过环路输出，环路输出设置为扭矩指令值
        {
            // regen
            if( gRotorSpeed.SpeedApply >= 0 && g_torquecmd < gAsr.Asr.PIOut )  //扭矩和转速方向
            {
                gAsr.Asr.PIOut = g_torquecmd;

            }
            else if( gRotorSpeed.SpeedApply < 0 && g_torquecmd > gAsr.Asr.PIOut )
            {
                gAsr.Asr.PIOut = g_torquecmd;
            }
        }
    }
    else  ////退出
    {
        g_u16HighSpeedRegulator = 0;

        g_u16AhsExitCnt = 250;
        g_torquecmd = gAsr.Asr.PIOut;

//        OutputString("AALeaveHSL");
    }

}

//减速判断
static int16_t IsSpeedDecreasingEx( void )
{
	int16_t ret = 0;

    if( gRotorSpeed.SpeedApplyFilter500msZ2 > gRotorSpeed.SpeedApplyFilter500msZ1
     && gRotorSpeed.SpeedApplyFilter500msZ1 > gRotorSpeed.SpeedApplyFilter500ms
     && gRotorSpeed.SpeedApplyFilter500ms >= 0 )
    {
        ret = 1;
    }
    else if( gRotorSpeed.SpeedApplyFilter500msZ2 < gRotorSpeed.SpeedApplyFilter500msZ1
     && gRotorSpeed.SpeedApplyFilter500msZ1 < gRotorSpeed.SpeedApplyFilter500ms
     && gRotorSpeed.SpeedApplyFilter500ms <= 0 )
    {
        ret = 1;
    }

    return ret;
}

