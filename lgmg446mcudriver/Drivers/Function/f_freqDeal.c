//=====================================================================
//
// 模块主要功能：对速度指令进行处理
//
//=====================================================================

//上位机的标定数据
//funcCode.code.maxFrq                    //最大频率
//funcCode.code.upperFrq                  //上限频率
//funcCode.code.lowerFrq                  //下限频率
//funcCode.code.accTime1                  //加速时间
//funcCode.code.decTime1                  //减速时间
//gMotorRunSetting.f32RunFreqSetting      //目标频率


#include "include/f_public.h"

//-------全局变量---------
LINE_CHANGE_STRUCT frqLine = LINE_CHANGE_STRTUCT_DEFALUTS;
uint32_t g_AccFrqTime = 0;                             //加速时间
uint32_t g_DecFrqTime = 0;                             //减速时间
float g_freqSet = 0;                               //执行转速频率

//-------文件内变量--------
static float frqAimValue = 0;                           //目标频率
static float frqCurValue = 0;                           //目标频率当前平滑值

//-------函数声明--------
void FrqSrcDeal(void);

/*************************************************************
    转速指令处理
*************************************************************/
void FrqSrcDeal(void)
{
	uint32_t AccDecTime = 0;                             //加减速时间
	float m_MaxRpm = 0;

    if (  (g_McuStMode != MODE_SPEEDCTRL)
       || (gFault3rd.all != 0)
       )
    {
        frqAimValue = 0;
        frqCurValue = gRotorSpeed.SpeedApply;  //目标频率当前值与当前转速保持一致，防止关波又恢复情况下目标转速再次从零开始平滑导致速度调节异常
        g_freqSet = 0;
        return;
    }

    frqAimValue = gDmdCanRe.FreqCmd;//gMotorRunSetting.f32RunFreqSetting;     //目标频率

    //根据上位机上限频率，对目标转速进行限制
    m_MaxRpm =  ( 0.0167f*gMotorInfo.MaxRpm*gMotorInfo.Poles );
    if (frqAimValue >= m_MaxRpm)
    {
        frqAimValue = m_MaxRpm;
    }
    else if (frqAimValue < -m_MaxRpm)
    {
        frqAimValue = -m_MaxRpm;
    }

    //根据运行方向，判断目标转速正负
    frqAimValue = fabs(frqAimValue);
    if ( FORWARD_DIR != g_McuRunDir )
    {
        frqAimValue = -frqAimValue;
    }

    //根据加减速时间，对目标转速进行加载处理
    if ( frqAimValue != frqCurValue)
    {
        if (  ( frqCurValue >= 0 && frqAimValue > frqCurValue )
           || ( frqCurValue <= 0 && frqAimValue < frqCurValue )
           )
        {
            AccDecTime = 20;//g_AccFrqTime;//10  //加速
        }
        else
        {
            AccDecTime = 20;//g_DecFrqTime;//10 //减速
        }


        AccDecTime = AccDecTime*100/RUN_CTRL_PERIOD;

        frqLine.aimValue = frqAimValue;
        frqLine.tickerAll = AccDecTime;
        frqLine.maxValue = gMotorInfo.MaxRpm;
        frqLine.curValue = frqCurValue;
        frqLine.calc(&frqLine);
        frqCurValue = frqLine.curValue;
    }

    g_freqSet = frqCurValue;                              //目标转速当前值赋值给转速设定
}
