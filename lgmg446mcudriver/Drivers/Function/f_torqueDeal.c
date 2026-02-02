//=====================================================================
//
// 模块主要功能：对扭矩指令进行处理
//
//=====================================================================


//上位机的标定数据
//funcCode.code.torqueCtrlAccTime      //加速时间
//funcCode.code.torqueCtrlDecTime      //减速时间
//funcCode.code.tcTorque               //目标扭矩

#include "include/f_public.h"

//-------全局变量---------
LINE_CHANGE_STRUCT torqurLine = LINE_CHANGE_STRTUCT_DEFALUTS;
float g_torquecmd = 0;                                  //需求扭矩
uint32_t  g_AccTorTime = 0;                                 //加速时间
uint32_t  g_AecTorTime = 0;                                 //减速时间

//-------文件内变量---------
static float torqueAimValue = 0;                        //目标扭矩
static float torqueCurValue = 0;                        //目标扭矩平滑值

//-------函数声明---------
void TorqueCalc(void);

/*************************************************************
    扭矩指令处理
*************************************************************/
void TorqueCalc(void)
{
    uint32_t accDecTime = 0;
    float m_Data = 0;
    static uint16_t SpeedToTorqExitFlag = 0;

    if (  (g_McuStMode != MODE_TORQUECTRL)
       || (gFault3rd.all != 0)
       )
    {
        torqueAimValue = 0;
        torqueCurValue = 0;
        g_torquecmd = 0;
        return;
    }


    //数据接收
    torqueAimValue = gDmdCanRe.TorqueCmd;//funcCode.code.tcTorque*0.05f - 1000;     //目标扭矩

    //扭矩指令平滑
    if ( torqueAimValue != torqueCurValue )  // 已经达到目标转矩
    {
        if ( ( torqueCurValue >= 0  &&  torqueAimValue > torqueCurValue )
          || ( torqueCurValue <= 0  &&  torqueAimValue < torqueCurValue ) )   // 转矩加速
        {
            accDecTime = 0;//g_AccTorTime;//funcCode.code.torqueCtrlAccTime;
        }
        else
        {
            accDecTime = 0;//g_AecTorTime;//funcCode.code.torqueCtrlDecTime;
        }

        accDecTime = accDecTime * (TIME_UNIT_TORQUE_CTRL_ACC_DEC / RUN_CTRL_PERIOD); // 转换成ticker

        torqurLine.aimValue = torqueAimValue;
        torqurLine.tickerAll = accDecTime;
        torqurLine.maxValue = gMotorInfo.MaxTorque;  //最大扭矩
        torqurLine.curValue = torqueCurValue;
        torqurLine.calc(&torqurLine);
        torqueCurValue = torqurLine.curValue;
    }

    //温度限扭矩
    m_Data = torqueCurValue;//torqurLine.curValue;

    if ( torqurLine.curValue >= 0 )
    {
        if( g_DerateTorq < m_Data )
        {
            m_Data = g_DerateTorq;
        }
    }
    else
    {
        if( g_DerateTorq < -m_Data )
        {
            m_Data = -g_DerateTorq;
        }
    }

    //高速限速功能退出与转速切扭矩模式后，扭矩过渡平滑
    if( g_u16AhsExitCnt > 0)
    {
        g_torquecmd = 0.97*g_torquecmd + 0.03*m_Data;          //高速限速退出，扭矩过渡
        g_u16AhsExitCnt--;
    }
    else if( g_u16SpeedToTorqExitCnt > 0 )
    {
        g_torquecmd = 0.90*g_torquecmd + 0.1*m_Data;           //转速切扭矩，扭矩过渡
        g_u16SpeedToTorqExitCnt--;
        SpeedToTorqExitFlag = 1;
    }
    else
    {
        if( SpeedToTorqExitFlag == 1)
        {
            g_torquecmd = 0.97*g_torquecmd + 0.03*m_Data;
            SpeedToTorqExitFlag = 0;
        }
        else
        {
            g_torquecmd = m_Data;
        }
    }
}
