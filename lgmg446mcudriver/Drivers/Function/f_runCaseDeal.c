/*
 * f_runCaseDeal22.c
 *
 *  Created on: 2024年12月3日
 *      Author: pc
 */

//funcCode.code.inputVoltOn  开机电压
//funcCode.code.inputVoltOff 关机电压(欠压点)
//funcCode.code.inputVoltOvr 过压保护点

#include "f_public.h"

//-------全局变量---------
uint16_t g_McuEnableReq = 0;             //使能
uint16_t g_McuModeRe = TORQUECTRL_CMD;   //控制命令
uint16_t g_McuRunDir = FORWARD_DIR;      //运行方向
uint16_t g_RunStatus = STATUS_LOWPOWER;  //运行状态
uint16_t g_McuStMode = 0;                //运行模式


uint16_t g_StartFlag = 0;          //启动标志
uint16_t g_TorqueCtlFlag = 0;//1;      //扭矩控标志

/*************************************************************
  状态机
*************************************************************/

//注意点：
//1 状态的切换要注意时间周期，例如状态机2ms，而PWM中断us，所以状态机最好不要有一个状态下，多个状态判断，如果Pwm使用了这个变化的状态可能有问题。
//2 标定只给控制模式，使能也操作赋值1？
//3 运行状态与参数辨识的处理
//4 如果用g_McuStMode进行判断，run和mode隔了一步，问题导致已经判断不运行了，mode还是多运行一次。参数辨识多运行一次，可能会导致又开波了。使用运行和模式条件判断时，可能会导致第一次条件异常
//5 运行时一级故障扭矩清零，这一点条件注意
//故障关波响应，2ms控制命令变化。
//遗留问题：1阈值先用后赋值，顺序不太对  2先使能，而后判断转速或扭矩
//硬件故障中断关波和2ms中断开波的冲突
//故障的恢复
void RunStModeDeal(CanReDmdStruct mDmdRe)
{
    static uint16_t s_standyWaiteTime = 0;
    static uint16_t s_uDCFilterOld  = 0;
    static uint16_t s_uDCStableFlag  = 0;
    static uint16_t s_McuModeOld  = 0;

    //控制命令输入
    g_McuModeRe = mDmdRe.ModeReq;
    g_McuRunDir = mDmdRe.Direction;
    g_McuEnableReq = mDmdRe.EnableReq;


    switch (g_RunStatus)
    {
        case STATUS_LOWPOWER:                                                       //待机状态

        	if (0 == s_uDCStableFlag)
        	{
                if(  (gUDC.uDCFilter > (gLimitParams.inputVoltOff + gLimitParams.VoltOnDiff))      //母线电压大于阈值
//                  && (gUDC.uDCFilter <= s_uDCFilterOld)  //母线电压没再增加
                  )
                {
                    s_standyWaiteTime++;
                    if (s_standyWaiteTime >= 200)
                    {
                        s_standyWaiteTime = 0;
                        s_uDCStableFlag = 1;
                    }

                }
                else
                {
                    s_standyWaiteTime = 0;
//                    s_uDCFilterOld = gUDC.uDCFilter;
                }
        	}
        	else
        	{
                s_standyWaiteTime++;
                if (s_standyWaiteTime >= 200)          //稳定后延迟200ms
                {
                    s_standyWaiteTime = 0;
                    s_uDCStableFlag = 0;
                    g_RunStatus = STATUS_READY;         //跳转准备状态

                    if (1 == gFault3rd.bit.SwUnderUdc)
                    {
                    	gFault3rd.bit.SwUnderUdc = 0;
                    }
                    else
                    {}
                }
        	}

            break;

        case STATUS_READY:                                                        //准备状态

            RunDataIntit();   //启动电机准备的数据初始化处理，为电机运行准备初始参数
        	PMFwInit();

			if (  (1 == gADOffsetInfo.Flag)                //零漂检验完成
			   && (1 == KeyOn_Enable)                  //预充完成
			   && (0 == PowerOn_Logic_Fault_Drive)     //提前动作不运行
			   && (1 == MainContactorStatus)           //主接触器吸合完成
			   )
			{
				if (  (g_McuModeRe == PARAMEST_CMD)
				   && (gFault3rd.all == 0)
				   )
				{
					g_RunStatus = STATUS_PARAMEST;//参数辨识状态
				}
				else
				{
					if (  (g_McuEnableReq == 1)
					   && (  (g_McuModeRe == ACTDISCHARGE_CMD)
						  || (  (g_McuModeRe == SPEEDCTRL_CMD || g_McuModeRe == TORQUECTRL_CMD)
							 && (gFault3rd.all == 0)
							 )
						  )
					   )
					{
//                            g_PwmEnable = 1;//开波使能
						g_RunStatus = STATUS_RUNNING;//运行状态
					}
				}
			}

            break;

        case STATUS_RUNNING:                                                     //运行状态

        	if (  (g_McuEnableReq == 0)  //无使能
               || ((gFault3rd.all != 0) && (g_McuModeRe != ACTDISCHARGE_CMD))//非泄放模式下有故障
               || (  (g_McuModeRe != TORQUECTRL_CMD)//非泄放/扭矩/转速模式
                  && (g_McuModeRe != SPEEDCTRL_CMD)
                  && (g_McuModeRe != ACTDISCHARGE_CMD)
                  )
               )
            {
                g_PwmEnable = 0;  //使能标志清零
                DisableDrive();
//                DisableSlavePWM();//关波延迟？

                g_RunStatus = STATUS_READY;//运行指令取消跳转到准备状态

            }

            break;

        case STATUS_PARAMEST:                                                      //参数辨识

            if (  (g_McuModeRe != PARAMEST_CMD)
               || (gFault3rd.all != 0)
               )
            {
                g_PwmEnable = 0;  //使能标志清零
                DisableDrive();
//                DisableSlavePWM();//关波延迟？
                g_RunStatus = STATUS_READY;
            }

            break;

        case STATUS_UPDATE_FIRMWARE:                                                      //参数辨识

			g_PwmEnable = 0;
			DisableDrive();

//			if (g_DriveUpgradeFlag == 1)
//			{
//				NVIC_SystemReset();
//			}


            break;

        default:
            break;
    }

    //运行状态下控制模式切换
    if (STATUS_RUNNING == g_RunStatus)
	{
        if (g_McuModeRe == ACTDISCHARGE_CMD)
        {
        	g_McuStMode = MODE_ACTDISCHARGE;      //主动泄放
        }
        else if (g_McuModeRe == TORQUECTRL_CMD)
        {
			g_McuStMode = MODE_TORQUECTRL;        //扭矩控制
        }
		else if (g_McuModeRe == SPEEDCTRL_CMD)
		{
			g_McuStMode = MODE_SPEEDCTRL;         //转速控制
		}

        //泄放开始前，电流环等数据初始化
    	if (  (g_McuStMode == MODE_ACTDISCHARGE)
    	   && (s_McuModeOld != MODE_ACTDISCHARGE)
    	   )
    	{
    		ResetAsrPar();
    		ResetCsrPar();
    		g_torquecmd = 0;
    	}

    	s_McuModeOld = g_McuStMode;
	}
    else
    {
    	g_McuStMode = 0;  //转速控制
    }


    //电机启动标志（包括参数辨识）
    if (g_RunStatus == STATUS_RUNNING || g_RunStatus == STATUS_PARAMEST)
    {
        g_StartFlag = 1;
    }
    else
    {
        g_StartFlag = 0;
    }

    //控制标志输出
    if (g_McuModeRe == TORQUECTRL_CMD)
    {
        g_TorqueCtlFlag = 1; //扭矩控标志
    }
    else if (g_McuModeRe == SPEEDCTRL_CMD)
    {
        g_TorqueCtlFlag = 0;
    }
}

/*************************************************************
  Running状态处理
*************************************************************/
void RunningDeal(void)
{

	if ((g_McuStMode == MODE_ACTDISCHARGE) && (g_PwmAlreadyOn == 0))
	{
		g_PwmEnable = 1;//开波使能
	}

	if ((g_McuStMode == MODE_SPEEDCTRL || g_McuStMode == MODE_TORQUECTRL) && (g_PwmAlreadyOn == 0) && (gFault3rd.all == 0))
	{
		g_PwmEnable = 1;//开波使能
	}

    //泄放执行函数
    RunActvDischgMode();

    //速度指令处理
    FrqSrcDeal();

    //扭矩指令处理
    TorqueCalc();

    //扭矩控或转速控下的IdIq给定
    GetSpdOrTorIdIqRef();
}


