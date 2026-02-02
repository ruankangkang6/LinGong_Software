
/*
输入信号：
1.泄放指令
1.泄放电压
1.泄放时间
1.母线电压
1.电机转速

输出信号：
1.Id给定
2.泄放故障
*/

//上位机的标定数据
//funcCode.code.actvDischgSafetyVol      //泄放电压阈值
//funcCode.code.actvDischgTimeOutSec     //泄放超时时间

#include "f_public.h"

//-------全局变量---------
uint16_t gu16ActvDischgRunStatus = ACTV_DISCHG_WAIT;
ACTV_DISCHG_CTRL_STRUCT gActvDischgCtrl = {0, 0, 0, 0};

//-------函数声明---------
void RunActvDischgMode(void);


/*************************************************************
    执行泄放函数
*************************************************************/
void RunActvDischgMode(void)
{
    float f32Temp = 0;
    static uint16_t DischgCtrlCounter = 0;
    static uint16_t DischgRecoveryCnt = 0;

    if ( g_McuStMode != MODE_ACTDISCHARGE)//非泄放模式
    {
        gActvDischgCtrl.IdRef = 0;
        gActvDischgCtrl.IqRef = 0;
        DischgCtrlCounter = 0;
        DischgRecoveryCnt = 0;
        gu16ActvDischgRunStatus  = ACTV_DISCHG_WAIT;
        return;
    }

    switch (gu16ActvDischgRunStatus)
    {
        case ACTV_DISCHG_WAIT:                                                                //泄放等待

        	if (  (ACTDISCHARGE_CMD == g_McuModeRe)            //有泄放命令
               && (1 == gADOffsetInfo.Flag)            //霍尔自检完成
//               && (0 == gFault3rd.bit.ErrorDisCharge)       //无泄放故障
               && (fabs(gRotorSpeed.SpeedApply) <= 2)    //转速小于2HZ
               && (gUDC.uDC >= gActvDischgCtrl.SafetyVolt)           //电压大于泄放电压
              )
            {
                gu16ActvDischgRunStatus  = ACTV_DISCHG_RUN;
            }

            break;

        case ACTV_DISCHG_RUN:                                                                 //泄放进行

        	if ( gUDC.uDCBigFilter < gActvDischgCtrl.SafetyVolt - 10)
            {
                gu16ActvDischgRunStatus = ACTV_DISCHG_END;  // 完成
                DischgCtrlCounter = 0;
            }
            else
            {
                DischgCtrlCounter++;
                if (DischgCtrlCounter > gActvDischgCtrl.TimeoutSec*500)
                {
                    DischgCtrlCounter = 0;
//                    gFault3rd.bit.ErrorDisCharge = 1;
                    gu16ActvDischgRunStatus = ACTV_DISCHG_TIMEOUT;
                }
                else
                {
                	f32Temp = 0.075f*( gActvDischgCtrl.SafetyVolt - 20 - gUDC.uDC );

                    if (f32Temp > 0)
                    {
                    	f32Temp = 0;
                    }

                    gActvDischgCtrl.IdRef = 0.95f*gActvDischgCtrl.IdRef + ( 1 - 0.95f )*f32Temp;

                    if ( gActvDischgCtrl.IdRef < f32Temp )
                    {
                        gActvDischgCtrl.IdRef = f32Temp;
                    }

                    Saturatefloat(&gActvDischgCtrl.IdRef, 0, -1.3f*gMotorInfo.Current);
                    gActvDischgCtrl.IqRef = 0;

                    gIMTSet.M = gActvDischgCtrl.IdRef;           //泄放Id给定
                    gIMTSet.T = gActvDischgCtrl.IqRef;           //泄放Iq给定
                }
            }


            break;

        case ACTV_DISCHG_END:                                                                  //泄放完成

        	gActvDischgCtrl.IdRef = 0;
            gActvDischgCtrl.IqRef = 0;
            gIMTSet.M = 0;
            gIMTSet.T = 0;

            g_PwmEnable = 0;  //使能标志清零
            DisableDrive();
//                DisableSlavePWM();//关波延迟？

            gFault3rd.bit.SwUnderUdc = 1;
            g_RunStatus = STATUS_LOWPOWER;
            gu16ActvDischgRunStatus = ACTV_DISCHG_WAIT;
            break;

        case ACTV_DISCHG_TIMEOUT:                                                              //泄放超时

        	gActvDischgCtrl.IdRef = 0;
            gActvDischgCtrl.IqRef = 0;
            gIMTSet.M = 0;
            gIMTSet.T = 0;

            DischgRecoveryCnt++;
            if (DischgRecoveryCnt >= 1000)   //2s后重新开始泄放
            {
                DischgRecoveryCnt = 0;
//                gFault3rd.bit.ErrorDisCharge = 0;
                gu16ActvDischgRunStatus = ACTV_DISCHG_WAIT;
            }
            else
            {
                if ( gUDC.uDCBigFilter < gActvDischgCtrl.SafetyVolt)
                {
    //            	gFault3rd.bit.ErrorDisCharge = 0;
                    g_PwmEnable = 0;  //使能标志清零
                    DisableDrive();
        //                DisableSlavePWM();//关波延迟？

                    DischgRecoveryCnt = 0;
                    gFault3rd.bit.SwUnderUdc = 1;
                    g_RunStatus = STATUS_LOWPOWER;
                    gu16ActvDischgRunStatus = ACTV_DISCHG_WAIT;
                }
            }

            break;

        default:
            break;
    }

}

