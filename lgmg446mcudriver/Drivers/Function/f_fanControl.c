/*/*
 * f_fanControl.c
 *
 *  Created on: 2024年12月2日
 *      Author: pc
 */
#include "include/f_public.h"


uint16_t FanDuty = 0;

//-------函数声明---------
void FanDutyControl( void );
//void FanFaultCheck( void );

#if ( COLD_MODE == WIND_COLD )

//风扇控制
void FanDutyControl( void )
{
    if( gTemperature.Temp >= 60)
    {
        FanDuty = 100;
    }
    else if (gTemperature.Temp <= 55)
    {
        if( gTemperature.Temp >= 50 )
        {
            FanDuty = 80;
        }
        else if (gTemperature.Temp <= 45)
        {
            FanDuty = 0;
        }
    }
    else
    {}

    if( FanDuty > 0 /*|| gu16IsFanSelfCheck*/ )
    {
        //gu16RunBoardAutoTest除上位机无1赋值，gu16IsFanAutoOn为0，gu16IsFanSelfCheck过500赋值1，后面又变零，单纯转一下？
        //gu16RunBoardAutoTest还涉及俩未被调用的函数
        if(0 /*gu16RunBoardAutoTest || gu16IsFanAutoOn || gu16IsFanSelfCheck*/ )
        {
            gPWM.Fan = HALF_INIT_FAN_PRD;
        }
        else
        {
            gPWM.Fan = FanDuty*4.99f;//499*0.01;//s_u16PwmDuty1;
        }


        if ( gPWM.Fan > 0 )
        {
//            if ( 0x00 != EPwm6Regs.AQCSFRC.bit.CSFA )
//            {
//                EPwm6Regs.AQCSFRC.bit.CSFA = 0x00;
//            }
//            EPwm6Regs.CMPA.half.CMPA = gPWM.Fan;
        }
        else
        {
//            EPwm6Regs.AQCSFRC.bit.CSFA = 0x01;
        }

        if ( gPWM.Fan > 0 )
        {
//            if ( 0x00 != EPwm4Regs.AQCSFRC.bit.CSFB )
//            {
//                EPwm4Regs.AQCSFRC.bit.CSFB = 0x00;
//            }
//            EPwm4Regs.CMPB = gPWM.Fan;
        }
//        else
//        {
//            EPwm4Regs.AQCSFRC.bit.CSFB = 0x01;
//        }
    }
    else
    {
//        EPwm6Regs.AQCSFRC.bit.CSFA = 0x01;
//        EPwm4Regs.AQCSFRC.bit.CSFB = 0x01;
    }
}

//风扇故障判断
//void FanFaultCheck( void )
//{
//    if ( !IS_FAN_RUNNING )
//    {
//        return;
//    }
//
//    if ( gMainStatus.ErrFlag.bit.FanFault )
//    {
//        if( !IS_FAN_FAULT )
//        {
//            ++s_u16ExitFanFaultCnt;
//            if ( s_u16ExitFanFaultCnt >= 500*3 )
//            {
//                gMainStatus.ErrorCode.bit.ERROR_FAN_OR_OTHERS = 0;
//                gMainStatus.ErrFlag.bit.FanFault = 0;
//                s_u16ExitFanFaultCnt = 0;
//                s_u16EnterFanFaultCnt = 0;
//            }
//        }
//        else
//        {
//            s_u16ExitFanFaultCnt = 0;
//        }
//    }
//    else
//    {
//        if( IS_FAN_FAULT )
//        {
//            ++s_u16EnterFanFaultCnt;
//            if ( s_u16EnterFanFaultCnt >= 500*3 )
//            {
//                //gMainStatus.ErrorCode.bit.ERROR_FAN_OR_OTHERS = 1;
//                //gMainStatus.ErrFlag.bit.FanFault = 1;
//                s_u16ExitFanFaultCnt = 0;
//            }
//        }
//        else
//        {
//            s_u16EnterFanFaultCnt = 0;
//        }
//    }
//}

#endif
