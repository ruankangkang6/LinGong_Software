/*
 * f_asrControl.c
 *
 *  Created on: 2024年12月2日
 *      Author: pc
 */
#include "f_public.h"

//Flash复制到RAM运行，响应快？
//#pragma CODE_SECTION(VCSpeedControl,            "ramfuncs")
//#pragma CODE_SECTION(SelectAsrPar,              "ramfuncs")
//#pragma CODE_SECTION(VCAsrControl,              "ramfuncs")

// funcCode.code.vcSpdLoopKp1,            //低速Kp
// funcCode.code.vcSpdLoopTi1,            //低速KI
// funcCode.code.vcSpdLoopKp2,            //高速Kp
// funcCode.code.vcSpdLoopTi2,            //高速Ki
// funcCode.code.vcSpdLoopChgFrq1,        //低速切换阈值
// funcCode.code.vcSpdLoopChgFrq2,        //高速切换阈值
// funcCode.code.vcSpdLoopFilterTime,     //PI输出滤波值

//-------全局变量---------
ASR_STRUCT gAsr = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0, 0, 0, 0, 0, 0};
uint16_t g_MaxFeedbackCurr = 0;   //最大回馈电流
uint16_t g_SpdCtrlEnable = 0;       //叉车速度环控制使能
extern float gf32MaxMotorTorque;
extern float gf32MaxGeneratorTorque;
extern Uint16 AntiTime;

uint16_t gASR_KP = 0;
uint16_t gASR_Ki = 0;
int16_t gASR_OUTkp = 0;
int16_t gASR_OUTki = 0;
int16_t gASR_SpdGmd = 0;
int16_t gASR_SpdFb = 0;

uint16_t g_ResetAnti = 0;       //重新驻坡标志
double g_RecoverKI = 0;         //重置前的Ki值
uint16_t g_ResetKIFlag = 0;       //驻坡标志
int16_t count2 = 0;

//Uint16 Flag_LowACC_old = 0;
//-------函数声明---------
void VCSpeedControl(void);
void SelectAsrPar(void);
void VCAsrControl(void);
void VCAsrControlFL(void);
void ResetAsrPar( void );

/*************************************************************
	速度控制
*************************************************************/
void VCSpeedControl(void)
{
    SelectAsrPar();    //转速环路PI
//    VCAsrControl();    //转速环输出
    VCAsrControlFL();  //转速环输出，叉车
}


/*************************************************************
	环路参数处理
*************************************************************/
void SelectAsrPar(void)
{
    float m_DetaKP, m_DetaKI, m_DetaFreq;
    float m_AbsFreq, m_FreqUp;

    //ASR环路参数处理
    if ( gPMEstControl.RunBemfEst )   //参数辨识
    {
    	m_AbsFreq = gPMEstControl.BemfEstSpdRef;
    }
    else
    {
        m_AbsFreq = fabs(gRotorSpeed.SpeedApplyFilter);
    }

    gAsr.KPHigh = 1.25f;
    gAsr.KIHigh = 0.01f;

    gAsr.KPLow  = 4.00f;
    gAsr.KILow  = 0.01f;

    gAsr.SwitchLow   = 150;
    gAsr.SwitchHigh   = 250;

    m_DetaKP = gAsr.KPHigh - gAsr.KPLow;
    m_DetaKI = gAsr.KIHigh - gAsr.KILow;

    if( m_AbsFreq <= gAsr.SwitchLow )
    {
        gAsr.Asr.Kp = gAsr.KPLow;
        gAsr.Asr.Ki = gAsr.KILow;
    }
    else if( m_AbsFreq >= gAsr.SwitchHigh )
    {
        gAsr.Asr.Kp = gAsr.KPHigh;
        gAsr.Asr.Ki = gAsr.KIHigh;
    }
    else
    {
        m_FreqUp    = m_AbsFreq - gAsr.SwitchLow;
        m_DetaFreq  = gAsr.SwitchHigh - gAsr.SwitchLow;
        gAsr.Asr.Kp = m_DetaKP*m_FreqUp/m_DetaFreq + gAsr.KPLow;
        gAsr.Asr.Ki = m_DetaKI*m_FreqUp/m_DetaFreq + gAsr.KILow;
    }


//	float m_AbsFreq = fabs(gRotorSpeed.SpeedApplyFilter);


//#define Switch_frq_1	(5)
//#define Switch_frq_2	(10)
//
//    if(m_AbsFreq < Switch_frq_1)
//    {
//    	m_AbsFreq = Switch_frq_1;
////        gAsr.Asr.Ki = 0.003F;//0.005
//    }
//    else if(m_AbsFreq > Switch_frq_2)
//    {
//    	m_AbsFreq = Switch_frq_2;
////        gAsr.Asr.Ki = 0.003F;//0.003
//    }
//
////    if(m_AbsFreq < 2)
////	{
////	    gAsr.Asr.Ki = 0.1F;
////	}



    if (Flag_Stop == 1)
    {
        gAsr.Asr.Kp = 7.5f;//1.5f;//-0.3F * m_AbsFreq + 4.5F;//-0.45F * m_AbsFreq + 15.0F;//1.8f;//3.80f;//1.5F;//（当前最优）
    }

//    if (gAsr.Asr.Kp < 0.0F)
//    	gAsr.Asr.Kp = 0.0F;
////    gAsr.Asr.Ki = 0.003F;//0.0005F * m_AbsFreq - 0.015F;//0.01f;//0.02f//0.05f;
//    gAsr.Asr.Ki = 0.02F;
//
//    if ((Flag_Anti == 1) && ((Flag_Behind == 1) || (Flag_Front == 1)))
//    {
//        if(fabs(gRotorSpeed.SpeedApplyFilter) < 20)
//    	{
//        	if ((slopecnt <= 1500) && (slopecnt1 <= 1500))   //处理爬坡驻坡后溜
//        	{
//        		if (g_ResetAnti ==1)
//        		{
//        		    if (Flag_Stop == 0)
//        		    {
//                		gAsr.Asr.Kp = 3.0f;
//        		    }
//        		}
//
//        	    gAsr.Asr.Ki = 0.02F;
//        	}
//    	}
//    }
//    else
//    {
//    	g_ResetAnti = 0;
//    }
}


/*************************************************************
	矢量控制的速度环
*************************************************************/
void VCAsrControl(void)
{
    float f32MaxFeedbackTorque = 0;

    if( 1 == g_TorqueCtlFlag/*gComPar.SubCommand.bit.TorqueCtl*/ )
    {
        if( 1 == g_u16HighSpeedRegulator )   //高速限速使能
        {
            f32MaxFeedbackTorque = gUDC.uDCBigFilter*g_MaxFeedbackCurr*gMotorInfo.Poles/( Value_2Pi*fabs(gRotorSpeed.SpeedApply) );
            if ( f32MaxFeedbackTorque > gf32MaxGeneratorTorque )
            {
                f32MaxFeedbackTorque = gf32MaxGeneratorTorque;
            }

            if ( gRotorSpeed.SpeedApply <= -gMotorInfo.Frequency )
            {
                gAsr.Asr.MaxPIOut   =   f32MaxFeedbackTorque;
                gAsr.Asr.MinPIOut   =   -gf32MaxMotorTorque;
            }
            else if ( gRotorSpeed.SpeedApply >= gMotorInfo.Frequency )
            {
                gAsr.Asr.MaxPIOut   =   gf32MaxMotorTorque;
                gAsr.Asr.MinPIOut   =   -f32MaxFeedbackTorque;
            }
            else
            {
                gAsr.Asr.MaxPIOut   =   gMotorInfo.MaxTorque;
                gAsr.Asr.MinPIOut   =   -gAsr.Asr.MaxPIOut;
            }
        }
        else
        {
            gAsr.Asr.MaxPIOut   =   gMotorInfo.MaxTorque;
            gAsr.Asr.MinPIOut   =   -gAsr.Asr.MaxPIOut;
        }
    }
    else
    {
        if (IS_POSITIVE_ROTATE_WITH_SPEED_N)
        {
            gAsr.Asr.MaxPIOut   =   -gDmdCanRe.MinTroqLimit;//MaxTroqLimit;
            gAsr.Asr.MinPIOut   =   -gDmdCanRe.MaxTroqLimit;//MinTroqLimit;
        }
        else
        {
            gAsr.Asr.MaxPIOut   =   gDmdCanRe.MaxTroqLimit;//MaxTroqLimit;
            gAsr.Asr.MinPIOut   =   gDmdCanRe.MinTroqLimit;//MinTroqLimit;
        }

    }

    gAsr.Asr.ErrLmt = 0.5f*gMotorInfo.Frequency;


    //转速环温度限扭
    if(gAsr.Asr.MinPIOut < -g_DerateTorq)
    {
        gAsr.Asr.MinPIOut = -g_DerateTorq;
    }

    if(gAsr.Asr.MaxPIOut > g_DerateTorq)
    {
        gAsr.Asr.MaxPIOut = g_DerateTorq;
    }


    if( 1 == gPMEstControl.RunBemfEst )   //参数辨识功能
    {
        gAsr.Asr.Ref = gPMEstControl.BemfEstSpdRef;

    }
    else
    {
        gAsr.Asr.Ref = g_freqSet;//gMainCmd.FreqSet;
    }

    gAsr.Asr.Fb = gRotorSpeed.SpeedApply;


    gASR_KP = gAsr.Asr.Kp * 100;
    gASR_Ki = gAsr.Asr.Ki * 100;

    gASR_SpdGmd =  gAsr.Asr.Ref * 10;
    gASR_SpdFb =  gAsr.Asr.Fb * 10;

    PIRegulator(&gAsr.Asr);



//    if (gDmdCanRe.EnableReq == 1)
//    {
//        gAsr.Asr.PIOut = 5;//
//    }
//    else
//    {
//        gAsr.Asr.PIOut = 0;//
//    }



    //gAsr.OutFilterCoef
    gAsr.AsrOutFilter = 0.98f*gAsr.AsrOutFilter + ( 1 - 0.98f )*gAsr.Asr.PIOut;  //用于扭矩控制的功能

    if( 1 == gPMEstControl.RunBemfEst )  //参数辨识功能
    {
        if( gPMEstControl.RunBemfEstStep >= 4 )
        {
            if( gIMTSet.M > -gPMEstControl.RotorZeroCompensateIdMax )
            {
                gIMTSet.M -= 0.25f;
            }
            else
            {
                gIMTSet.M = -gPMEstControl.RotorZeroCompensateIdMax;
            }
        }
        else
        {
            gIMTSet.M = 0;
        }

        gIMTSet.T = gAsr.Asr.PIOut;
    }
    else
    {



        if ( 0 == IS_CALIBRATED )                                //非查表整车运行程序
        {
            if ( gIMTSet.T >= 0 )
            {
                if( gPMFwCtrl.StopIncIqByIdRef || gPMFwCtrl.StopIncIqByVout )
                {
                    if ( gAsr.Asr.PIOut > gIMTSet.T )
                    {
                        gAsr.Asr.PIOut = gIMTSet.T;
                    }
                    else
                    {
                        gIMTSet.T = gAsr.Asr.PIOut;
                    }
                }
                else
                {
                    gIMTSet.T = gAsr.Asr.PIOut;
                }
            }
            else
            {
                if( gPMFwCtrl.StopIncIqByIdRef || gPMFwCtrl.StopIncIqByVout )
                {
                    if ( gAsr.Asr.PIOut < gIMTSet.T )
                    {
                        gAsr.Asr.PIOut = gIMTSet.T;
                    }
                    else
                    {
                        gIMTSet.T = gAsr.Asr.PIOut;
                    }
                }
                else
                {
                    gIMTSet.T = gAsr.Asr.PIOut;
                }
            }
        }
    }
}


/*************************************************************
	矢量控制的速度环－－叉车版
*************************************************************/
double PI_Error = 0;
double PI_Error_Z1 = 0;
double KP_Result = 0;
double KI_Result = 0;
int count1 = 0;
int count1_limit = 9;
int KI_Reset_Flag1 = 0;
int Flag_Fast2Zero = 0;
int Flag_Fast2Zero_Spd = 0;
int Flag_De =0,Flag_In = 0;
int AntiCnt = 0, AntiCnt1 = 0;
int Flag_Anti = 0;
int slopecnt = 0,slope_stopcnt=0;
int SlopOut_cnt = 0;
int slopecnt1 = 0,slope_stopcnt1=0;
int SlopOut_cnt1 = 0;
int Flag_Behind = 0,Flag_Front = 0;
int BrakeSignalTest_old = 0,FlagRest = 0,Flag_Trigger = 0;
double Pre_LoopOut = 0,PowerLimit = 0;
double ChgUp_Rate =2;
double ChgDown_Rate = 6;
double ChgUp_Rate_Spd =3;
double ChgDown_Rate_Spd = 10;
double ChgDown_Rate_Spd1 = 1;
double ChgUp_Rate_Spd1 =1;
double Brk_DownRate = 0.0;
double ErrLmt_Calibrate = 50;
float MaxRegen_Torq = 125;
float gMainCmd_FreqSet_old = 0,gMainCmd_FreqSet_Pre = 0;
float gMainFb_Speed_old = 0;
int Cnt_clear = 0;
int KeyEnable_old = 0,Flag_SpdReduce = 0,Flag_Stop = 0,Flag_SpdReduce1 = 0;
int ACC_Enable = 0,Gear_information = 0;
int ACC_Release_flag = 0;

static int16_t IsSpeedDecreasingEx1( void )
{
	int16_t ret = 0;

	if( gRotorSpeed.SpeedApplyFilter500msZ2 > gRotorSpeed.SpeedApplyFilter500msZ1
	 && gRotorSpeed.SpeedApplyFilter500msZ1 > gRotorSpeed.SpeedApplyFilter500ms
	 && gRotorSpeed.SpeedApplyFilter500ms >= 0 )
	{
		ret = 2;//正转减速
	}
	else if( gRotorSpeed.SpeedApplyFilter500msZ2 < gRotorSpeed.SpeedApplyFilter500msZ1
	 && gRotorSpeed.SpeedApplyFilter500msZ1 < gRotorSpeed.SpeedApplyFilter500ms
	 && gRotorSpeed.SpeedApplyFilter500ms <= 0 )
	{
		ret = 1;//负转减速
	}

	return ret;
}

void VCAsrControlFL(void)
{
	float f32MaxFeedbackTorque = 0;
	Saturatefloat(&MaxRegen_Torq, 200, 0);

	//刹车状态，目标速度步进值处理
    if (fabs(gAsr.Asr.Ref)<150)
    {
        Brk_DownRate = 0.1;
    }
    else if (fabs(gAsr.Asr.Ref)>200)
    {
        Brk_DownRate = 1.5;

    }
    else
    {
        Brk_DownRate = 0.1+(fabs(gAsr.Asr.Ref)-150)*0.028;
        if (Brk_DownRate > 1.5)
        {
            Brk_DownRate = 1.5;
        }
        else if (Brk_DownRate < 0.1 )
        {
            Brk_DownRate = 0.1;
        }
        else
        {
            ;
        }
    }


    ///驻坡状态，扭矩加载速率处理
	if (Flag_Anti == 1)
	{
		ChgUp_Rate = 15;
		ChgDown_Rate = 35;
		if(fabs(gRotorSpeed.SpeedApplyFilter)>120)
		{
			ChgUp_Rate =6;
			ChgDown_Rate = 12;
		}
	}
	else
	{
		 ChgUp_Rate =6;
		 ChgDown_Rate = 12;


	}

	//目标速度步进值处理
    ChgUp_Rate_Spd =5;
    ChgDown_Rate_Spd = 17;


    //驻坡状态，最大扭矩切换
	if (Flag_Anti == 0)
	{
		MaxRegen_Torq = gMotorInfo.MaxTorque;//125;

	}
	else
	{
		MaxRegen_Torq = gMotorInfo.MaxTorque;//125;
	}

    if( 1 == g_TorqueCtlFlag/*gComPar.SubCommand.bit.TorqueCtl*/ )
    {
        if( 1 == g_u16HighSpeedRegulator )   //高速限速使能
        {
            f32MaxFeedbackTorque = gUDC.uDCBigFilter*g_MaxFeedbackCurr*gMotorInfo.Poles/( Value_2Pi*fabs(gRotorSpeed.SpeedApply) );
            if ( f32MaxFeedbackTorque > gf32MaxGeneratorTorque )
            {
                f32MaxFeedbackTorque = gf32MaxGeneratorTorque;
            }

            if ( gRotorSpeed.SpeedApply <= -gMotorInfo.Frequency )
            {
                gAsr.Asr.MaxPIOut   =   f32MaxFeedbackTorque;
                gAsr.Asr.MinPIOut   =   -gf32MaxMotorTorque;
            }
            else if ( gRotorSpeed.SpeedApply >= gMotorInfo.Frequency )
            {
                gAsr.Asr.MaxPIOut   =   gf32MaxMotorTorque;
                gAsr.Asr.MinPIOut   =   -f32MaxFeedbackTorque;
            }
            else
            {
                gAsr.Asr.MaxPIOut   =   gMotorInfo.MaxTorque;
                gAsr.Asr.MinPIOut   =   -gAsr.Asr.MaxPIOut;
            }
        }
        else
        {
            gAsr.Asr.MaxPIOut   =   gMotorInfo.MaxTorque;
            gAsr.Asr.MinPIOut   =   -gAsr.Asr.MaxPIOut;
        }
    }
    else
    {
		if( 1 == IS_POSITIVE_ROTATE_WITH_SPEED_N )
		{
			if (gRotorSpeed.SpeedApplyFilter > 0)
			{

				gAsr.Asr.MaxPIOut	=	-gDmdCanRe.MinTroqLimit;
				if (fabs(MaxRegen_Torq)<fabs(gDmdCanRe.MaxTroqLimit))
				{
					gAsr.Asr.MinPIOut	=	-MaxRegen_Torq;
				}
				else
				{
					gAsr.Asr.MinPIOut	=	-gDmdCanRe.MaxTroqLimit;
				}
			}
			else
			{
				if(fabs(MaxRegen_Torq)<fabs(gDmdCanRe.MinTroqLimit))
				{
					gAsr.Asr.MaxPIOut	=	MaxRegen_Torq;
				}
				else
				{
					gAsr.Asr.MaxPIOut = -gDmdCanRe.MinTroqLimit;
				}
				gAsr.Asr.MinPIOut	=	-gDmdCanRe.MaxTroqLimit;

			}

		}
		else
		{
			if (gRotorSpeed.SpeedApply >= 0)
			{

				gAsr.Asr.MaxPIOut	=	gDmdCanRe.MaxTroqLimit;

				if (fabs(MaxRegen_Torq)<fabs(gDmdCanRe.MinTroqLimit))
				{
					gAsr.Asr.MinPIOut	= -MaxRegen_Torq;
				}
				else
				{
					gAsr.Asr.MinPIOut = gDmdCanRe.MinTroqLimit;
				}
			}
			else
			{

				if (MaxRegen_Torq < gDmdCanRe.MaxTroqLimit)
				{
					gAsr.Asr.MaxPIOut	=	MaxRegen_Torq;
				}
				else
				{
					gAsr.Asr.MaxPIOut = gDmdCanRe.MaxTroqLimit;
				}


				gAsr.Asr.MinPIOut	=	gDmdCanRe.MinTroqLimit;

			}

		}

	}

	if(gAsr.Asr.MinPIOut < -g_DerateTorq)
	{
		gAsr.Asr.MinPIOut = -g_DerateTorq;
	}

	if(gAsr.Asr.MaxPIOut > g_DerateTorq)
	{
		gAsr.Asr.MaxPIOut = g_DerateTorq;
	}



	//加速状态判断
	if( gRotorSpeed.SpeedApplyFilter500msZ2 < gRotorSpeed.SpeedApplyFilter500msZ1
	 && gRotorSpeed.SpeedApplyFilter500msZ1 < gRotorSpeed.SpeedApplyFilter500ms
	 && gRotorSpeed.SpeedApplyFilter500msZ2 >= 0 )
	{
		Flag_In = 1; //正转加速
	}
	else if( gRotorSpeed.SpeedApplyFilter500msZ2 > gRotorSpeed.SpeedApplyFilter500msZ1
	 && gRotorSpeed.SpeedApplyFilter500msZ1 > gRotorSpeed.SpeedApplyFilter500ms
	 && gRotorSpeed.SpeedApplyFilter500msZ2 <= 0 )

	{
		Flag_In = 2; //反转加速
	}
	else
	{
		Flag_In = 0;
	}


	if((BrakeSignalTest == 1) || fabs(g_freqSet)>100)
	{
		;
	}
	else
	{
	    //驻坡状态判断
        if ((((Flag_In == 1)&&((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(gPowerTrq.TrqOut < -3)) || ((Flag_In == 2)&&((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(gPowerTrq.TrqOut >= 3)))||(((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(fabs(gPowerTrq.TorqueRef)>=10)&&(fabs(gRotorSpeed.SpeedApplyFilter)<1)))
        //if ((((Flag_In == 1)&&((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(gPowerTrq.TrqOut < 1)) || ((Flag_In == 2)&&((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(gPowerTrq.TrqOut >= -1)))||(((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(fabs(gPowerTrq.TorqueRef)>=1)&&(fabs(gRotorSpeed.SpeedApplyFilter)<1)))

        {
            AntiCnt++;
        }
        else
        {
            AntiCnt = 0;
        }
        if (AntiCnt >=45)
        {
            AntiCnt = 50;
            Flag_Anti = 1;
        }

        if((((Flag_In == 1)&&((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(gPowerTrq.TrqOut < 0))||(((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(gPowerTrq.TorqueRef<=1)&&(fabs(gRotorSpeed.SpeedApplyFilter)<1)))&&(Flag_Front == 0))
        //if((((Flag_In == 1)&&((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(gPowerTrq.TrqOut < 1))||(((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(gPowerTrq.TorqueRef<=1)&&(fabs(gRotorSpeed.SpeedApplyFilter)<1)))&&(Flag_Front == 0))
        {

        if (Flag_Anti == 1)
        {
            Flag_Behind = 1;
            Flag_Front = 0;
        }
        else
        {
            Flag_Behind = 0;
        }
        }

        if ((((Flag_In == 2)&&((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(gPowerTrq.TrqOut >= 0))||(((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(gPowerTrq.TorqueRef>=-1)&&(fabs(gRotorSpeed.SpeedApplyFilter)<1)))&&(Flag_Behind==0))
        //if ((((Flag_In == 2)&&((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(gPowerTrq.TrqOut >= -1))||(((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&(gPowerTrq.TorqueRef>=-1)&&(fabs(gRotorSpeed.SpeedApplyFilter)<1)))&&(Flag_Behind==0))
        {
        if (Flag_Anti == 1)
        {
            Flag_Front = 1;
            Flag_Behind = 0;
        }
        else
        {
            Flag_Front = 0;
        }

        }

        if(((fabs(g_freqSet) < 1)||(ACC_Enable == 0))&&((fabs(gRotorSpeed.SpeedApplyFilter)<1))&&(fabs(gPowerTrq.TrqOut) >= 30))
        {
            AntiCnt1++;
            if(AntiCnt1>=45)
            {
                AntiCnt1 = 50;
                AntiCnt = 50;
                Flag_Anti = 1;
//                Flag_Behind = 1;
            }

        }
        else
        {
            AntiCnt1 = 0;
        }

	}

    gAsr.Asr.ErrLmt	= ErrLmt_Calibrate;//0.1f*gMotorInfo.Frequency;

    if( 1 == gPMEstControl.RunBemfEst )
    {
        gAsr.Asr.Ref = gPMEstControl.BemfEstSpdRef;
    }
    else
    {
        //目标转速处理
		if ((Flag_Anti == 0) || ( (ACC_Enable == 1) && (GearD_Enable ==1 || GearR_Enable ==1)))  //非驻坡状态下
		{
			slopecnt = 0;
			slope_stopcnt = 0;
			slopecnt1 = 0;
			slope_stopcnt1 = 0;
			Flag_Behind = 0;
			Flag_Front = 0;

			if ((BrakeSignalTest == 0) && (Flag_SpdReduce1 == 0))//非刹车
			{
				if(ACC_Enable == 0)
				{
					if ((gAsr.Asr.Ref > g_freqSet) && (g_freqSet >= 0))
					{
						gAsr.Asr.Ref  = g_freqSet;
					}
					else if ((gAsr.Asr.Ref < g_freqSet) && (g_freqSet <= 0))
					{
						gAsr.Asr.Ref  = g_freqSet;
					}
					else
					{
						;
					}
				}
				if (Flag_Fast2Zero_Spd == 1)
				{
					gAsr.Asr.Ref  = 0;
				}
				else if (gAsr.Asr.Ref  > 0)
				{
					if((g_freqSet -gAsr.Asr.Ref)> ChgUp_Rate_Spd)
					{
						gAsr.Asr.Ref  = gAsr.Asr.Ref  + ChgUp_Rate_Spd;
					}
					else if ((gAsr.Asr.Ref  - g_freqSet) > ChgDown_Rate_Spd)
					{
						gAsr.Asr.Ref  = gAsr.Asr.Ref - ChgDown_Rate_Spd;
					}
					else
					{
						gAsr.Asr.Ref  = g_freqSet;
					}
				}
				else
				{
					if((gAsr.Asr.Ref  - g_freqSet )> ChgUp_Rate_Spd)
					{
						gAsr.Asr.Ref  = gAsr.Asr.Ref  - ChgUp_Rate_Spd;
					}
					else if ((g_freqSet - gAsr.Asr.Ref ) > ChgDown_Rate_Spd)
					{
						gAsr.Asr.Ref  = gAsr.Asr.Ref  + ChgDown_Rate_Spd;
					}
					else
					{
						gAsr.Asr.Ref  = g_freqSet;
					}
				}

		   }
		   else                                              //刹车
		   {
				if ((gAsr.Asr.Ref > g_freqSet) && (g_freqSet >= 0))
				{
					gAsr.Asr.Ref  = g_freqSet;
				}
				else if ((gAsr.Asr.Ref < g_freqSet) && (g_freqSet <= 0))
				{
					gAsr.Asr.Ref  = g_freqSet;
				}
				else
				{
					;
				}

//				 if (gAsr.Asr.Ref  > 0)
//				{
//					if (gAsr.Asr.Ref  > Brk_DownRate)
//					{
//						gAsr.Asr.Ref  = gAsr.Asr.Ref - Brk_DownRate;
//					}
//					else
//					{
//						gAsr.Asr.Ref  = 0;
//					}
//				}
//				else
//				{
//					if ((0- gAsr.Asr.Ref ) > Brk_DownRate)
//					{
//						gAsr.Asr.Ref  = gAsr.Asr.Ref  + Brk_DownRate;
//					}
//					else
//					{
//						gAsr.Asr.Ref  = 0;
//					}
//				}
		   }
		}
		else                                                         //驻坡状态下
		{
			count2 = 0;
			if (BrakeSignalTest == 0)//非刹车
			{
				if (Flag_Behind == 1)
				{
                    if ((gPowerTrq.TrqOut < -12)&&(fabs(gRotorSpeed.SpeedApplyFilter) <1) )
                    {
                        Flag_Fast2Zero = 0;
                        slopecnt++;        //驻坡计数
                    }
                    //else if ((gPowerTrq.TrqOut > 5) || (fabs(g_freqSet)>2))
                    else if ((gPowerTrq.TrqOut > 5))
                    {
                        slope_stopcnt ++;
                    }
                    else
                    {
                        slope_stopcnt = 0;
                    }

                    if (slope_stopcnt > 21)
                    {
                        slope_stopcnt = 22;
                        slopecnt = 0;
                    }

                    if (slopecnt > AntiTime) //300
                    {
                        Flag_Fast2Zero = 0;
                        slopecnt = AntiTime + 100; //390
                        gAsr.Asr.Ref  = g_freqSet;

                    }
                    else
                    {
                        gAsr.Asr.Ref  = g_freqSet;
                        Flag_Fast2Zero = 0;

                    }

				}
				else if ((Flag_Front == 1) || ((Flag_Behind == 0) && (Flag_Anti == 1)))
				{
                    if ((gPowerTrq.TrqOut >12)&&(fabs(gRotorSpeed.SpeedApplyFilter) <1))
                    {
                        Flag_Fast2Zero = 0;
                        slopecnt1++;
                    }
                    //else if ((gPowerTrq.TrqOut <-5) || (fabs(g_freqSet)>2))
                    else if ((gPowerTrq.TrqOut <-5))
                    {
                        slope_stopcnt1 ++;
                    }
                    else
                    {
                        slope_stopcnt1 = 0;
                    }

                    if (slope_stopcnt1 > 21)
                    {
                        slope_stopcnt1 = 22;
                        slopecnt1 = 0;
                    }

                    if (slopecnt1 > AntiTime)
                    {
                        Flag_Fast2Zero = 0;
                        slopecnt1 = AntiTime + 100;
                        gAsr.Asr.Ref  = g_freqSet;
                    }
                    else
                    {
                        gAsr.Asr.Ref  = g_freqSet;
                        Flag_Fast2Zero = 0;
                    }

				}
				else
				{
					gAsr.Asr.Ref  = g_freqSet;
				}
			}
			else
			{
				gAsr.Asr.Ref  = g_freqSet;
				slopecnt = 0;
				slope_stopcnt = 0;
				slopecnt1 = 0;
				slope_stopcnt1 = 0;
			}
		}

		if ((Flag_SpdReduce1 == 1) && (Flag_Anti == 0))
		{
            if ((gAsr.Asr.Ref > g_freqSet) && (g_freqSet >= 0))
            {
                gAsr.Asr.Ref  = g_freqSet;
            }
            else if ((gAsr.Asr.Ref < g_freqSet) && (g_freqSet <= 0))
            {
                gAsr.Asr.Ref  = g_freqSet;
            }
            else
            {
                ;
            }

//            if (gAsr.Asr.Ref  > 0)
//            {
//                if (gAsr.Asr.Ref  > Brk_DownRate)
//                {
//                    gAsr.Asr.Ref  = gAsr.Asr.Ref - Brk_DownRate;
//                }
//                else
//                {
//                    gAsr.Asr.Ref  = 0;
//                }
//            }
//            else
//            {
//                if ((0- gAsr.Asr.Ref ) > Brk_DownRate)
//                {
//                    gAsr.Asr.Ref  = gAsr.Asr.Ref  + Brk_DownRate;
//                }
//                else
//                {
//                    gAsr.Asr.Ref  = 0;
//                }
//            }

		}

		gAsr.Asr.Fb =gRotorTrans.FreqFeed;

    	//位置式PI计算
		PI_Error = gAsr.Asr.Ref -gAsr.Asr.Fb;

        if(PI_Error > gAsr.Asr.ErrLmt)
        {
            PI_Error = gAsr.Asr.ErrLmt;
        }
        else if ( PI_Error < -gAsr.Asr.ErrLmt )
        {
            PI_Error = -gAsr.Asr.ErrLmt;
        }

        KP_Result = gAsr.Asr.Kp*PI_Error;

        KI_Result += gAsr.Asr.Ki*PI_Error;

        //Ki限制
#if 0
        if ((fabs(gRotorSpeed.SpeedApplyFilter) > 60) && (fabs(gRotorSpeed.SpeedApplyFilter) < 350))
            {
                PowerLimit = 22300/(fabs(gRotorSpeed.SpeedApplyFilter));
            }
            else
            {
                PowerLimit = 200;
            }
#endif
        if ((fabs(gRotorSpeed.SpeedApplyFilter) > 60) && (fabs(gRotorSpeed.SpeedApplyFilter) < 350))
        {

//        	if (  (fabs(gRotorSpeed.SpeedApplyFilter) > 40) && (fabs(gRotorSpeed.SpeedApplyFilter) < 200) //满载爬坡速度限制到900rpm
//        	   && ((gRotorSpeed.SpeedApplyFilter * gAsr.Asr.PIOut) >= 0)//区分回馈和驱动，防止爬坡后退力矩不够，后溜失控
//			   )
//        	{
//                PowerLimit = 0.9*PowerLimit + 650/(fabs(gRotorSpeed.SpeedApplyFilter));
//        	}
//        	else
//        	{
                PowerLimit = 0.9*PowerLimit + 700/(fabs(gRotorSpeed.SpeedApplyFilter));//800 //1000//1500 //1040 //650
//        	}
        }
        else
        {
            PowerLimit = 200;
        }

        if (PowerLimit > 200)
        {
            PowerLimit = 200;
        }

        if (gRotorSpeed.SpeedApplyFilter>10)
        {
                if (KI_Result > PowerLimit)
                {
                    KI_Result = PowerLimit;
                }
                if(KI_Result < -PowerLimit)
                {
                    KI_Result = -PowerLimit;
                }
        }

        if (gRotorSpeed.SpeedApplyFilter < -10)
        {
            if (KI_Result > PowerLimit)
            {
                KI_Result = PowerLimit;
            }
            if (KI_Result < -PowerLimit)
            {
                KI_Result = -PowerLimit;
            }
        }

        Flag_De = IsSpeedDecreasingEx1();//减速标志


        //减速处理
        if (  ((fabs(g_freqSet)<20  && ((fabs(g_freqSet)-fabs(gMainCmd_FreqSet_old))<=0))||(ACC_Enable == 0))
           && (  (fabs(gRotorSpeed.SpeedApplyFilter)< (7+Flag_SpdReduce1*15))
        	  && (  (Flag_De ==1 && (Pre_LoopOut>3  || (Pre_LoopOut>3 && BrakeSignalTest == 1)) )
        		 || (Flag_De ==2 && ((Pre_LoopOut<-3 && Pre_LoopOut>-45) || (Pre_LoopOut<-3 && BrakeSignalTest == 1)))//爬坡倒车减速
				 )
			  )
		   && (ACC_Enable == 0 )
		   )
        {
            count1++;//减速计算
        }
        else
        {
            count1 = 0;
        }


        if ((count1>=count1_limit)&& ( Flag_Anti==0))//----2.5t
        {
			KI_Reset_Flag1 =1;
			count1 = count1_limit+5;

        	if (g_ResetKIFlag == 0)  //头朝下下坡，误清扭矩，扭矩恢复
        	{
                g_RecoverKI = KI_Result;
                g_ResetKIFlag = 1;
        	}

			KI_Result = KI_Result *0.95;
			KP_Result = KP_Result *0.95;//2.5t
        }
        else if ((fabs(g_freqSet)-fabs(gMainCmd_FreqSet_old))> 0 || ACC_Enable == 1||( Flag_Anti==1)|| ( fabs(gRotorSpeed.SpeedApplyFilter)>(10))) //----2.5T
        {
            KI_Reset_Flag1 = 0;     //重新加速恢复
            count1 = 0;
            count2 = 0;

            g_RecoverKI = 0;
            g_ResetKIFlag = 0;
        }
        else
        {
            ;
        }

        gMainCmd_FreqSet_old = g_freqSet;

        //即停时，清除KI
        if ((KI_Reset_Flag1==1) && (fabs(gAsr.Asr.PIOut)<=3) && (BrakeSignalTest == 0))
        {
            KI_Result = 0;
			gAsr.Asr.Kp = 0.5f;

            Flag_Fast2Zero = 1;
            if((fabs(gRotorSpeed.SpeedApplyFilter)<=3))
            {
                Flag_Anti = 1;
                Flag_Front=1;
                KI_Reset_Flag1 = 0;
                count1 = 0;

                g_RecoverKI = 0;
                g_ResetKIFlag = 0;
//                gMainFb_Speed_old = 0;
            }

        }
        else
        {
            Flag_Fast2Zero = 0;
        }


        //停车，清除KPKI
        if ((  g_McuModeRe != SPEEDCTRL_CMD)||(1 == Flag_Fast2Zero)||(Cnt_clear>1500))
        {
            ResetAsrPar();
            KI_Result = 0;

			gAsr.Asr.Kp = 0.5f;
//            KP_Result = 0;
//            gAsr.Asr.PIOut = KP_Result;
        }
#if 0
        if (((BrakeSignalTest == 1) || (Flag_SpdReduce == 1)) && ( Flag_Anti==0))
            {
                if (gRotorSpeed.SpeedApplyFilter >= 0 )
                {
                    gAsr.Asr.MaxPIOut = 0;
                    if(KI_Result > 0)
                    {
                        KI_Result = 0;
                    }
                }
                else if (gRotorSpeed.SpeedApplyFilter <= 0)
                {
                    gAsr.Asr.MinPIOut = 0;
                    if(KI_Result < 0)
                    {
                        KI_Result = 0;
                    }
                }
           }
#endif

        //即停
    	if(Flag_Stop == 1)
    	{
    		if(fabs(KI_Result) > 40)
    		{
    			KI_Result = 0.5*KI_Result;//车辆已判断即停，如果KI还很大，进行减半
    		}
    		else
    		{
    			if ((GearD_Enable == 1) && (gRotorSpeed.SpeedApplyFilter > 0))  //预防回弹
    			{
    				gAsr.Asr.Kp = 7.5f;
    			}
    			else if ((GearR_Enable == 1) && (gRotorSpeed.SpeedApplyFilter < 0))
    			{
    				gAsr.Asr.Kp = 7.5f;
    			}
    			else
    			{
    				gAsr.Asr.Kp = 0.5f;

    				ResetAsrPar();            //减半KI小于40，直接清零，转速环参数复位
//    				KP_Result = 0;
    				KI_Result = 0;
    				Pre_LoopOut = 0;
    				gAsr.AsrOutFilter = 0;
    				gAsr.Asr.PIOut = 0;
    			}

    		}
    	}

        //KiKP输出限制
        if (KI_Result> gAsr.Asr.MaxPIOut)
        {
            KI_Result = gAsr.Asr.MaxPIOut;
        }
        else if (KI_Result < gAsr.Asr.MinPIOut )
        {
            KI_Result = gAsr.Asr.MinPIOut;
        }


		if (BrakeSignalTest == 1)
		{
			if(fabs(gAsr.Asr.Fb) > fabs(gAsr.Asr.Ref))
			{
		        Pre_LoopOut = KP_Result + KI_Result;
			}
			else
			{
				KP_Result = 0;
				KI_Result = gAsr.Asr.PIOut;
		        Pre_LoopOut = KI_Result;
			}

		}
		else
		{
	        Pre_LoopOut = KP_Result + KI_Result;
		}

        if (gRotorSpeed.SpeedApplyFilter>10)
        {
            if (Pre_LoopOut > PowerLimit)
            {
                Pre_LoopOut = PowerLimit;
            }
            if (Pre_LoopOut < -PowerLimit)
            {
                Pre_LoopOut = -PowerLimit;
            }
        }
        else if (gRotorSpeed.SpeedApplyFilter < -10)
        {
            if (Pre_LoopOut > PowerLimit)
            {
                Pre_LoopOut = PowerLimit;
            }
            if (Pre_LoopOut < -PowerLimit)
            {
                Pre_LoopOut = -PowerLimit;
            }

            if(Pre_LoopOut > gf32MaxGeneratorTorque)
            {
                Pre_LoopOut = gf32MaxGeneratorTorque;
            }

            if(Pre_LoopOut < -gf32MaxMotorTorque)
            {
                Pre_LoopOut = -gf32MaxMotorTorque;
            }
        }
        else
        {
            if(ACC_Enable == 0)
            {
                if (Pre_LoopOut > PowerLimit)
                {
                    Pre_LoopOut = PowerLimit;
                }
                if (Pre_LoopOut < -PowerLimit)
                {
                    Pre_LoopOut = -PowerLimit;
                }

            }
        }

        //扭矩加减载
        if(ACC_Release_flag == 1)
        {
            ACC_Release_flag = 0;
            if(gAsr.Asr.PIOut  > 0)
            {
                if((gAsr.Asr.PIOut> gPowerTrq.TrqOut)&&(fabs(fabs(gAsr.Asr.PIOut)- fabs(gPowerTrq.TrqOut))>50))
                {
                    gAsr.Asr.PIOut = gPowerTrq.TrqOut;
                }
            }
            else
            {
                if((gAsr.Asr.PIOut < gPowerTrq.TrqOut)&&(fabs(fabs(gAsr.Asr.PIOut)- fabs(gPowerTrq.TrqOut))>50))
                {
                    gAsr.Asr.PIOut = gPowerTrq.TrqOut;
                }
            }
//            KI_Result = gAsr.Asr.PIOut;
        }

        if (Flag_Fast2Zero == 1)
        {
            gAsr.Asr.PIOut	= KP_Result;
        }
        else if (gAsr.Asr.PIOut  > 0)
        {
            if((Pre_LoopOut -gAsr.Asr.PIOut  )> ChgUp_Rate)
            {
                gAsr.Asr.PIOut	= gAsr.Asr.PIOut  + ChgUp_Rate;
            }
            else if ((gAsr.Asr.PIOut  - Pre_LoopOut) > ChgDown_Rate)
            {
                gAsr.Asr.PIOut	= gAsr.Asr.PIOut - ChgDown_Rate;
            }
            else
            {
                gAsr.Asr.PIOut	= Pre_LoopOut;
            }
        }
        else
        {
            if((gAsr.Asr.PIOut	- Pre_LoopOut )> ChgUp_Rate)
            {
                gAsr.Asr.PIOut	= gAsr.Asr.PIOut  - ChgUp_Rate;
            }
            else if ((Pre_LoopOut - gAsr.Asr.PIOut ) > ChgDown_Rate)
            {
                gAsr.Asr.PIOut	= gAsr.Asr.PIOut  + ChgDown_Rate;
            }
            else
            {
                gAsr.Asr.PIOut	= Pre_LoopOut;
            }
        }

        //PI限制
        if (gAsr.Asr.PIOut> gAsr.Asr.MaxPIOut)
        {
            gAsr.Asr.PIOut = gAsr.Asr.MaxPIOut;
        }
        else if (gAsr.Asr.PIOut < gAsr.Asr.MinPIOut )
        {
            gAsr.Asr.PIOut = gAsr.Asr.MinPIOut;
        }

		gAsr.AsrOutFilter = 0.98f*gAsr.AsrOutFilter + ( 1 - 0.98f )*gAsr.Asr.PIOut;


		//标定与参数辨识处理
		if( 1 == gPMEstControl.RunBemfEst )
		{
			if( gPMEstControl.RunBemfEstStep >= 4 )
			{
				if( gIMTSet.M > -gPMEstControl.RotorZeroCompensateIdMax )
				{
					gIMTSet.M -= 0.25f;
				}
				else
				{
					gIMTSet.M = -gPMEstControl.RotorZeroCompensateIdMax;
				}
			}
			else
			{
				gIMTSet.M = 0;
			}

			gIMTSet.T = gAsr.Asr.PIOut;
		}
		else
		{
			if ( 0 == IS_CALIBRATED )
			{
				if ( gIMTSet.T >= 0 )
				{
					if( gPMFwCtrl.StopIncIqByIdRef || gPMFwCtrl.StopIncIqByVout )
					{
						if ( gAsr.Asr.PIOut > gIMTSet.T )
						{
							gAsr.Asr.PIOut = gIMTSet.T;
						}
						else
						{
							gIMTSet.T = gAsr.Asr.PIOut;
						}
					}
					else
					{
						gIMTSet.T = gAsr.Asr.PIOut;
					}
				}
				else
				{
					if( gPMFwCtrl.StopIncIqByIdRef || gPMFwCtrl.StopIncIqByVout )
					{
						if ( gAsr.Asr.PIOut < gIMTSet.T )
						{
							gAsr.Asr.PIOut = gIMTSet.T;
						}
						else
						{
							gIMTSet.T = gAsr.Asr.PIOut;
						}
					}
					else
					{
						gIMTSet.T = gAsr.Asr.PIOut;
					}
				}
			}
		}
    }

}


//转速环数据复位
void ResetAsrPar( void )
{
    gAsr.Asr.PIOut  =   0;
    gAsr.Asr.Err    =   0;
    gAsr.Asr.ErrLmt =   100;
    gAsr.Asr.Ref    =   0;

    gAsr.AsrOutFilter = 0;
}
