#include <string.h>
#include "f_public.h"
#include "MotorCalibration.h"
#include "main.h"

extern float gIdTSetM;
extern float gIqTSetM;
uint16_t g_u16SpeedToTorqExitCnt = 0;

#if   ( CODE_MODE == CODE_MODE_CALIBRATED_ON_ROAD_RUNNING )
void CalibratedOnRoadRunning( void );            //整车程序
#endif

#if   ( CODE_MODE == CODE_MODE_CALIBRATED_ON_TEST_BENCH_RUNNING )
void CalibratedOnBenchRunning( void );           //测试程序
#endif

#if   ( CODE_MODE == CODE_MODE_UNCALIBRATED_ON_ROAD_RUNNING )
void UncalibratedOnRoadRunning( void );          //非查表整车程序
#endif

#if ( CODE_MODE == CODE_MODE_UNCALIBRATED )
void RunCalibrationMode( void );                 //标定程序
static uint16_t s_u16CalUseIs = 0;
#endif


void GetSpdOrTorIdIqRef(void);
void RunDataIntit(void);

//#pragma CODE_SECTION(SEQ_ISR_PROCESS,       "ramfuncs")
//#pragma CODE_SECTION(RunCalibrationMode,    "ramfuncs")

/*************************************************************
    IdIq给定
*************************************************************/
float gPIOut = 0;

float gSpeedApply = 0;

void GetSpdOrTorIdIqRef(void)
{
    float temp = 0;
    float IMSet = 0;
    float ITSet = 0;
    static uint16_t RunMode_TorqueCtl_Old = 0;

//    if(  (g_McuStMode != MODE_TORQUECTRL && g_McuStMode != MODE_SPEEDCTRL)
//      || (gFault3rd.all != 0)
//      )
//    {
//        return;
//    }

    if (1 == g_TorqueCtlFlag)
    {

        if( 0 == RunMode_TorqueCtl_Old )
        {
            g_torquecmd = gPowerTrq.TorqueRef;//gAsr.Asr.PIOut;
            g_u16SpeedToTorqExitCnt = 500;
        }

#if   ( CODE_MODE == CODE_MODE_CALIBRATED_ON_ROAD_RUNNING )
            CalibratedOnRoadRunning();                                              //整车程序
#elif ( CODE_MODE == CODE_MODE_CALIBRATED_ON_TEST_BENCH_RUNNING )
            CalibratedOnBenchRunning();                                             //测试程序
#elif ( CODE_MODE == CODE_MODE_UNCALIBRATED_ON_ROAD_RUNNING )
            UncalibratedOnRoadRunning();                                            //非查表整车程序
            PMMaxTorqCtrl(gIMTSet.T);
#else
            RunCalibrationMode();                                                   //标定程序
            if ( 1 == s_u16CalUseIs )
            {
                if( gMTPA.CosVal > 0 ) gMTPA.CosVal = 0;
                gPMFwCtrl.IdForTorq = fabs(gMTPA.Curr)*gMTPA.CosVal; //Is根据转矩角算出的分量Id
            }
            else
            {
                PMMaxTorqCtrl(gIMTSet.T);  //MTPA控制，id计算
            }
#endif
            Saturatefloat(&gIMTSet.M, 0, -2.3f*gMotorInfo.Current);//电机额定电流
            temp = 2.0f*gMotorInfo.Current;    //电机额定电流
            Saturatefloat(&gIMTSet.T, temp, -temp);
    }
    else
    {
        if(0 /*1 == RunMode_TorqueCtl_Old*/ )
        {
            gAsr.Asr.PIOut = gPowerTrq.TorqueRef;//g_torquecmd;
            gAsr.Asr.Err = 0;
            g_freqSet = gRotorSpeed.SpeedApply;
        }

        ForkLiftRunDeal();//叉车速度环处理

//        VCSpeedControl(); //速度环

        PMMaxTorqCtrl(gAsr.Asr.PIOut);  //MTPA控制，id计算

        gPowerTrq.TorqueRef = gAsr.Asr.PIOut;

//        if (gAsr.Asr.PIOut > 10)
//        {
//        	gAsr.Asr.PIOut = 10;
//        }
//
//        if (gAsr.Asr.PIOut < -10)
//        {
//        	gAsr.Asr.PIOut = -10;
//        }

//        gPIOut = 10;
//        gSpeedApply = 10;
        if ( IS_CALIBRATED )
        {
            GetCalibIdqRef(gRotorSpeed.SpeedApply, gAsr.Asr.PIOut+g_TorqRefComp.TRefCompVal, &IMSet, &ITSet);

//            DINT;
            gIMTSet.M = IMSet;
            gIMTSet.T = ITSet;
//            EINT;
        }
        else
        {
//            DINT;
            gIMTSet.M = gPMFwOut.IdSet;
            gIMTSet.T = gAsr.Asr.PIOut;
//            EINT;
        }
    }

    RunMode_TorqueCtl_Old = g_TorqueCtlFlag;//gComPar.SubCommand.bit.TorqueCtl;
}

#if ( CODE_MODE == CODE_MODE_CALIBRATED_ON_ROAD_RUNNING )
void CalibratedOnRoadRunning( void )     //整车程序运行
{
    //高速限速功能
    if (0 == g_u16HighSpeedRegulator )   //高速限速功能未进入
    {
        if( ReadyToEnterHighSpeedRegulator() ) //判断是否达到进入条件
        {
            EnterHighSpeedRegulator();//启动前参数复位
        }
    }

    if ( 1 == g_u16HighSpeedRegulator )//高速限速启动
    {
        RunHighSpeedRegulator();//执行高速限速
    }

    if (1 == g_u16HighSpeedRegulator )
    {
        GetCalibIdqRef(gRotorSpeed.SpeedApply, gAsr.Asr.PIOut, &gIMTSet.M, &gIMTSet.T);
        gPowerTrq.TorqueRef = gAsr.Asr.PIOut;
    }
    else
    {
        if( IS_T_COMP_EN && fabs( gRotorSpeed.SpeedApply ) <= 216 )
        {
            CalcCompTorque();//扭矩补偿
        }
        else
        {
            g_TorqRefComp.TRefCompVal = 0;
        }

        GetCalibIdqRef(gRotorSpeed.SpeedApply, g_torquecmd + g_TorqRefComp.TRefCompVal, &gIMTSet.M, &gIMTSet.T);
        gPowerTrq.TorqueRef = g_torquecmd;     //外发的执行扭矩gPowerTrq.TorqueRef
   }
}
#endif


#if ( CODE_MODE == CODE_MODE_CALIBRATED_ON_TEST_BENCH_RUNNING )
void CalibratedOnBenchRunning( void )   //测试程序使用
{
    g_u16HighSpeedRegulator = 0;

    GetCalibIdqRef(gRotorSpeed.SpeedApply, g_torquecmd, &gIMTSet.M, &gIMTSet.T);
    gPowerTrq.TorqueRef = g_torquecmd;
}
#endif


#if ( CODE_MODE == CODE_MODE_UNCALIBRATED_ON_ROAD_RUNNING )
void UncalibratedOnRoadRunning( void )
{
    float f32Temp = 0;
    float f32IqSet = 0;
    float f32MaxIqSet = 0;

    //高速限速功能
    if (0 == g_u16HighSpeedRegulator )   //高速限速功能未进入
    {
        if( ReadyToEnterHighSpeedRegulator() ) //判断是否达到进入条件
        {
            EnterHighSpeedRegulator();//启动前参数复位
        }
    }

    if ( 1 == g_u16HighSpeedRegulator )//高速限速启动
    {
        RunHighSpeedRegulator();//执行高速限速
    }


    if (1 == g_u16HighSpeedRegulator )
    {
        f32IqSet = gAsr.Asr.PIOut;
    }
    else
    {
        if( fabs( gRotorSpeed.SpeedApply ) <= 150 )   // 100Hz
            CalcCompTorque();         //扭矩补偿
        else
            g_TorqRefComp.TRefCompVal = 0;

        f32IqSet = g_torquecmd + g_TorqRefComp.TRefCompVal;
    }

    f32Temp = 2*gMotorInfo.MaxCurrent*gMotorInfo.MaxCurrent - gIMTSet.M*gIMTSet.M;  //两电流分量矢量和小于电机最大电流
    if ( f32Temp < 0 )
    {
        f32MaxIqSet = 0;
    }
    else
    {
        f32MaxIqSet = sqrt(f32Temp);
    }

    Saturatefloat(&f32IqSet, f32MaxIqSet, -f32MaxIqSet);
    gIMTSet.T = f32IqSet;
    gPowerTrq.TorqueRef = f32IqSet;
}
#endif


#if ( CODE_MODE == CODE_MODE_UNCALIBRATED )
void RunCalibrationMode( void )
{
    float temp = 0;

    //标定模式下设置Iq根据弱磁算法计算Id（根据Is和转矩角设定idiq的方式未使用）
//    if ( gMTPA.i16IqSetting > 0 )
//    {
//        if ( gMTPA.Iq < gMTPA.Curr*gMTPA.SinVal )  //Is*转矩角sin即Iq,但实际上Is默认为0(gMTPA.Curr)，转矩角也默认90，
//        {
//            gIMTSet.T = gMTPA.Curr*gMTPA.SinVal;
//            s_u16CalUseIs = 1;
//        }
//        else
//        {
//            gIMTSet.T = gMTPA.Iq;   //gMTPA.Iq由gMTPA.i16IqSetting得到，上位机的Iq给定
//            s_u16CalUseIs = 0;
//        }
//    }
//    else if ( gMTPA.i16IqSetting < 0 )
//    {
//        if ( gMTPA.Iq > gMTPA.Curr*gMTPA.SinVal )
//        {
//            gIMTSet.T = gMTPA.Curr*gMTPA.SinVal;
//            s_u16CalUseIs = 1;
//        }
//        else
//        {
//            gIMTSet.T = gMTPA.Iq;   //gMTPA.Iq由gMTPA.i16IqSetting得到，上位机的Iq给定
//            s_u16CalUseIs = 0;
//        }
//    }
//    else
//    {
//        gIMTSet.T = gMTPA.Curr*gMTPA.SinVal;//不设置Iq时，Iq默认使用Is根据转矩角的分量，值为0
//        s_u16CalUseIs = 1;
//    }

    gIMTSet.T = gCommProVar.iqSet;

    temp = 2.3f*gMotorInfo.Current;
    Saturatefloat(&gIMTSet.T, temp, -temp);  //标定时的iq给定

}
#endif


//变量上电初始化
void InitForMotorApp(void)
{
    DisableDrive();


    //状态机步骤初始化
//    gMainStatus.RunStep = STATUS_LOW_POWER; //主步骤
//    gMainStatus.SubStep = 1;                //辅步骤

    //故障结构体清零
//    gMainStatus.ErrorCode.all = 0;          //出错标志

    //逐波限流功能开启或关闭
//    gCBCProtect.EnableFlag = 1;             //默认启动逐波限流功能

    //ADC采样复位
    gADCResetTime  =   0;

    //载频初始化
    gFcCal.FcBak = 10000;
    gBasePar.FcSetApply =   10000;  // default 10khz
    gPWM.gPWMPrd = C_INIT_PRD;


    //载频初始化
    gRotorSpeed.SpeedApplyFilter            =   0;
    gRotorSpeed.SpeedApplyFilter500ms       =   0;
    gRotorSpeed.SpeedApplyFilter500msZ1     =   0;
    gRotorSpeed.SpeedApplyFilter500msZ2     =   0;
    gRotorSpeed.SpeedApplyFilter500msCnt    =   0;
    gRotorSpeed.SpeedApplyFilter500msSum    =   0;



//  gCommProVar.UdcCalCoef //母线电压采样校准系数
//	gCommProVar.PhaseCurrCalCoef//相电流采样校准系数
//	gCommProVar.CtrlVolCalCoef //控制电压采样校准系数
    //霍尔基值与零漂初始化
    if( HALL == HALL_200 )
    {
        g_ADConvert.gain.f32Ia   = 1080.0f/4096;  //电流+/-540A，对应AD端口3V/0V
        g_ADConvert.gain.f32Ib   = 1080.0f/4096;
        g_ADConvert.gain.f32Ic   = 1080.0f/4096;
    }
    else if( HALL == HALL_300 )
    {
        g_ADConvert.gain.f32Ia   = 1620.0f/4096;  //电流+/-810A，对应AD端口3V/0V
        g_ADConvert.gain.f32Ib   = 1620.0f/4096;
        g_ADConvert.gain.f32Ic   = 1620.0f/4096;
    }
    else if( HALL == HALL_500 )
    {
        g_ADConvert.gain.f32Ia   = 1250.0f/4096;  //
        g_ADConvert.gain.f32Ib   = 1250.0f/4096;
        g_ADConvert.gain.f32Ic   = 1250.0f/4096;
    }
    else if( HALL == HALL_800 )
    {
        g_ADConvert.gain.f32Ia   = 2500.0f/4096;
        g_ADConvert.gain.f32Ib   = 2500.0f/4096;
        g_ADConvert.gain.f32Ic   = 2500.0f/4096;
    }
    else if( HALL == HALL_600 )
    {
        g_ADConvert.gain.f32Ia   = 1500.0f/4096;
        g_ADConvert.gain.f32Ib   = 1500.0f/4096;
        g_ADConvert.gain.f32Ic   = 1500.0f/4096;
    }
	else if( HALL == HALL_600P )
	{
		g_ADConvert.gain.f32Ia	 = 2400.0f/4096;
		g_ADConvert.gain.f32Ib	 = 2400.0f/4096;
		g_ADConvert.gain.f32Ic	 = 2400.0f/4096;
	}
    else
    {
        g_ADConvert.gain.f32Ia   = 1080.0f/4096;  //电流+/-540A，对应AD端口3V/0V
        g_ADConvert.gain.f32Ib   = 1080.0f/4096;
        g_ADConvert.gain.f32Ic   = 1080.0f/4096;

    }
    gADOffsetInfo.EnableCount = 0;
    gADOffsetInfo.Flag = 0;
    g_ADConvertSum.offset.f32Ia = 0;
    g_ADConvertSum.offset.f32Ib = 0;
    g_ADConvertSum.offset.f32Ic = 0;
    g_ADConvert.offset.f32Ia   = 0;
    g_ADConvert.offset.f32Ib   = 0;
    g_ADConvert.offset.f32Ic   = 0;

    //电压K值和B值初始化
    g_ADConvert.gain.f32Vdc  = 203.3/4096;      //电压112.1/0V，对应AD端口3V/0V         HP1   new
    g_ADConvert.offset.f32Vdc  = 0;

    //温度采样K值和B值初始化
    g_ADConvert.gain.f32Temp = 1;     //温度120℃，对应AD端口2.95V ;  温度-40℃，对应AD端口0.057V
    g_ADConvert.gain.f32AI4  = 1;
    g_ADConvert.gain.f32AI5  = 1;
    g_ADConvert.gain.f32Temp_AMB = 1;

    g_ADConvert.offset.f32Temp = 0;
    g_ADConvert.offset.f32AI4  = 0;
    g_ADConvert.offset.f32AI5  = 0;
    g_ADConvert.offset.f32Temp_AMB = 0;

    //温度值初始化
    gTemperature.Temp           =   -40;
    gTemperature.MotorTemp      =   -50;
    gTemperature.MotorTemp2     =   -50;
    gTemperature.AMBTemp        =   -40;
    gTemperature.ErrMotorCnt    =   0;
    gTemperature.ErrCnt         =   0;

    //UdUq给定补偿相关变量初始化
    gIdDecouple.out     =   0;
    gIdDecouple.LdivRs  =   0;
    gIdDecouple.Ts      =   0.1e-3f;
    gIdDecouple.c       =   0;
    gIqDecouple.out     =   0;
    gIqDecouple.LdivRs  =   0;
    gIqDecouple.Ts      =   0.1e-3f;
    gIqDecouple.c       =   0;



    //传递电机实时运行数据的中间变量
//    gSendToFunctionDataBuff[0] = 0x0040;

    //电流环相关参数初始化
    gIMTSet.M = 0;
    gIMTSet.T = 0;
    gIMT.M = 0;
    gIMT.T = 0;
    gIMT.MFilter = 0;
    gIMT.TFilter = 0;
    gIMAcr.Err      =   0;
    gIMAcr.ErrLmt   =   50;
    gITAcr.Err      =   0;
    gITAcr.ErrLmt   =   50;
    gIMAcr.Ref = 0;
    gITAcr.Ref = 0;

    //弱磁参数初始化无作用
//    gPMFwCtrl.IqLimitLpf = 0;


    //电机极对数与旋变极对数初始化
    gMotorInfo.Poles = 4;        // king added, 20170306
    gRotorTrans.Poles   = 4;


    //旋变相关变量初始化
    gIPMPos.RotorZero       =   0;  // arc degree
    gRotorTrans.PosComp     =   0;
    gRotorTrans.Pos         =   0;
    gRotorTrans.PrevPos0    =   0;
    gRotorTrans.PrevPos1    =   0;
    gRotorTrans.LimitCnt    =   0;
    gRotorTrans.RTPosDelt   =   0;
    gRotorTrans.ConFlag     =   0;
    gRotorTrans.FreqFeed    =   0;
    gRotorTrans.PosCompCoef =   3.9106f;
    gRotorPosTrip.isTrip    =   0;
    gRotorPosTrip.prevPos1  =   0;
    gRotorPosTrip.prevPos0  =   0;
    gRotorPosTrip.currPos   =   0;
    gRotorPosTrip.currSpeed =   0;

    //转速的第一次滤波时间变量，上位机传递
    g_SpeedFilter    =   0;

    //线电流变量初始化
    gLineCur.Current = 0;
    gLineCur.CurrentFilter = 0;

    //电压滤波初始化
    gUDC.uDCBigFilter = 0;
    gUDC.uDCFilter = 0;

    //参数辨识变量初始化
    PMEstInit();
    gPMEstControl.RotorZeroEstOnly = 0;
    gPMEstControl.RotorZeroCompensateEn = 0;

    //弱磁变量初始化
    PMFwInit();
    gPMFwCtrl.FwPI.Kp       =   0.100f;
    gPMFwCtrl.FwPI.Ki       =   0.0010f;

    //扭矩补偿参数初始化
    InitTRefComp();

    //电机标定变量初始化
    gMTPA.Curr              =   0.01f;
    gMTPA.Theta             =   0;
    gMTPA.SinVal            =   1;
    gMTPA.CosVal            =   0;
    gMTPA.Iq                =   0.01f;
    gMTPA.i16CurrSetting    =   0;
    gMTPA.u16ThetaSetting   =   0;
    gMTPA.i16IqSetting      =   0;


    //转速环路KPKI初始化
    gAsr.Asr.Kp = 2.00f;
    gAsr.Asr.Ki = 0.03f;

    KeyOn_Enable = 0;

    //逐波限流相关
//    gCBCProtect.CntU    =   0;
//    gCBCProtect.CntV    =   0;
//    gCBCProtect.CntW    =   0;
//    gCBCProtect.TotalU  =   0;
//    gCBCProtect.TotalV  =   0;
//    gCBCProtect.TotalW  =   0;

    //输出扭矩初始化
    gPowerTrq.TrqOut        =   0;

    //转速限制初始化
    gDmdCanRe.FwdSpdLimit = 400;
    gDmdCanRe.RevSpdLimit = 400;

    gDmdCanRe.MaxTroqLimit       =  125;
    gDmdCanRe.MinTroqLimit       =  -125;
//    gVCPar.torqCtrlMaxFwdFreq = 500;
//    gVCPar.torqCtrlMaxRevFreq = 500;

    //扭矩初始化
    g_torquecmd = 0;

    //外发输出扭矩 gPowerTrq.TrqOut*10/gf32SuperAccK
//    gf32TrqOut = 0;
}

/*************************************************************
    启动电机运行前的数据初始化处理，为电机运行准备初始参数
*************************************************************/
void RunDataIntit(void)
{
	ResetAsrPar();
	ResetCsrPar();

	gIMT.M          =   0;
	gIMT.T          =   0;

	gIUVW.U         =   0;
	gIUVW.V         =   0;
	gIUVW.W         =   0;
	gCurSamp.U      =   0;
	gCurSamp.V      =   0;
	gCurSamp.W      =   0;

	gLineCur.Current        =   0;
	gLineCur.CurrentFilter  =   0;

	g_torquecmd = 0;

//	KP_Result = 0;
//	KI_Result = 0;
//	Pre_LoopOut = 0;
}
