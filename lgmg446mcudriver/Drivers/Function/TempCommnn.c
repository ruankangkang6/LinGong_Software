//=====================================================================
//
// 模块主要功能：接收和处理上位机相关变量
//
//=====================================================================
#include "f_public.h"
#include "MotorCalibration.h"

COMM_PROTOCOL_STRU gCommProVar = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

COMM_PROTOCOL_SEED_STRU gCommSeedVar = {0};
/*************************************************************
  上位机参数接收与转换
*************************************************************/
void ParGetFromFunction(void)
{
    float f32Temp = 0;

    if (1 == gCommProVar.CtrlCmdSrc)  //上位机控制命令指令
    {
        gDmdCanRe.EnableReq = gCommProVar.EnableReq;
        gDmdCanRe.Direction = gCommProVar.Direction;
        gDmdCanRe.ModeReq  = gCommProVar.ModeReq;
        gDmdCanRe.FreqCmd  = gCommProVar.FreqCmd;
        gDmdCanRe.TorqueCmd = gCommProVar.TorqueCmd*0.05f - 1000;
    }

    if (0 == g_StartFlag/*0 == gMainCmd.Command.bit.Start*/)//dspMainCmd.bit.run
    {
        //--------------------------------------电机参数-------------------------------------------
        gMotorInfo.Number                =   0;//gCommProVar.MotorNumber;             //电机编号
        gMotorInfo.Type                  =   0;//parameter[MOTORTYPE];//0;//gCommProVar.MotorType;               //电机类型
        gMotorInfo.Power                 =   35;//parameter[MOTORPOWER];//gCommProVar.MotorPower;              //电机额定功率
        gMotorInfo.PeakPower             =   0;//parameter[MOTORPEAKPOWER];//0;//gCommProVar.MotorPeakPower;          //电机峰值功率
        gMotorInfo.Rpm                   =   3000;//parameter[MOTORRPM];//gCommProVar.MotorRpm;                //电机额定转速
        gMotorInfo.MaxRpm                =   6000;//parameter[MOTORMAXRPM];//gCommProVar.MotorMaxRpm;             //电机峰值转速

        gMotorInfo.Poles                 =   4;//parameter[MOTORPOLES];//gCommProVar.MotorPoles;              //电机旋变极对数

        gRotorTrans.Poles                =   4;//parameter[ROTORTRANSPOLES];//gCommProVar.RTPoles;                 //旋变极对数
        gMotorInfo.Voltage               =   81;//parameter[MOTORVOLTAGE];//gCommProVar.MotorVotage;             //电机额定电压
        gMotorInfo.Current               =   500;//parameter[MOTORCURRENT];//gCommProVar.MotorCurrent;            //电机额定电流

//        gMotorInfo.MaxCurrent            =   (1 + 0.01f*gCommProVar.MotorMaxCurrent)*gMotorInfo.Current;//电机允许最大电流 = 电机最大电流%*电机额定电流
        gMotorInfo.MaxCurrent            =   810;//parameter[MOTORMAXCURRENT];//gCommProVar.MotorMaxCurrent;         //电机峰值电流

        gMotorInfo.MaxTorque             =   125;//parameter[MOTORMAXTORQUE];//gCommProVar.MotorMaxTorque;//RATED_TORQUE_VALUE                  //电机峰值扭矩

        f32Temp = parameter[MOTORZEROPOSITION]*1.0e-3f;
        if ( f32Temp >= Value_2Pi )
        {
            f32Temp -= Value_2Pi;
        }
        gIPMPos.RotorZero                =   0;//-2.18749;//-1.49258f;//-3.39475f;//-1.2686f;//-f32Temp;                             //角度计算使用的零位值，取负。

        gMotorInfo.PmRs                  =   0.005;//0.001*parameter[MOTORRS];//gCommProVar.MotorRs*1.0e-3f;         //鐢垫満瀹氬瓙鐢甸樆
        gMotorInfo.LD                    =   0.00005;//0.001*parameter[MOTORLD];//gCommProVar.MotorLd*1.0e-6f;         //鐢垫満d杞寸數鎰?
        gMotorInfo.LQ                    =   0.00011;//0.001*parameter[MOTORLQ];//gCommProVar.MotorLq*1.0e-6f;         //鐢垫満q杞寸數鎰?
        gMotorInfo.VsCoe                 =   0.004;//0.001*parameter[MOTORBEMF];//gCommProVar.MotorBemf*1.0e-3f;       //鐢垫満鍙嶇數鍔垮父鏁?

//        gPMFwCtrl.IdMax1                 =   0.0001f*gu16MaxFwIdSetting*gMotorInfo.Current;
        gPMFwCtrl.IdMax1                 =   600;//parameter[MOTORMAXID];//gCommProVar.IdMax;                   //最大弱磁电流

        g_MaxFeedbackCurr                =   600;//parameter[MOTORMAXFEDBACKCUR];//0.01f*gCommProVar.MaxFeedbackCurr;   //最大回馈电流 用于高速限速功能的扭矩限制





        //--------------------------------------控制器参数-------------------------------------------
        gInvInfo.InvPower                =   0;//parameter[INVPOWER];//gCommProVar.InvPower;                 //控制器额定功率
        gInvInfo.InvVotage               =   0;//parameter[INVVOLTAGE];//gCommProVar.InvVotage;                //控制器额定电压

//        gInvInfo.InvCurrent     =   260; //变频器额定电流-有效值，单位A
        gInvInfo.InvCurrent              =   0;//parameter[INVCURRENT];//gCommProVar.InvCurrent;                //控制器额定电流

        gInvInfo.InvMaxCurrent           =   0;//parameter[INVMAXCURRENT];//gCommProVar.InvMaxCurrent;             //控制器峰值电流






        //--------------------------------------速度控制-------------------------------------------
        g_AccFrqTime                     =   60;//parameter[ACCFRQTIME];//gCommProVar.AccFrqTime;              //转速加速时间
        g_DecFrqTime                     =   60;//parameter[DECFRQTIME];//gCommProVar.DecFrqTime;              //转速减速时间
        if (gCommProVar.SpdLimSrc == 1)
        {
            gDmdCanRe.FwdSpdLimit        =   6000;//parameter[FWDSPDLIMIT];//gCommProVar.FwdSpdLimit;                       //正转速度上限设定
            gDmdCanRe.RevSpdLimit        =   6000;//parameter[REVSPDLIMIT];//gCommProVar.RevSpdLimit;                       //正转速度上限设定
        }





        //--------------------------------------扭矩控制-------------------------------------------
        g_AccTorTime                     =   1;//parameter[ACCTORTIME];//gCommProVar.AccTorTime;              //扭矩控转矩加载时间
        g_AecTorTime                     =   1;//parameter[DECTORTIME];//gCommProVar.DecTorTime;              //扭矩控转矩降载时间
//        if (gCommProVar.TorLimSrc == 1)
//        {
//            gDmdCanRe.MaxTroqLimit       =  20;// parameter[MAXTORQLIMIT];//20;
//            gDmdCanRe.MinTroqLimit       =  -20;//-parameter[MINTORQLIMIT];//-20;
//        }
        gDmdCanRe.MaxTroqLimit       =   125;//parameter[MAXTORQLIMIT];//20;
        gDmdCanRe.MinTroqLimit       =   -125;//-(int32_t)parameter[MINTORQLIMIT];//-20;


        //--------------------------------------增益参数-------------------------------------------
        gAsr.KPLow                       =   0;//0.001f*parameter[ASRKPL];          //速度环高速KP1
        gAsr.KILow                       =   0;//0.001f*parameter[ASRKIL];          //速度环高速Ki1
        gAsr.KPHigh                      =   0;//0.001f*parameter[ASRKPH];          //速度环低速Kp2
        gAsr.KIHigh                      =   0;//0.001f*parameter[ASRKIH];          //速度环低速Ki2
        gAsr.SwitchLow                   =   0;//parameter[ASRCHGFRQL];             //速度环低速切换频率
        gAsr.SwitchHigh                  =   0;//parameter[ASRCHGFRQH];             //速度环高速切换频率
        gAsr.OutFilterCoef               =   0;//0.001*parameter[SPEEDFILTER];          //速度环输出滤波参数

//        if ( gCommProVar.SpeedFilter > 0 && gCommProVar.SpeedFilter < 10000 )    //反馈速度滤波设置
//        {
//            g_SpeedFilter = 0.0001f*gCommProVar.SpeedFilter;
//        }
//        else
//        {
            g_SpeedFilter = 0.100f;//0.0300f;
//        }

		gAcr.GainChgSel                 =    0;//parameter[GAINCHGSEL];//gCommProVar.AcrGainChgSel;          //电流环增益切换选择
		gAcr.ChgSpd                     =    0;//parameter[CHGSPD];//gCommProVar.AcrChgSpd;              //电流环增益切换速度点
		gAcr.ChgCurr                    =    0;//parameter[CHGCURR];//gCommProVar.AcrChgCurr;             //电流环增益切换电流点
		gAcr.ChgHysterVal               =    0;//parameter[CHGHYSTERVAL];//gCommProVar.AcrChgHysterVal;        //电流环增益切换滞环值
		gAcr.IdKpH                      =    0;//0.001f*parameter[IDKPH];//gCommProVar.AcrIdKpH;        //D轴电流环KP1
		gAcr.IdKiH                      =    0;//0.001f*parameter[IDKIH];//gCommProVar.AcrIdKiH;        //D轴电流环Ki1
		gAcr.IdKpL                      =    0;//0.001f*parameter[IDKPL];//gCommProVar.AcrIdKpL;        //D轴电流环KP2
		gAcr.IdKiL                      =    0;//0.001f*parameter[IDKIL];//gCommProVar.AcrIdKiL;        //D轴电流环Ki2
		gAcr.IqKpH                      =    0;//0.001f*parameter[IQKPH];//gCommProVar.AcrIqKpH;        //Q轴电流环KP1
		gAcr.IqKiH                      =    0;//0.001f*parameter[IQKIH];//gCommProVar.AcrIqKiH;        //Q轴电流环Ki1
		gAcr.IqKpL                      =    0;//0.001f*parameter[IQKPL];//gCommProVar.AcrIqKpL;        //Q轴电流环KP2
		gAcr.IqKiL                      =    0;//0.001f*parameter[IQKIL];//gCommProVar.AcrIqKiL;        //Q轴电流环Ki2




        //--------------------------------------控制优化参数-------------------------------------------
        gBasePar.FcSet                   =   0;//parameter[CARRIERFRQ];//gCommProVar.CarrierFrq;              //载波频率
        gPMEstControl.RotorZeroEstOnly   =   1;//parameter[ROTORZEROESTONLY];//1;//gCommProVar.RotorZeroEstOnly;        //仅辨识零位功能


        //--------------------------------------叉车速度环行驶控制参数-------------------------------------------
        g_SpdCtrlEnable                   =   0;//parameter[SPDCTRLENABLE];//gCommProVar.SpdCtrlEnable;          //叉车速度控行驶工况使能开启

        //--------------------------------------通讯参数-------------------------------------------
//        g_CAN1BaudRate                   =   0;//gCommProVar.CAN1BaudRate;            //CAN1波特率
//        g_CAN2BaudRate                   =   0;//gCommProVar.CAN2BaudRate;            //CAN2波特率

        //--------------------------------------保护参数-------------------------------------------
        gLimitParams.IACOffsetLimit      =   80;//parameter[IACOFFSETLIMIT];//80;//gCommProVar.CurrCheckErrVal;          //闆舵紓鏁呴殰妫€娴嬮槇鍊?
        gLimitParams.motorTempLimit      =   155;//parameter[MOTORTEMPLIMIT];//100;//0.1f*gCommProVar.MotorTempLimit;      //鐢垫満杩囨俯鐐?
        gLimitParams.motorTempWarn       =   150;//parameter[MOTORTEMPLIMIT] - 10;//100;//0.1f*gCommProVar.MotorTempWarn;       //鐢垫満棰勮鐐?
        gLimitParams.igbtTempLimit       =   100;//parameter[IGBTTEMPLIMIT];//100;//0.1f*gCommProVar.IGBTTempLimit;       //IGBT杩囨俯鐐?
        gLimitParams.igbtTempWarn        =   90;//parameter[IGBTTEMPLIMIT] - 10;//100;//0.1f*gCommProVar.IGBTTempWarn;        //IGBT棰勮鐐?

        gLimitParams.speedPLimit         =   450;//parameter[SPEEDPLIMIT];//180;//0.1f*gCommProVar.SpeedPLimit;         //瓒呴€熶繚鎶ょ偣姝ｈ浆
        gLimitParams.speedNLimit         =   450;//parameter[SPEEDNLIMIT];//180;//0.1f*gCommProVar.SpeedNLimit;         //瓒呴€熶繚鎶ょ偣鍙嶈浆

        gLimitParams.OcpInstLimit        =   750;//parameter[OCPINSTLIMIT];//350;//0.1f*gCommProVar.OcpInstLimit;        //鐬椂杩囨祦鐐?
        gLimitParams.OcpFilterLimit      =   750;//parameter[OCPFILTERLIMIT];//350;//0.1f*gCommProVar.OcpFilterLimit;      //婊ゆ尝杩囨祦鐐?

        gLimitParams.LowVoltUnderVol     =   0;//gCommProVar.LowVoltUnderVol;          //浣庡帇娆犲帇淇濇姢闃堝€?
        gLimitParams.LowVoltOverVol      =   0;//gCommProVar.LowVoltOverVol;           //浣庡帇杩囧帇淇濇姢闃堝€?


        gLimitParams.VoltOnDiff          =   5;//gCommProVar.VoltOnDiff;               //寮€鏈虹數鍘嬫粸鐜?
        gLimitParams.inputVoltOff        =   55;//parameter[INPUTVOLTOFF];//60;//gCommProVar.InputVoltOff;             //楂樺帇娆犲帇淇濇姢闃堝€?
        gLimitParams.inputVoltOvr        =   100;//parameter[INPUTVOLTOVR];//100;//gCommProVar.InputVoltOvr;             //楂樺帇杩囧帇淇濇姢闃堝€?

        //--------------------------------------泄放参数-------------------------------------------
        gActvDischgCtrl.SafetyVolt = 0;//parameter[SAFETYVOLT];//0.1f*gCommProVar.SafetyVolt;
        gActvDischgCtrl.TimeoutSec = 0;//parameter[TIMEOUTSEC];//0.01f*gCommProVar.TimeoutSec;
        Saturatefloat(&gActvDischgCtrl.SafetyVolt, 200, 30);
        Saturatefloat(&gActvDischgCtrl.TimeoutSec, 100, 0.1f);
    }
}


/*************************************************************
    参数计算程序：运行参数准备
*************************************************************/
void ParameterChange(void)
{
    if( 0 == g_StartFlag/*0 == gMainCmd.Command.bit.Start*/ )  //这个开始不包括泄放啊
    {

    	gMotorInfo.Frequency = (gMotorInfo.Rpm * gMotorInfo.Poles)/60;

        //计算电机极对数，通过电机额定频率和电机额定转速，n=60f/p
//        if ( 0 == gMotorInfo.Frequency )
//        {
//            gMotorInfo.Poles =   1;
//        }
//        else
//        {
//            gMotorInfo.Poles =   (uint16_t)( gMotorInfo.Frequency*60/gMotorInfo.Rpm + 0.5f );
//        }

        //计算用于电压给定补偿的参数
        if ( gMotorInfo.PmRs > 0
            && gMotorInfo.LD > 0
            && gMotorInfo.LQ > 0 )
        {
            gIdDecouple.LdivRs  =   gMotorInfo.LQ/gMotorInfo.PmRs;
            gIqDecouple.LdivRs  =   gMotorInfo.LD/gMotorInfo.PmRs;
        }
        else
        {
            gIdDecouple.LdivRs = 0;
            gIqDecouple.LdivRs = 0;
        }

        /*********************************************************/
        //计算死区时间
        gDeadBand.DeadBand = gCommProVar.DeadComp;//DEADDANDTIME;
        gDeadBand.DeadComp = 0;//gCommProVar.DeadComp;

//        EALLOW;//TI为了提高DSP安全性能，一些关键寄存器禁止改写。需使用汇编EALLOW命令，解除禁止然后进行改写，然后EDIS关闭解除
//        EPwm1Regs.DBFED.bit.DBFED = gDeadBand.DeadBand;
//        EPwm1Regs.DBRED.bit.DBRED = gDeadBand.DeadBand;
//        EPwm2Regs.DBFED.bit.DBFED = gDeadBand.DeadBand;
//        EPwm2Regs.DBRED.bit.DBRED = gDeadBand.DeadBand;
//        EPwm3Regs.DBFED.bit.DBFED = gDeadBand.DeadBand;
//        EPwm3Regs.DBRED.bit.DBRED = gDeadBand.DeadBand;
//        EDIS;
    }

    //弱磁功能Id的最大限制，Id给定时限制
//    gPMFwCtrl.IdMax1       =   0.0001f*gu16MaxFwIdSetting*gMotorInfo.Current;

    /*********************************************************/
//    SelectAcrPar();         //电流环PI参数切换
    CalCarrierWaveFreq();   //计算载波频率
}

/*************************************************************
  监控参数外发
*************************************************************/
void ParSeedFunction(void)
{
	//外发值精度和偏移量需确定
    //--------------------------------------基本参数监控组-------------------------------------------
	gCommSeedVar.MotorSpeed  = 0;//gRotorSpeed.SpeedApply;
	gCommSeedVar.TorqueCmd   = 0;//gDmdCanRe.TorqueCmd;
	gCommSeedVar.TorqueOut   = 0;//gPowerTrq.TorqueRef;
	gCommSeedVar.Udc        = 0;//gUDC.uDCFilter;
	gCommSeedVar.LowVolt    = 0;
	gCommSeedVar.Idc        = 0;
	gCommSeedVar.IGBTTemp   = 0;//gTemperature.Temp;
	gCommSeedVar.MotorTemp1 = 0;//gTemperature.MotorTemp;
	gCommSeedVar.MotorTemp2 = 0;//gTemperature.MotorTemp2;

	gCommSeedVar.KsiVolt        = 0;//Ctrl_Vin;
	gCommSeedVar.RunCase   = 0;//g_RunStatus;
	gCommSeedVar.Fault = 0;
	gCommSeedVar.MotorNum = 0;//gMotorInfo.Number;
	gCommSeedVar.SWVersion = 0;//MOTOR_SEL;



    //--------------------------------------内部监控参数-------------------------------------------
	gCommSeedVar.IUFb  = 0;//gIUVW.U;
	gCommSeedVar.IVFb   = 0;//gIUVW.V;
	gCommSeedVar.IWFb   = 0;//gIUVW.W;
	gCommSeedVar.IdGd   = 0;//gIMT.M;
	gCommSeedVar.IdFb   = 0;//gIMT.T;
	gCommSeedVar.IqGd   = 0;//gIMTSet.M;
	gCommSeedVar.IqFb   = 0;//gIMTSet.T;
	gCommSeedVar.UdPIOut   = 0;//gIMAcr.PIOut;
	gCommSeedVar.UqPIOut   = 0;//gITAcr.PIOut;
	gCommSeedVar.UdComp   = 0;//gPMFwOut.UdComp;
	gCommSeedVar.UqComp   = 0;//gPMFwOut.UqComp;
	gCommSeedVar.UdGd   = 0;//gUMTSet.M;
	gCommSeedVar.UqGd   = 0;//gUMTSet.T;
	gCommSeedVar.ModulateIndex   = 0;//ModulateIndex;
	gCommSeedVar.IMPhase   = 0;//gPhase.IMPhase;

    //--------------------------------------内部监控参数-------------------------------------------
	gCommSeedVar.IUSampleDVal  = 0;//g_SampleDValue.f32Ia;
	gCommSeedVar.IVSampleDVal   = 0;//g_SampleDValue.f32Ib;
	gCommSeedVar.IWSampleDVal   = 0;//g_SampleDValue.f32Ic;
	gCommSeedVar.UDCSampleDVal   = 0;//g_SampleDValue.f32Vdc;
	gCommSeedVar.LowVolSampleDVal   = 0;
	gCommSeedVar.IGBTSampleVal   = 0;//g_SampleDValue.f32Vdc;
	gCommSeedVar.KSISampleVal   = 0;//AD_Ctrl_Vin;
}
