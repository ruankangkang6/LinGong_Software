/*
 * PrmMgr.c
 *
 *  Created on: Feb 27, 2025
 *      Author:
 */

#include "main.h"

#define SOFTWARE_VERSION_EEP_ADDR               1
#define SOFTWARE_VERSION_EE                      4
typedef struct
{
    Uint32 BootloaderVersion;
    Uint32 SoftwareVersion;
    Uint32 SoftwareVersionCopy;
    Uint32 MotorCtrlVersion;
}StructVersion;
StructVersion   Version;



#define  TOTAL_FUNC_NUM  81    /* All Funcode Num */
#define  TOTAL_FUNC_GROUP  1   /* All Group Num */

extern const  StructMonitorInfo* MonitorTbl[];

const Uint16 MonitorCanAddrOffset[] =
{
 0,                                        /* Un00xx */
 18,                                       /* Un01xx */
 29,

};

void parameter_init(void)
{
	parameter[MOTORTYPE] = 234;
	parameter[MOTORPOWER] = 35;
	parameter[MOTORVOLTAGE] = 81;
	parameter[MOTORCURRENT] = 550;
	parameter[MOTORFREQUENCY] = 15;
	parameter[MOTORRPM] = 3000;
	parameter[MOTORMAXCURRENT]= 810;
	parameter[ROTORTRANSPOLES] = 4;
	parameter[MOTORLD] = 123;
	parameter[MOTORLQ] = 355;
	parameter[MOTORRS] = 235;
	parameter[MOTORBEMF] = 600;
	parameter[MOTORZEROPOSCOMVAL] = 5;
	parameter[MOTORZEROPOSITION] = 179500;
	parameter[ASRKPH] = 4400;
	parameter[ASRKIH] = 600;
	parameter[ASRKPL] = 1500;
	parameter[ASRKIH] = 600;
	parameter[ASRKIL] = 200;
	parameter[ASRCHGFRQL] = 5;
	parameter[ASRCHGFRQH] = 10;
	parameter[SPEEDFILTER] = 100;
	parameter[ACRKP] = 1000;
	parameter[ACRKI] = 40;
	parameter[MAXFEEDBACKCURR] = 600;
	parameter[ACCTORTIME] = 1;
	parameter[DECTORTIME] = 1;
	parameter[MAXFRQ] = 250;
	parameter[UPPERFRQ] = 250;
	parameter[LOWERFRQ] = 250;
	parameter[ACCFRQTIME] = 3;
	parameter[DECFRQTIME] = 3;
	parameter[CARRIERFRQ] = 10;
	parameter[INPUTVOLTON] = 60;
	parameter[INPUTVOLTOFF] = 60;
	parameter[INPUTVOLTOVR] = 100;
	parameter[SPEEDPLIMIT] = 450;
	parameter[SPEEDNLIMIT] = 450;
	parameter[OCPINSTLIMIT] = 750;
	parameter[OCPFILTERLIMIT] = 750;
	parameter[AMBTEMPDERATEBEG] = 100;
	parameter[AMBTEMPDERATEEND] = 110;
	parameter[IGBTTEMPLIMIT] = 95;
	parameter[MOTORTEMPLIMIT] = 145;
	parameter[IGBTTEMPDERATEBEG] = 85;
	parameter[IGBTTEMPDERATEEND] = 95;
	parameter[MOTORTEMPDERATEBEG] = 135;
	parameter[MOTORTEMPDERATEEND] = 145;
	parameter[SAFETYVOLT] = 60;
	parameter[TIMEOUTSEC] = 3;
	parameter[FUNENFLAG] = 1;
	parameter[MOTORNUM] = 2;
	parameter[MOTORPEAKPOWER] = 45;
	parameter[MOTORMAXRPM] = 6000;
	parameter[MOTORPOLES] = 4;
	parameter[MOTORMAXTORQUE] = 125;
	parameter[MOTORMAXID] = 600;
	parameter[MOTORMAXFEDBACKCUR] = 600;
	parameter[INVPOWER] = 40;
	parameter[INVVOLTAGE] = 80;
	parameter[INVCURRENT] = 550;
	parameter[INVMAXCURRENT] = 800;
	parameter[FWDSPDLIMIT] = 6000;
	parameter[REVSPDLIMIT] = 6000;
	parameter[MAXTORQLIMIT] = 125;
	parameter[MINTORQLIMIT] = 125;
	parameter[ROTORZEROESTONLY] = 0;
	parameter[SPDCTRLENABLE] = 0;
	parameter[IACOFFSETLIMIT] = 80;
	parameter[GAINCHGSEL] = 1;
	parameter[CHGSPD] = 1500;
	parameter[CHGCURR] = 50;
	parameter[CHGHYSTERVAL] = 10;
	parameter[IDKPH] = 2200;
	parameter[IDKIH] = 50;
	parameter[IDKPL] = 2200;
	parameter[IDKIL] = 50;
	parameter[IQKPH] = 2200;
	parameter[IQKIH] = 50;
	parameter[IQKPL] = 2200;
	parameter[IQKIL] = 50;
	parameter[LVUNDERVOL] = 9;
	parameter[LVOVRVOL] = 18;

}

void parameter_update(void)
{
    gMotorInfo.Type                  =   parameter[MOTORTYPE];              //电机类型
    gMotorInfo.Power                 =  parameter[MOTORPOWER];              //电机额定功率
    gMotorInfo.Rpm                   =   parameter[MOTORRPM];               //电机额定转速
    gRotorTrans.Poles                 =   parameter[ROTORTRANSPOLES];              //电机旋变极对数
    gMotorInfo.Voltage               =   parameter[MOTORVOLTAGE];             //电机额定电压
    gMotorInfo.Current               =   parameter[MOTORCURRENT];            //电机额定电流
    gMotorInfo.MaxCurrent            =   parameter[MOTORMAXCURRENT];         //电机峰值电流
    gIPMPos.RotorZero                =   0.001*parameter[MOTORZEROPOSITION];
    gMotorInfo.PmRs                  =   0.001*parameter[MOTORRS];
    gMotorInfo.LD                    =   0.001*parameter[MOTORLD];
    gMotorInfo.LQ                    =   0.001*parameter[MOTORLQ];
    gMotorInfo.VsCoe				=	0.001*parameter[MOTORBEMF];
    g_AccFrqTime                     =   parameter[ACCFRQTIME];
    g_DecFrqTime                     =   parameter[DECFRQTIME];
    gDmdCanRe.FwdSpdLimit			=	parameter[FWDSPDLIMIT];
    gDmdCanRe.RevSpdLimit       	 =   parameter[REVSPDLIMIT];
    g_AccTorTime                     =   parameter[ACCTORTIME];
    g_AecTorTime                     =   parameter[DECTORTIME];

    //--------------------------------------增益参数-------------------------------------------
    gAsr.KPLow                       =   0.001f*parameter[ASRKPL];          //速度环高速KP1
    gAsr.KILow                       =   0.001f*parameter[ASRKIL];          //速度环高速Ki1
    gAsr.KPHigh                      =   0.001f*parameter[ASRKPH];          //速度环低速Kp2
    gAsr.KIHigh                      =   0.001f*parameter[ASRKIH];          //速度环低速Ki2
    gAsr.SwitchLow                   =   parameter[ASRCHGFRQL];             //速度环低速切换频率
    gAsr.SwitchHigh                  =   parameter[ASRCHGFRQH];             //速度环高速切换频率
    gAsr.OutFilterCoef               =   0.001*parameter[SPEEDFILTER];          //速度环输出滤波参数

	gAcr.IdKpH                      =    0.001f*parameter[ACRKP];        //D轴电流环KP1
	gAcr.IdKiH                      =    0.001f*parameter[ACRKI];        //D轴电流环Ki1
	gAcr.IdKpL                      =    0.001f*parameter[ACRKP];        //D轴电流环KP2
	gAcr.IdKiL                      =    0.001f*parameter[ACRKI];        //D轴电流环Ki2
	gAcr.IqKpH                      =    0.001f*parameter[ACRKP];        //Q轴电流环KP1
	gAcr.IqKiH                      =    0.001f*parameter[ACRKI];        //Q轴电流环Ki1
	gAcr.IqKpL                      =    0.001f*parameter[ACRKP];        //Q轴电流环KP2
	gAcr.IqKiL                      =    0.001f*parameter[ACRKI];        //Q轴电流环Ki2

	gBasePar.FcSet                   =   parameter[CARRIERFRQ];              //载波频率

	gLimitParams.motorTempLimit      =   parameter[MOTORTEMPLIMIT];
	gLimitParams.igbtTempLimit       =   parameter[IGBTTEMPLIMIT];
	gLimitParams.speedPLimit         =   parameter[SPEEDPLIMIT];
	gLimitParams.speedNLimit         =   parameter[SPEEDNLIMIT];
	gLimitParams.OcpInstLimit        =   parameter[OCPINSTLIMIT];
	gLimitParams.OcpFilterLimit      =   parameter[OCPFILTERLIMIT];
	gLimitParams.inputVoltOff        =   parameter[INPUTVOLTOFF];
	gLimitParams.inputVoltOvr        =   parameter[INPUTVOLTOVR];
	gActvDischgCtrl.SafetyVolt		 = 	parameter[SAFETYVOLT];
    gActvDischgCtrl.TimeoutSec		 = 	parameter[TIMEOUTSEC];

    gMotorInfo.Number 				 = 	parameter[MOTORNUM];			//电机编号
    gMotorInfo.PeakPower             =  parameter[MOTORPEAKPOWER];              //电机额定功率
    gMotorInfo.MaxRpm                =  parameter[MOTORMAXRPM];             //电机峰值转速
    gMotorInfo.Poles                 =  parameter[MOTORPOLES];              //电机极对数
    gMotorInfo.MaxTorque             =   parameter[MOTORMAXTORQUE];                 //电机峰值扭矩
    gPMFwCtrl.IdMax1                 =   parameter[MOTORMAXID];
    g_MaxFeedbackCurr                =   parameter[MOTORMAXFEDBACKCUR];   //最大回馈电流 用于高速限速功能的扭矩限制

    //--------------------------------------控制器参数-------------------------------------------
    gInvInfo.InvPower                =   parameter[INVPOWER];                 //控制器额定功率
    gInvInfo.InvVotage               =   parameter[INVVOLTAGE];                //控制器额定电压
    gInvInfo.InvCurrent              =   parameter[INVCURRENT];                //控制器额定电流
    gInvInfo.InvMaxCurrent           =   parameter[INVMAXCURRENT];             //控制器峰值电流

	gDmdCanRe.MaxTroqLimit       =   parameter[MAXTORQLIMIT];           //电动扭矩上限设定（最大正扭矩）
	gDmdCanRe.MinTroqLimit       =   -parameter[MINTORQLIMIT];//回馈扭矩上限设定（最大负扭矩）

    gPMEstControl.RotorZeroEstOnly   =   parameter[ROTORZEROESTONLY];        //仅辨识零位功能

    g_SpdCtrlEnable                  =   parameter[SPDCTRLENABLE];          //叉车速度控行驶工况使能开启
    gLimitParams.IACOffsetLimit      =   parameter[IACOFFSETLIMIT];
	gAcr.GainChgSel                 =    parameter[GAINCHGSEL];          //电流环增益切换选择
	gAcr.ChgSpd                     =    parameter[CHGSPD];              //电流环增益切换速度点
	gAcr.ChgCurr                    =    parameter[CHGCURR];             //电流环增益切换电流点
	gAcr.ChgHysterVal               =    parameter[CHGHYSTERVAL];        //电流环增益切换滞环值
	gAcr.IdKpH                      =    0.001f*parameter[IDKPH];        //D轴电流环KP1
	gAcr.IdKiH                      =    0.001f*parameter[IDKIH];        //D轴电流环Ki1
	gAcr.IdKpL                      =    0.001f*parameter[IDKPL];        //D轴电流环KP2
	gAcr.IdKiL                      =    0.001f*parameter[IDKIL];        //D轴电流环Ki2
	gAcr.IqKpH                      =    0.001f*parameter[IQKPH];        //Q轴电流环KP1
	gAcr.IqKiH                      =    0.001f*parameter[IQKIH];        //Q轴电流环Ki1
	gAcr.IqKpL                      =    0.001f*parameter[IQKPL];        //Q轴电流环KP2
	gAcr.IqKiL                      =    0.001f*parameter[IQKIL];        //Q轴电流环Ki2
	gLimitParams.LowVoltUnderVol     =   parameter[LVUNDERVOL];
	gLimitParams.LowVoltOverVol      =   parameter[LVOVRVOL];
}

Uint16  GetPrmEepromAddress(Uint16 TblIndex)
{
    Uint32 u32Data;
    u32Data = PnPrmTbl[TblIndex]->EepromAddr;

    return u32Data;
}

Uint32* GetPrmRamAddress(Uint16 TblIndex)
{
    int* pu32Data=0;
    switch(PnPrmTbl[TblIndex]->Attr.Rev1)
    {
    case 0: pu32Data = PnPrmTbl[TblIndex]->RamPtr;break;
    case 1: pu32Data = PnPrmTbl[TblIndex]->RamPtr2;break;
    case 2: pu32Data = (int*)PnPrmTbl[TblIndex]->RamPtr3;break;
    }
    //  pu32Data = PnPrmTbl[TblIndex]->RamPtr;

    return (Uint32*)pu32Data;
}

int32 GetMonitorValue(Uint16 Addr, int32 *Value)
{
    Uint16 InputBuffer[2];
    Uint16 AbsAddr;

    InputBuffer[0] = ((Addr & 0x0fff)>>8);      /* 监控参数组号 */
    InputBuffer[1] = Addr&0xff;                  /* 监控参数序号 */
    AbsAddr = MonitorCanAddrOffset[InputBuffer[0]] + InputBuffer[1]; /* 监控参数绝对地址 */
    if(AbsAddr == 0x3000)
    {
    	AbsAddr = 0x3000;
    }
    switch(AbsAddr)
	{
    case 0: *Value = (int32)gRotorSpeed.SpeedApply; break;
    case 1: *Value = u16AdcResult1_iv; break;
    case 2: *Value = u16AdcResult2_tempMotor; break;
    default: *Value = 0; break;
	}
//    if(InputBuffer[0] > (TOTAL_MONITOR_GROUP - 1))
//    {
//        return 1;
//    }
//    else
//    {
//        if(InputBuffer[0] == (TOTAL_MONITOR_GROUP - 1))
//        {
//            if(AbsAddr >= TOTAL_MONITOR_NUM)
//            {
//                return 1;
//            }
//        }
//        else if(AbsAddr >= MonitorCanAddrOffset[InputBuffer[0]+1])
//        {
//            return 1;
//        }
//        else
//        {
//
//        }
//    }
//    if(1 != (MonitorTbl[AbsAddr]->Attr).AccessLevel)
//    {
//        //return 2;      /* 参数不可读 */
//    }
//
//
//        switch(MonitorTbl[AbsAddr]->Attr.Rev1)
//        {
//        case 0:
//            *Value = *(MonitorTbl[AbsAddr]->RamPtr);
//            if(MonitorTbl[AbsAddr]->Attr.fixpoint==0||MonitorTbl[AbsAddr]->Attr.fixpoint==1)
//            {
//                switch(MonitorTbl[AbsAddr]->Attr.DecimalNum)
//                {
//                case 0:*Value =*Value ;
//                switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                {
//                case 0:*Value =*Value ;break;
//                case 1:*Value =*Value>>15;break;
//                case 2:*Value =*Value>>20 ;break;
//                case 3:*Value =*Value>>24 ;break;
//                }
//                break;
//                case 1:*Value =*Value*10 ;
//                switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                {
//                case 0:*Value =*Value ;break;
//                case 1:*Value =*Value>>15;break;
//                case 2:*Value =*Value>>20 ;break;
//                case 3:*Value =*Value>>24 ;break;
//                }
//                break;
//                case 2:*Value =*Value*(100) ;
//                switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                {
//                case 0:*Value =*Value ;break;
//                case 1:*Value =*Value>>15;break;
//                case 2:*Value =*Value>>20 ;break;
//                case 3:*Value =*Value>>24 ;break;
//                }break;
//                case 3:*Value =*Value*1000 ;
//                switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                {
//                case 0:*Value =*Value ;break;
//                case 1:*Value =*Value>>15;break;
//                case 2:*Value =*Value>>20 ;break;
//                case 3:*Value =*Value>>24 ;break;
//                }break;
//                }}
//            break;
//        case 1:
//            *Value = *(MonitorTbl[AbsAddr]->RamPtr2);
//            if(MonitorTbl[AbsAddr]->Attr.fixpoint==0||MonitorTbl[AbsAddr]->Attr.fixpoint==1)
//            {
//                switch(MonitorTbl[AbsAddr]->Attr.DecimalNum)
//                {
//                case 0:*Value =*Value ;
//                switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                {
//                case 0:*Value =*Value ;break;
//                case 1:*Value =*Value>>15;break;
//                case 2:*Value =*Value>>20 ;break;
//                case 3:*Value =*Value>>24 ;break;
//                }
//                break;
//                case 1:*Value =*Value*10 ;
//                switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                {
//                case 0:*Value =*Value ;break;
//                case 1:*Value =*Value>>15;break;
//                case 2:*Value =*Value>>20 ;break;
//                case 3:*Value =*Value>>24 ;break;
//                }break;
//                case 2:*Value =*Value*(100) ;
//                switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                {
//                case 0:*Value =*Value ;break;
//                case 1:*Value =*Value>>15;break;
//                case 2:*Value =*Value>>20 ;break;
//                case 3:*Value =*Value>>24 ;break;
//                }break;
//                case 3:*Value =*Value*1000 ;
//                switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                {
//                case 0:*Value =*Value ;break;
//                case 1:*Value =*Value>>15;break;
//                case 2:*Value =*Value>>20 ;break;
//                case 3:*Value =*Value>>24 ;break;
//                }break;
//                }}
//            break;
//        case 2:
//            *Value = *(MonitorTbl[AbsAddr]->RamPtr3)*100;
//            if(PnPrmTbl[AbsAddr]->Attr.fixpoint==0||PnPrmTbl[AbsAddr]->Attr.fixpoint==1)
//            {
//                switch(MonitorTbl[AbsAddr]->Attr.DecimalNum)
//                {
//                case 0:*Value =*Value ;
//                switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                {
//                case 0:*Value =*Value ;break;
//                case 1:*Value =*Value>>15;break;
//                case 2:*Value =*Value>>20 ;break;
//                case 3:*Value =*Value>>24 ;break;
//                }
//                break;
//                case 1:*Value =*Value*10 ;
//                switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                {
//                case 0:*Value =*Value ;break;
//                case 1:*Value =*Value>>15;break;
//                case 2:*Value =*Value>>20 ;break;
//                case 3:*Value =*Value>>24 ;break;
//                }break;
//                case 2:*Value =*Value*(100) ;
//                switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                {
//                case 0:*Value =*Value ;break;
//                case 1:*Value =*Value>>15;break;
//                case 2:*Value =*Value>>20 ;break;
//                case 3:*Value =*Value>>24 ;break;
//                }break;
//                case 3:*Value =*Value*1000 ;
//                switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                {
//                case 0:*Value =*Value ;break;
//                case 1:*Value =*Value>>15;break;
//                case 2:*Value =*Value>>20 ;break;
//                case 3:*Value =*Value>>24 ;break;
//                }break;
//                }}
//            break;
//        case 3:
//            *Value = *(MonitorTbl[AbsAddr]->RamPtr4);
//                  if(PnPrmTbl[AbsAddr]->Attr.fixpoint==0||PnPrmTbl[AbsAddr]->Attr.fixpoint==1)
//                  {
//                      switch(MonitorTbl[AbsAddr]->Attr.DecimalNum)
//                      {
//                      case 0:*Value =*Value ;
//                      switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                      {
//                      case 0:*Value =*Value ;break;
//                      case 1:*Value =*Value>>15;break;
//                      case 2:*Value =*Value>>20 ;break;
//                      case 3:*Value =*Value>>24 ;break;
//                      }
//                      break;
//                      case 1:*Value =*Value*10 ;
//                      switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                      {
//                      case 0:*Value =*Value ;break;
//                      case 1:*Value =*Value>>15;break;
//                      case 2:*Value =*Value>>20 ;break;
//                      case 3:*Value =*Value>>24 ;break;
//                      }break;
//                      case 2:*Value =*Value*(100) ;
//                      switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                      {
//                      case 0:*Value =*Value ;break;
//                      case 1:*Value =*Value>>15;break;
//                      case 2:*Value =*Value>>20 ;break;
//                      case 3:*Value =*Value>>24 ;break;
//                      }break;
//                      case 3:*Value =*Value*1000 ;
//                      switch(MonitorTbl[AbsAddr]->Attr.fixpoint)
//                      {
//                      case 0:*Value =*Value ;break;
//                      case 1:*Value =*Value>>15;break;
//                      case 2:*Value =*Value>>20 ;break;
//                      case 3:*Value =*Value>>24 ;break;
//                      }break;
//                      }}
//                  break;
//        }
    return 0;
}

Uint16 GetPrmValue(Uint16 Addr, int32* Value)
{
    Uint16 InputBuffer[2];
    Uint16 AbsAddr;

    InputBuffer[0] = Addr >> 8;        /* 功能码组号 */
    InputBuffer[1] = Addr & 0x00ff;    /* 功能码序号 */
    AbsAddr = CanAddrOffset[InputBuffer[0]] + InputBuffer[1]; /* 功能码绝对地址 */
    if(AbsAddr == 65)
    {
        AbsAddr = 65;
    }
    if(InputBuffer[0]==0)
    {
        AbsAddr=AbsAddr-16;
    }
    if(AbsAddr == 28)
    {
        AbsAddr = AbsAddr;
    }
    /* 地址有效性检查 */
    if(InputBuffer[0] > (TOTAL_FUNC_GROUP - 1))
    {
        return 1;
    }
    else
    {
        if(InputBuffer[0] == (TOTAL_FUNC_GROUP - 1))
        {
            if(AbsAddr >= TOTAL_FUNC_NUM+16)
            {
                return 1;
            }
        }
        else if(AbsAddr >= CanAddrOffset[InputBuffer[0]+1]+16)
        {
            return 1;
        }
        else
        {

        }
    }

    *Value = parameter[AbsAddr];
//    switch(PnPrmTbl[AbsAddr]->Attr.Rev1)
//    {
//    case 0:
//
//        *Value = *(PnPrmTbl[AbsAddr]->RamPtr);
//        if(PnPrmTbl[AbsAddr]->Attr.fixpoint==0||PnPrmTbl[AbsAddr]->Attr.fixpoint==1)
//        {
//            switch(PnPrmTbl[AbsAddr]->Attr.DecimalNum)
//            {
//            case 0:*Value =*Value ;
//            switch(PnPrmTbl[AbsAddr]->Attr.fixpoint)
//            {
//            case 0:*Value =*Value ;break;
//            case 1:*Value =*Value>>15;break;
//            case 2:*Value =*Value>>20 ;break;
//            case 3:*Value =*Value>>24 ;break;
//            }
//            break;
//            case 1:*Value =*Value*10 ;
//            switch(PnPrmTbl[AbsAddr]->Attr.fixpoint)
//            {
//            case 0:*Value =*Value ;break;
//            case 1:*Value =*Value>>15;break;
//            case 2:*Value =*Value>>20 ;break;
//            case 3:*Value =*Value>>24 ;break;
//            }
//            break;
//            case 2:*Value =*Value*(100) ;
//            switch(PnPrmTbl[AbsAddr]->Attr.fixpoint)
//            {
//            case 0:*Value =*Value ;break;
//            case 1:*Value =*Value>>15;break;
//            case 2:*Value =*Value>>20 ;break;
//            case 3:*Value =*Value>>24 ;break;
//            }break;
//            case 3:*Value =*Value*1000 ;
//            switch(PnPrmTbl[AbsAddr]->Attr.fixpoint)
//            {
//            case 0:*Value =*Value ;break;
//            case 1:*Value =*Value>>15;break;
//            case 2:*Value =*Value>>20 ;break;
//            case 3:*Value =*Value>>24 ;break;
//            }break;
//            }}
//        break;
//    case 1:
//        *Value = *(PnPrmTbl[AbsAddr]->RamPtr2);
//        if(PnPrmTbl[AbsAddr]->Attr.fixpoint==0||PnPrmTbl[AbsAddr]->Attr.fixpoint==1)
//        {
//            switch(PnPrmTbl[AbsAddr]->Attr.DecimalNum)
//            {
//            case 0:*Value =*Value ;
//            switch(PnPrmTbl[AbsAddr]->Attr.fixpoint)
//            {
//            case 0:*Value =*Value ;break;
//            case 1:*Value =*Value>>15;break;
//            case 2:*Value =*Value>>20 ;break;
//            case 3:*Value =*Value>>24 ;break;
//            }
//            break;
//            case 1:*Value =*Value*10 ;
//            switch(PnPrmTbl[AbsAddr]->Attr.fixpoint)
//            {
//            case 0:*Value =*Value ;break;
//            case 1:*Value =*Value>>15;break;
//            case 2:*Value =*Value>>20 ;break;
//            case 3:*Value =*Value>>24 ;break;
//            }break;
//            case 2:*Value =*Value*(100) ;
//            switch(PnPrmTbl[AbsAddr]->Attr.fixpoint)
//            {
//            case 0:*Value =*Value ;break;
//            case 1:*Value =*Value>>15;break;
//            case 2:*Value =*Value>>20 ;break;
//            case 3:*Value =*Value>>24 ;break;
//            }break;
//            case 3:*Value =*Value*1000 ;
//            switch(PnPrmTbl[AbsAddr]->Attr.fixpoint)
//            {
//            case 0:*Value =*Value ;break;
//            case 1:*Value =*Value>>15;break;
//            case 2:*Value =*Value>>20 ;break;
//            case 3:*Value =*Value>>24 ;break;
//            }break;
//            }}
//        break;
//    case 2:
//        *Value = *(PnPrmTbl[AbsAddr]->RamPtr3);
//        if(PnPrmTbl[AbsAddr]->Attr.fixpoint==0||PnPrmTbl[AbsAddr]->Attr.fixpoint==1)
//        {
//            switch(PnPrmTbl[AbsAddr]->Attr.DecimalNum)
//            {
//            case 0:*Value =*Value ;
//            switch(PnPrmTbl[AbsAddr]->Attr.fixpoint)
//            {
//            case 0:*Value =*Value ;break;
//            case 1:*Value =*Value>>15;break;
//            case 2:*Value =*Value>>20 ;break;
//            case 3:*Value =*Value>>24 ;break;
//            }
//            break;
//            case 1:*Value =*Value*10 ;
//            switch(PnPrmTbl[AbsAddr]->Attr.fixpoint)
//            {
//            case 0:*Value =*Value ;break;
//            case 1:*Value =*Value>>15;break;
//            case 2:*Value =*Value>>20 ;break;
//            case 3:*Value =*Value>>24 ;break;
//            }break;
//            case 2:*Value =*Value*(100) ;
//            switch(PnPrmTbl[AbsAddr]->Attr.fixpoint)
//            {
//            case 0:*Value =*Value ;break;
//            case 1:*Value =*Value>>15;break;
//            case 2:*Value =*Value>>20 ;break;
//            case 3:*Value =*Value>>24 ;break;
//            }break;
//            case 3:*Value =*Value*1000 ;
//            switch(PnPrmTbl[AbsAddr]->Attr.fixpoint)
//            {
//            case 0:*Value =*Value ;break;
//            case 1:*Value =*Value>>15;break;
//            case 2:*Value =*Value>>20 ;break;
//            case 3:*Value =*Value>>24 ;break;
//            }break;
//            }}
//        break;
//    case 3:
//        break;
//    }
    return 0;       /*正常返回*/
}

Uint16  NvmReadInt32(Uint16 Address, Uint32* u32Data)
{
    return I2CReadInt32WaitMode(Address, u32Data);

}
void    SetPrmDefaultValue(Uint16 TblIndex)
{
    if(TblIndex < TOTAL_FUNC_NUM)
    {
        *PnPrmTbl[TblIndex]->RamPtr= PnPrmTbl[TblIndex]->DefaultValue;
        *PnPrmTbl[TblIndex]->RamPtr2= PnPrmTbl[TblIndex]->DefaultValue;
        *PnPrmTbl[TblIndex]->RamPtr3= PnPrmTbl[TblIndex]->DefaultValue;
    }
}
void NvmReadAllData(void)
{
    Uint32 ReadData;
    //Uint16 Address;
    //Uint32* pData;

    Uint16 i;
    I2CReadInt32WaitMode(SOFTWARE_VERSION_EEP_ADDR,&ReadData);
    Version.SoftwareVersion = ReadData;
    if( (Version.SoftwareVersion != SOFTWARE_VERSION_EE) )
    {
    	   parameter_init();

        for(i = 0; i <TOTAL_MONITOR_NUM ; i++)   //锟斤拷锟斤拷锟?
          {
//            SetPrmDefaultValue(i);
//            Address = GetPrmEepromAddress(i);
//            pData = GetPrmRamAddress(i);
            I2CWriteInt32WaitMode(0x10+i,parameter[i]);
          }
        I2CWriteInt32WaitMode(1,SOFTWARE_VERSION_EE);

    }
    else // 锟芥本没锟叫变化
    {
        for(i = 0; i <TOTAL_MONITOR_NUM ; i++)   //锟斤拷锟斤拷锟?
        {
//            Address = GetPrmEepromAddress(i);
//            pData = GetPrmRamAddress(i);
//            NvmReadInt32(Address,*pData);
            I2CReadInt32WaitMode(0x10+i,&parameter[i]);
        }

    }
    SDO_SetData_EEP_def();
    //SDO_GetData_EEP_Read();
}
Uint16 SetPrmValue(Uint16 Addr, int32 Value)
{
    Uint16 InputBuffer[2];
    Uint16 AbsAddr;

    InputBuffer[0] = Addr >> 8;        /* 功能码组号 */
    InputBuffer[1] = Addr & 0x00ff;    /* 功能码序号 */
   // AbsAddr = CanAddrOffset[InputBuffer[0]] + InputBuffer[1]; /* 功能码绝对地址 */
    AbsAddr = CanAddrOffset[InputBuffer[0]];
    AbsAddr +=InputBuffer[1]; /* 功能码绝对地址 */
    if(AbsAddr == 0x58)
    {
    	AbsAddr = 0x58;
    }
    if(InputBuffer[0]==0)
    {
        AbsAddr=AbsAddr-16;
    }
    /* 地址有效性检查 */
    if(InputBuffer[0] > (TOTAL_FUNC_GROUP - 1))
    {
        return 1;
    }
    else
    {
        if(InputBuffer[0] == (TOTAL_FUNC_GROUP - 1))
        {
            if(AbsAddr >= TOTAL_FUNC_NUM)
            {
                return 1;
            }
        }
        else if(AbsAddr >= CanAddrOffset[InputBuffer[0]+1]+16)
        {
            return 1;
        }
        else
        {

        }
    }

    parameter[AbsAddr] = Value;
//    switch(PnPrmTbl[AbsAddr]->Attr.Rev1)
//    {
//    case 0: *(PnPrmTbl[AbsAddr]->RamPtr) =Value ;break;
//    case 1: *(PnPrmTbl[AbsAddr]->RamPtr2) =Value;break;
//    case 2:
//        switch(PnPrmTbl[AbsAddr]->Attr.DecimalNum)
//        {
//        case 0:
//            *(PnPrmTbl[AbsAddr]->RamPtr3) =(double)Value;break;
//        case 1:
//            *(PnPrmTbl[AbsAddr]->RamPtr3) =(double)Value/10;break;
//        case 2:
//            *(PnPrmTbl[AbsAddr]->RamPtr3) =(double)Value/100;break;
//        }
//    }
    return 0;       /*正常返回*/
}
Uint16 GetTotalFuncNum(void)
{

    return TOTAL_FUNC_NUM;
}


void InitAllEepData(void)
{

//    SaveAllPrmDefaultToNvm();
}
void SaveAllPrmDefaultToNvm(void)
{
    Uint16 TotalFuncNum;
    Uint16 i;
    //TotalFuncNum=58;
    TotalFuncNum = GetTotalFuncNum();
    for(i = 0; i < TotalFuncNum; i++)
    {
        SetPrmDefaultValue(i);
//      SetPrmChangeStatus(i);
    }
    Uint16 Address;
    Uint32* pData;
      for(i = 0; i <17 ; i++)   //锟斤拷锟斤拷锟�
	{
    	  Address = GetPrmEepromAddress(i);
    	  pData = GetPrmRamAddress(i);
    	  NvmReadInt32(Address,pData);
	}
}
