/*
 * TempCommnn.h
 *
 *  Created on: 2024年12月25日
 *      Author: pc
 */

#ifndef SOURCE_FUNCTION_INCLUDE_TEMPCOMMNN_H_
#define SOURCE_FUNCTION_INCLUDE_TEMPCOMMNN_H_

//上位机变量-功能参数组
typedef struct COMM_PROTOCOL_STRU_DEF
{
	//基本控制组
    uint16_t CtrlCmdSrc;                        //控制指令来源
    uint16_t EnableReq;                         //开机命令
    uint16_t Direction;                         //电机旋转方向
    uint16_t ModeReq;                           //电机控制模式
    uint16_t FreqCmd;                           //转速命令
    uint16_t TorqueCmd;                         //扭矩命令
    uint16_t IdSet;                             //D轴电流指令
    uint16_t iqSet;                             //Q轴电流指令

    //电机参数组
    uint16_t MotorNumber;                       //电机编号    新增
    uint16_t MotorType;                         //电机类型
    uint16_t NegSeqCtrl;                        //负序控制    功能标定
    uint16_t MotorPower;                        //电机额定功率
    uint16_t MotorPeakPower;                    //电机峰值功率  新增
    uint16_t MotorRpm;                          //电机额定转速
    uint16_t MotorMaxRpm;                       //电机峰值转速  新增
    uint16_t MotorPoles;                        //电机极对数
    uint16_t RTPoles;                           //旋变极对数
    uint16_t MotorVotage;                       //电机额定电压
    uint16_t MotorCurrent;                      //电机额定电流
    uint16_t MotorMaxCurrent;                   //电机峰值电流  新增(原为上位机%)
    uint16_t MotorMaxTorque;                    //电机峰值扭矩  新增
    uint16_t MotorZeroPosition;                 //旋变零位
    uint16_t MotorRs;                           //电机定子电阻
    uint16_t MotorLd;                           //电机d轴电感
    uint16_t MotorLq;                           //电机q轴电感
    uint16_t MotorBemf;                         //电机反电势常数
    uint16_t IdMax;                             //最大弱磁电流  新增(原固定值)
    uint16_t MaxFeedbackCurr;                   //最大回馈电流  用于高速限速功能的扭矩限制
    uint16_t InitPosEstCurr;                    //初始位置辨识电流
    uint16_t RsEstCurr1;                        //定子电阻辨识电流1
    uint16_t RsEstCurr2;                        //定子电阻辨识电流2

    //控制器参数组
    uint16_t InvPower;                          //控制器额定功率    新增
    uint16_t InvVotage;                         //控制器额定电压    新增
    uint16_t InvCurrent;                        //控制器额定电流    新增(原固定值)
    uint16_t InvMaxCurrent;                     //控制器峰值电流    新增
    uint16_t DeadBand;                          //死区时间        新增
    uint16_t DeadComp;                          //死区补偿时间      新增
    uint16_t UdcCalCoef;                        //母线电压采样校准系数 新增
    uint16_t PhaseCurrCalCoef;                  //相电流采样校准系数  新增
    uint16_t CtrlVolCalCoef;                    //控制电压采样校准系数 新增


    //速度控制组
    uint16_t AccFrqTime;                        //转速控加速时间
    uint16_t DecFrqTime;                        //转速控减速时间
    uint16_t SpdLimSrc;                         //速度上限指令来源  新增
    uint16_t FwdSpdLimit;                       //正转速度上限设定  新增
    uint16_t RevSpdLimit;                       //反转速度上限设定  新增

    //扭矩控制组
    uint16_t AccTorTime;                        //转矩加速时间
    uint16_t DecTorTime;                        //转矩降速时间
    uint16_t TorLimSrc;                         //扭矩上限指令来源  新增
    uint16_t DriveTorLimit;                     //电动扭矩上限设定  新增
    uint16_t ChargeTorlimit;                    //回馈扭矩上限设定  新增

    //增益参数组
    uint16_t AsrKpH;                            //转速环高速KP1
    uint16_t AsrKiH;                            //转速环高速Ki1
    uint16_t AsrChgFrqL;                        //转速环低速切换频率
    uint16_t AsrKpL;                            //转速环低速KP2
    uint16_t AsrKiL;                            //转速环低速Ki2
    uint16_t AsrChgFrqH;                        //转速环高速切换频率
    uint16_t OutFilterCoef;                     //转度环输出滤波参数 新增
    uint16_t SpeedFilter;                       //反馈速度滤波设置

    uint16_t AcrGainChgSel;                     //电流环增益切换选择  新增
    uint16_t AcrChgSpd;                         //电流环增益切换速度点 新增
    uint16_t AcrChgCurr;                        //电流环增益切换电流点 新增
    uint16_t AcrChgHysterVal;                   //电流环增益切换滞环值 新增

    uint16_t AcrIdKpH;                          //D轴电流环KP1 新增
    uint16_t AcrIdKiH;                          //D轴电流环Ki1 新增
    uint16_t AcrIdKpL;                          //D轴电流环KP2 新增
    uint16_t AcrIdKiL;                          //D轴电流环Ki2 新增

    uint16_t AcrIqKpH;                          //Q轴电流环KP1 新增
    uint16_t AcrIqKiH;                          //Q轴电流环Ki1 新增
    uint16_t AcrIqKpL;                          //Q轴电流环KP2 新增
    uint16_t AcrIqKiL;                          //Q轴电流环Ki2 新增


    //控制优化参数组
    uint16_t SpdLimtAHS;                        //高速限速功能
    uint16_t TorqueComp;                        //扭矩补偿功能
    uint16_t RotorZeroEstOnly;                  //仅辨识零位功能
    uint16_t CarrierFrq;                        //载波频率

    //叉车速度环行驶控制参数组
    uint16_t SpdCtrlEnable;                     //速度控行驶工况使能开启  新增

    //通讯参数组
    uint16_t CAN1BaudRate;                      //CAN1波特率  新增
    uint16_t CAN2BaudRate;                      //CAN2波特率  新增


    //保护参数
    uint16_t CurrCheckErrVal;                   //零漂故障检测阈值   新增

    uint16_t MotorTempLimit;                    //电机过温保护点
    uint16_t MotorTempWarn;                     //电机过温告警点   新增
    uint16_t IGBTTempLimit;                     //IGBT过温保护点
    uint16_t IGBTTempWarn;                      //IGBT过温告警点   新增

    uint16_t SpeedPLimit;                       //正转超速保护点
    uint16_t SpeedNLimit;                       //反转超速保护点

    uint16_t OcpInstLimit;                      //瞬时过流点
    uint16_t OcpFilterLimit;                    //滤波过流点

    uint16_t LowVoltUnderVol;                   //低压欠压保护阈值   新增
    uint16_t LowVoltOverVol;                    //低压过压保护阈值   新增

    uint16_t VoltOnDiff;                        //开机电压滞环
    uint16_t InputVoltOff;                      //高压欠压保护阈值
    uint16_t InputVoltOvr;                      //高压过压保护阈值

    uint16_t LockRotorCarrier;                  //堵转载频   新增
    uint16_t LockRotorTorProt;                  //堵转保护扭矩   新增
    uint16_t LockRotorTimeProt;                 //堵转保护时间   新增
    uint16_t LockRotorSafeTor;                  //堵转安全扭矩   新增

    //泄放参数
    uint16_t SafetyVolt;                        //安全电压
    uint16_t TimeoutSec;                        //泄放超时时间
}COMM_PROTOCOL_STRU;
extern COMM_PROTOCOL_STRU gCommProVar;

//上位机变量-监控参数组
typedef struct COMM_PROTOCOL_SEED_STRU_DEF
{
    //基本参数监控组
    uint16_t MotorSpeed;                        //电机转速
    uint16_t TorqueCmd;                         //给定扭矩命令
    uint16_t TorqueOut;                         //当前实际扭矩
    uint16_t Udc;                               //母线电压
    uint16_t LowVolt;                           //控制电源电压
    uint16_t Idc;                               //母线电流

    uint16_t IGBTTemp;                          //IGBT温度（功率板温度）
    uint16_t MotorTemp1;                        //电机温度1
    uint16_t MotorTemp2;                        //电机温度2
    uint16_t KsiVolt;                           //KSI电压值
    uint16_t RunCase;                           //当前运行状态
    uint16_t Fault;                             //故障信息
    uint16_t MotorNum;                          //电机编号信息
    uint16_t SWVersion;                         //软件版本信息

    //内部监控参数组
    uint16_t IUFb;                              //U相电流反馈值
    uint16_t IVFb;                              //V相电流反馈值
    uint16_t IWFb;                              //W相电流反馈值
    uint16_t IdGd;                              //D轴电流指令
    uint16_t IdFb;                              //D轴实际电流
    uint16_t IqGd;                              //Q轴电流指令
    uint16_t IqFb;                              //Q轴实际电流

    uint16_t UdPIOut;                           //D轴电流调节器电压
    uint16_t UqPIOut;                           //Q轴电流调节器电压
    uint16_t UdComp;                            //D轴前馈电压
    uint16_t UqComp;                            //Q轴前馈电压
    uint16_t UdGd;                              //D轴电压输出
    uint16_t UqGd;                              //Q轴电压输出

    uint16_t ModulateIndex;                     //电压利用率
    uint16_t IMPhase;                           //电角度

    //AD采样原值监控组
    uint16_t IUSampleDVal;                     //U相电流AD采样值
    uint16_t IVSampleDVal;                     //V相电流AD采样值
    uint16_t IWSampleDVal;                     //W相电流AD采样值
    uint16_t UDCSampleDVal;                    //母线电压AD采样值
    uint16_t LowVolSampleDVal;                 //控制电源电压AD采样值
    uint16_t IGBTSampleVal;                    //IGBT温度（功率板温度）AD采样值
    uint16_t KSISampleVal;                     //KSI电压采样AD值
    uint16_t Reserve0;                        //加速踏板AD采样值
    uint16_t Reserve2;                          //举升操纵杆AD采样值
    uint16_t Reserve3;                          //下降电磁阀电流采样AD值
    uint16_t Reserve4;                          //主接触器电流采样AD值


    //整车信号组
    uint16_t Reserve5;                       //座椅开关
    uint16_t Reserve6;                      //驻车停车开关
    uint16_t Reserve7;                      //行车制动开关
    uint16_t Reserve8;                       //安全带开关
    uint16_t Reserve9;                       //前进开关
    uint16_t Reserve10;                      //加速踏板开关
    uint16_t Reserve11;                    //后退开关
    uint16_t Reserve12;                  //举升（低速）
    uint16_t Reserve13;                  //举升（高速）
    uint16_t Reserve14;                     //倾斜
    uint16_t Reserve15;                    //侧移
    uint16_t Reserve16;                 //属具
    uint16_t Reserve17;                 //下降电磁阀电流值
    uint16_t Reserve18;                //主接触器电流值
    uint16_t Reserve19;                //整车运行模式

    //故障记录组
    uint16_t Reserve20;                        //当前故障码
    uint16_t Reserve21;                        //当前给定转速
    uint16_t Reserve22;                        //当前反馈转速
    uint16_t Reserve23;                        //当前给定扭矩
    uint16_t Reserve25;                       //当前实际扭矩
    uint16_t Reserve26;                      //当前ID反馈
    uint16_t Reserve27;                      //当前IQ反馈
    uint16_t Reserve28;                       //当前电机电流
    uint16_t Reserve29;                       //当前母线电压
    uint16_t Reserve30;                      //当前输出电压
    uint16_t Reserve31;                    //当前运行状态
    uint16_t Reserve32;                  //前一次故障码
    uint16_t Reserve33;                  //前一次给定转速
    uint16_t Reserve34;                     //前一次反馈转速
    uint16_t Reserve35;                    //前一次给定扭矩
    uint16_t Reserve36;                 //前一次实际扭矩
    uint16_t Reserve37;                 //前一次ID反馈
    uint16_t Reserve38;                //前一次IQ反馈
    uint16_t Reserve39;                //前一次电机电流
    uint16_t Reserve40;                      //前一次母线电压
    uint16_t Reserve41;                    //前一次输出电压
    uint16_t Reserve42;                  //前一次运行状态
    uint16_t Reserve43;                  //前二次故障码
    uint16_t Reserve44;                  //前二次给定转速
    uint16_t Reserve45;                     //前二次反馈转速
    uint16_t Reserve46;                    //前二次给定扭矩
    uint16_t Reserve47;                 //前二次实际扭矩
    uint16_t Reserve48;                 //前二次ID反馈
    uint16_t Reserve49;                //前二次IQ反馈
    uint16_t Reserve50;                //前二次电机电流
    uint16_t Reserve51;                //前二次母线电压
    uint16_t Reserve52;                //前二次输出电压
    uint16_t Reserve53;                //前二次运行状态

}COMM_PROTOCOL_SEED_STRU;

extern COMM_PROTOCOL_SEED_STRU gCommSeedVar;

#define IS_AHS_EN           ( 0 != gCommProVar.SpdLimtAHS )    //高速限速使能（扭矩控模式下）
#define IS_T_COMP_EN        1 //( 0 != gCommProVar.TorqueComp )    //转矩补偿使能
#define IS_NEG_SEQ_CTRL_EN  1 //( 0 != gCommProVar.NegSeqCtrl )    //负序控制

extern void ParameterChange(void);
extern void ParGetFromFunction(void);


#endif /* SOURCE_FUNCTION_INCLUDE_TEMPCOMMNN_H_ */
