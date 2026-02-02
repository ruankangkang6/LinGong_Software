/*
 * f_runCaseDeal22.h
 *
 *  Created on: 2024年12月3日
 *      Author: pc
 */

#ifndef F_RUNCASEDEAL22_H_
#define F_RUNCASEDEAL22_H_


//控制命令
#define TORQUECTRL_CMD           0x02       //扭矩控制
#define SPEEDCTRL_CMD            0x03       //转速控制
#define ACTDISCHARGE_CMD         0x06       //主动泄放
#define PARAMEST_CMD             0x09       //参数辨识

//运行状态
#define STATUS_LOWPOWER         1          //待机状态
#define STATUS_READY            2          //准备状态
#define STATUS_RUNNING          3          //运行状态
#define STATUS_PARAMEST         9          //参数辨识
#define STATUS_UPDATE_FIRMWARE  10         //固件升级

//控制模式
#define MODE_TORQUECTRL         0x02       //扭矩控制
#define MODE_SPEEDCTRL          0x03       //转速控制
#define MODE_NONELOCKERROR      0x05       //非锁定故障
#define MODE_ACTDISCHARGE       0x06       //主动泄放


struct MAIN_COMMAND_STRUCT_DEF{
   uint16_t Start;                 //电机启动标志   0:停机, 1:起动
   uint16_t RunDir;                //电机运行方向   0:正转, 1:反转
   uint16_t TorqueCtl;             //转矩控制标志   0:转速, 1:扭矩
   uint16_t ActvDischg;            //主动放电           0:未泄放, 1:泄放
}; //主命令字的bit安排


extern void RunningDeal(void);


extern uint16_t g_McuEnableReq;
extern uint16_t g_McuModeRe;
extern uint16_t g_McuRunDir;
extern uint16_t g_RunStatus;
extern uint16_t g_McuStMode;

extern uint16_t g_StartFlag;
extern uint16_t g_TorqueCtlFlag;

#endif /* F_RUNCASEDEAL22_H_ */

