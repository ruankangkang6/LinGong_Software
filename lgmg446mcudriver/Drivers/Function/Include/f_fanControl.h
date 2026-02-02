/*
 * f_fanControl.h
 *
 *  Created on: 2024年12月2日
 *      Author: pc
 */

#ifndef FANCONTROL_H_
#define FANCONTROL_H_


#define  WATER_COLD     0  //风扇功能关
#define  WIND_COLD      1  //风扇功能开

#define  COLD_MODE  WIND_COLD              //还涉及风扇外设配置的初始化EPwm4Regs和EPwm6Regs，设计风扇故障校验(未调用)，设计变量定义。

#define  HALF_INIT_FAN_PRD    250



#if ( COLD_MODE == WIND_COLD )
#define IS_FAN_FAULT    ( 1 == GpioDataRegs.GPBDAT.bit.GPIO42 )
#define IS_FAN_RUNNING  ( 0x00 == EPwm6Regs.AQCSFRC.bit.CSFA && 0x00 == EPwm4Regs.AQCSFRC.bit.CSFA)
#endif

extern uint16_t FanDuty;

extern void FanDutyControl( void );
extern void FanFaultCheck( void );

#endif /* FANCONTROL_H_ */
