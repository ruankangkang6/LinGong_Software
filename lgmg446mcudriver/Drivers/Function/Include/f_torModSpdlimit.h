/*
 * f_torModSpdlimit.h
 *
 *  Created on: 2024年12月16日
 *      Author: pc
 */

#ifndef SOURCE_FUNCTION_INCLUDE_F_TORMODSPDLIMIT_H_
#define SOURCE_FUNCTION_INCLUDE_F_TORMODSPDLIMIT_H_


extern uint16_t g_u16HighSpeedRegulator;
extern uint16_t g_u16AhsExitCnt;


extern uint16_t ReadyToEnterHighSpeedRegulator( void );
extern void EnterHighSpeedRegulator( void );
extern void RunHighSpeedRegulator( void );

#endif /* SOURCE_FUNCTION_INCLUDE_F_TORMODSPDLIMIT_H_ */
