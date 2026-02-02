/*
 * f_deadBandComp.h
 *
 *  Created on: 2024年12月16日
 *      Author: pc
 */

#ifndef SOURCE_FUNCTION_INCLUDE_F_DEADBANDCOMP_H_
#define SOURCE_FUNCTION_INCLUDE_F_DEADBANDCOMP_H_


#define     DEADDANDTIME           225       //6us死区时间

typedef struct DEAD_BAND_STRUCT_DEF{
   int16_t      DeadBand;           //死区时间(20ns单位)
   int16_t      DeadComp;           //死区时间补偿(20ns单位)
   float  CompU;
   float  CompV;
   float  CompW;
}DEAD_BAND_STRUCT;  //死区补偿相关变量


extern DEAD_BAND_STRUCT gDeadBand;
extern float g_f32DeadZoneComp;
extern void CalDeadBandComp( float IdRef, float IqRef );

#endif /* SOURCE_FUNCTION_INCLUDE_F_DEADBANDCOMP_H_ */
