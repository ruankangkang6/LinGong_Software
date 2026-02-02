/*
 * f_torqueComp.h
 *
 *  Created on: 2024年12月16日
 *      Author: pc
 */

#ifndef SOURCE_FUNCTION_INCLUDE_F_TORQUECOMP_H_
#define SOURCE_FUNCTION_INCLUDE_F_TORQUECOMP_H_


typedef struct TORQUE_REF_COMP_STRUCT_DEF
{
    float TRefCompKp;
    float TRefCompVal;
    float TRefCompMaxVal;
    float TRefCompMinVal;
    float TRefCompErrSpd;
    float TRefCompSpdRef;
} TORQUE_REF_COMP_STRUCT;
extern TORQUE_REF_COMP_STRUCT g_TorqRefComp;

extern void CalcCompTorque( void );
extern void InitTRefComp( void );

#endif /* SOURCE_FUNCTION_INCLUDE_F_TORQUECOMP_H_ */
