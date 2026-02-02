/*
 * f_deadBandComp.c
 *
 *  Created on: 2024年12月16日
 *      Author: pc
 */
#include "f_public.h"

DEAD_BAND_STRUCT gDeadBand = {0, 0, 0, 0, 0};
float g_f32DeadZoneComp = 0;

#define DB_ZERO_ZONE        5
#define DIV_DB_ZERO_ZONE    0.2f

void CalDeadBandComp( float IdRef, float IqRef )
{
    float f32Temp_Ialpha, f32Temp_Ibeta;
    float f32Temp_IaRef, f32Temp_IbRef, f32Temp_IcRef;

    f32Temp_Ialpha  =   IdRef*gPhase.ICOS - IqRef*gPhase.ISIN;
    f32Temp_Ibeta   =   IdRef*gPhase.ISIN + IqRef*gPhase.ICOS;

    f32Temp_IaRef   =   f32Temp_Ialpha;
    f32Temp_IbRef   =   -0.5f*f32Temp_Ialpha + Cnst_SQRT3div2*f32Temp_Ibeta;
    f32Temp_IcRef   =   -0.5f*f32Temp_Ialpha - Cnst_SQRT3div2*f32Temp_Ibeta;

    if( f32Temp_IaRef >= DB_ZERO_ZONE )
    {
        gDeadBand.CompU = -g_f32DeadZoneComp;//50;
    }
    else if( f32Temp_IaRef < -DB_ZERO_ZONE )
    {
        gDeadBand.CompU = g_f32DeadZoneComp;
    }
    else
    {
        gDeadBand.CompU = -g_f32DeadZoneComp*f32Temp_IaRef*DIV_DB_ZERO_ZONE;
    }

    if( f32Temp_IbRef >= DB_ZERO_ZONE )
    {
        gDeadBand.CompV = -g_f32DeadZoneComp;
    }
    else if( f32Temp_IbRef < -DB_ZERO_ZONE )
    {
        gDeadBand.CompV = g_f32DeadZoneComp;
    }
    else
    {
        gDeadBand.CompV = -g_f32DeadZoneComp*f32Temp_IbRef*DIV_DB_ZERO_ZONE;
    }

    if( f32Temp_IcRef >= DB_ZERO_ZONE )
    {
        gDeadBand.CompW = -g_f32DeadZoneComp;
    }
    else if( f32Temp_IcRef < -DB_ZERO_ZONE )
    {
        gDeadBand.CompW = g_f32DeadZoneComp;
    }
    else
    {
        gDeadBand.CompW = -g_f32DeadZoneComp*f32Temp_IcRef*DIV_DB_ZERO_ZONE;
    }
}
