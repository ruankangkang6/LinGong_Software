/*
 * f_faultProtect.c
 *
 *  Created on: 2025 Jan 24
 *      Author: pc
 */
#include "f_public.h"

LIMIT_PARAMS_STRUCT gLimitParams = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
                                    0, 0, 0, 0, 0, 0, 0, 0};        //故障保护点
FaultLv1 gFault1st = {0};                                              //一级故障
FaultLv2 gFault2nd = {0};                                              //二级故障
FaultLv3 gFault3rd = {0};                                              //三级故障
extern  uint16_t gAdcResult1_iw;
extern  uint16_t gAdcResult1_iv;

Uint16 MotorTempUpdateCount10s = 0;
float MotorTempLowRateUpdate = 0.0F;
/*************************************************************
    温度保护
*************************************************************/
void TemperatureProtect(void)
{
    static float m_CoefCnt = 0;
    static float m_CoefNum = 0;

    float m_MaxMotorTemp;
    float Temp;

    if ( 0 != gRotorSpeed.SpeedApply )
    {
        //根据功率(2倍电机功率)和转速算扭矩
        Temp = 1000.0f*gMotorInfo.Power*2.0f*gMotorInfo.Poles/( Value_2Pi*fabs(gRotorSpeed.SpeedApply) ); //转速(rpm)*扭矩(nm)/9550 =功率(kw), 转速n(rpm)=60*电流频率f/电机极对数P，1000/2PI= 9550/60

        if ( Temp > gMotorInfo.MaxTorque )
        {
            Temp = gMotorInfo.MaxTorque;
        }
    }
    else
    {
        Temp = gMotorInfo.MaxTorque;
    }

    //IGBT过温保护判断
    if( gTemperature.Temp >= gLimitParams.igbtTempLimit )
    {
        gTemperature.ErrCnt++;
        if( gTemperature.ErrCnt >= 100 )
        {
            gTemperature.ErrCnt = 0;
            gFault3rd.bit.OverIGBTTemp = 1;
            DisableDrive();
        }
    }
    else
    {
        gTemperature.ErrCnt = 0;
    }


    //
    //驾驶感受不好，改成10s检测一次温度进行限制
    //IGBT过温降功率处理
    if( gTemperature.Temp >= gLimitParams.igbtTempLimit )
    {
        gTemperature.DerateTorqueByTIgbt = Temp*0;
    }
    else if( gTemperature.Temp >= gLimitParams.igbtTempWarn )
    {
        gTemperature.DerateTorqueByTIgbt = Temp*( 1 - ( gTemperature.Temp - gLimitParams.igbtTempWarn )/( gLimitParams.igbtTempLimit - gLimitParams.igbtTempWarn ) );
        gFault2nd.bit.IgbtTempLimTor = 1;
    }
    else
    {
        gTemperature.DerateTorqueByTIgbt = Temp;
        gFault2nd.bit.IgbtTempLimTor = 0;
    }
    Saturatefloat(&gTemperature.DerateTorqueByTIgbt, gMotorInfo.MaxTorque, 0);








    //电机过温保护判断
    if( gTemperature.MotorTemp > gTemperature.MotorTemp2 )
    {
        m_MaxMotorTemp = gTemperature.MotorTemp;
    }
    else
    {
        m_MaxMotorTemp = gTemperature.MotorTemp2;
    }

    if( m_MaxMotorTemp >= gLimitParams.motorTempLimit )
    {
        gTemperature.ErrMotorCnt++;
        if( gTemperature.ErrMotorCnt >= 100 )
        {
            gTemperature.ErrMotorCnt = 0;
            gFault3rd.bit.OverMotorTemp = 1;
            DisableDrive();
        }
    }
    else
    {
        gTemperature.ErrMotorCnt = 0;
    }

    //10s更新一次温度以便完成扭矩限制不平滑
    if(++MotorTempUpdateCount10s > 5000)
    {
    	MotorTempUpdateCount10s = 0;
    	MotorTempLowRateUpdate = m_MaxMotorTemp;
    }

    //电机过温降功率处理
    if( m_MaxMotorTemp >= gLimitParams.motorTempLimit )
    {
        gTemperature.DerateTorqueByTMotor = Temp*0;
    }
    else if( MotorTempLowRateUpdate >= gLimitParams.motorTempWarn )
    {
        gTemperature.DerateTorqueByTMotor = Temp*( 1 - ( MotorTempLowRateUpdate - gLimitParams.motorTempWarn )/( gLimitParams.motorTempLimit - gLimitParams.motorTempWarn ) );
        gFault2nd.bit.MotorTempLimTor = 1;
    }
    else
    {
        gTemperature.DerateTorqueByTMotor = Temp;
        gFault2nd.bit.MotorTempLimTor = 0;
    }
    Saturatefloat(&gTemperature.DerateTorqueByTMotor, gMotorInfo.MaxTorque, 0);


    //限制扭矩
    if (gFault2nd.bit.IGBTSenHighFault == 1 || gFault2nd.bit.IGBTSenLowFault == 1 || gFault2nd.bit.Motor1SenHighFault == 1 || gFault2nd.bit.Motor1SenLowFault == 1)
    {
    	m_CoefCnt++;
    	if (m_CoefCnt >= 100)
    	{
    		m_CoefCnt = 0;
    		m_CoefNum +=0.1f;
    	}

		if (m_CoefNum >= 0.5f)
		{
			m_CoefCnt = 0;
			m_CoefNum = 0.5f;
		}

        g_DerateTorq = gMotorInfo.MaxTorque * (1 - m_CoefNum);
    }
    else
    {
    	m_CoefCnt = 0;
    	m_CoefNum = 0;
        g_DerateTorq = gMotorInfo.MaxTorque;
    }




    if (g_DerateTorq > gTemperature.DerateTorqueByTMotor )
    {
        g_DerateTorq = gTemperature.DerateTorqueByTMotor;
    }

    if (g_DerateTorq > gTemperature.DerateTorqueByTIgbt)
    {
        g_DerateTorq = gTemperature.DerateTorqueByTIgbt;
    }
}

/*************************************************************
    堵转保护
*************************************************************/
#define STALL_CURRENT   450
// #define STALL_CURRENT   120  测试使用
#define STALL_CURRENT1   350 //堵转下松油门退出堵转故障的电流阈值

int16_t Stall_Cnt = 0,Stall_Fault = 0,Stall_Fault_Recover = 0,Stall_Fault_Recovercnt = 0,Temp_cnt=0,TempDerate_Cnt = 0, DerateCnt1 = 0;
float IGBTTemp_old = 0.0;
void StallProtect(void)
{
	float s_StallDerateTorq = 0;

	//堵转状态限扭矩
	if ( (fabs(gRotorSpeed.SpeedApplyFilter)<3)
       &&(gLineCur.CurrentFilter> STALL_CURRENT
    	  || gCurSamp.U > STALL_CURRENT
	      || gCurSamp.V > STALL_CURRENT
		  || gCurSamp.W > STALL_CURRENT
		  || gCurSamp.U < -STALL_CURRENT
		  || gCurSamp.V < -STALL_CURRENT
		  || gCurSamp.W < -STALL_CURRENT
		 )
	   )
	{
		Stall_Cnt = Stall_Cnt + 1;
	}

	if (gLineCur.CurrentFilter < STALL_CURRENT1
			&& gCurSamp.U > -STALL_CURRENT1
			&& gCurSamp.V > -STALL_CURRENT1
			&& gCurSamp.W > -STALL_CURRENT1
			&& gCurSamp.U < STALL_CURRENT1
			&& gCurSamp.V < STALL_CURRENT1
			&& gCurSamp.W < STALL_CURRENT1
			 )
	{
		Stall_Cnt = Stall_Cnt - 1;
	}

	if (Stall_Cnt <= 0)
	{
		Stall_Cnt = 0;
	}

	if (Stall_Cnt> 10000)  //10s //堵转条件满足且累计时间达到，进入堵转状态
	{
		Stall_Fault = 1;
		Stall_Cnt = 10001;
	}
	else if (Stall_Fault_Recover == 1 || Stall_Cnt == 0) //“堵转条件不满足且累计时间达到”或者“转速大于150rpm”。退出堵转状态
	{
		Stall_Fault = 0;
		Stall_Cnt = 0;
	}
	else
	{
		;
	}

	if (Stall_Fault_Recover == 1 || Stall_Cnt == 0) //“堵转条件不满足且累计时间达到”或者“转速大于150rpm”。退出堵转状态
	{
		Stall_Fault = 0;
		Stall_Cnt = 0;
	}

	if (fabs(gRotorSpeed.SpeedApplyFilter)>10)
	{
		Stall_Fault_Recovercnt ++;
	}
	else
	{
		Stall_Fault_Recovercnt = 0;
	}

	if (Stall_Fault_Recovercnt > 500)  //转速频率大于10，0.5s后退出堵转
	{
		Stall_Fault_Recovercnt = 501;
		Stall_Fault_Recover = 1;
	}
	else
	{
		Stall_Fault_Recover = 0;
	}

	//堵转IGBT温升过快限扭矩
//	Temp_cnt ++;
//	if (Temp_cnt > 1000)    //判断周期1s
//	{
//		Temp_cnt = 0;
//
//		if (gTemperature.Temp - IGBTTemp_old >5)  //温升速率超过5℃/s
//		{
//			TempDerate_Cnt ++;
//		}
//		else
//		{
//			TempDerate_Cnt --;
//		}
//
//		if (TempDerate_Cnt >= 6)
//		{
//			TempDerate_Cnt = 6;
//		}
//		else if (TempDerate_Cnt <= 0)
//		{
//			TempDerate_Cnt = 0;
//		}
//		else
//		{
//			;
//		}
//
//		IGBTTemp_old = gTemperature.Temp;
//	}


	//扭矩限制处理
	s_StallDerateTorq = ((float)(10-TempDerate_Cnt*1.5))/10 * gMotorInfo.MaxTorque;

	if (Stall_Fault == 1)
	{
		DerateCnt1++;
		s_StallDerateTorq = ((float)(10-TempDerate_Cnt*1.5))/10 * gMotorInfo.MaxTorque;
		if (DerateCnt1 > 160)
		{
			DerateCnt1 = 160;
		}
		s_StallDerateTorq = ((float)(200-DerateCnt1))/200*s_StallDerateTorq;
	}
	else
	{
		DerateCnt1 = 0;
	}

	if (s_StallDerateTorq < 0)
	{
		s_StallDerateTorq = 0;
	}
	else if (s_StallDerateTorq > gMotorInfo.MaxTorque)
	{
		s_StallDerateTorq = gMotorInfo.MaxTorque;
	}
	else
	{
		;
	}

    if (g_DerateTorq > s_StallDerateTorq)
    {
        g_DerateTorq = s_StallDerateTorq;
    }
}

/*************************************************************
    温度传感器故障检测
*************************************************************/
void SenLowFaultCheck(void)
{
	static int16_t Motor1SenLowFault_Cnt = 0;
	static int16_t Motor2SenLowFault_Cnt = 0;
	static int16_t Motor1SenHighFault_Cnt = 0;
	static int16_t Motor2SenHighFault_Cnt = 0;
	static int16_t IGBTSenLowFault_Cnt = 0;
	static int16_t IGBTSenHighFault_Cnt = 0;
	static int16_t AmbSenLowFault_Cnt = 0;
	static int16_t AmbSenHighFault_Cnt = 0;


	if (gTemperature.MotorTempAD < 10)
	{
		Motor1SenLowFault_Cnt++;
	}
	else
	{
		Motor1SenLowFault_Cnt = 0;
	}

	if (Motor1SenLowFault_Cnt > 5000)
	{
		Motor1SenLowFault_Cnt = 5100;
		gFault2nd.bit.Motor1SenLowFault = 1;
	}
	else
	{
		gFault2nd.bit.Motor1SenLowFault = 0;
	}
//	if (gTemperature.MotorTempAD2 < 10)
//	{
//		Motor2SenLowFault_Cnt++;
//	}
//	else
//	{
//		Motor2SenLowFault_Cnt = 0;
//	}
//
//	if (Motor2SenLowFault_Cnt > 5000)
//	{
//		Motor2SenLowFault_Cnt = 5100;
//		gFault2nd.bit.Motor2SenLowFault = 1;
//	}


	if (gTemperature.TempAD < 10)
	{
		IGBTSenLowFault_Cnt++;
	}
	else
	{
		IGBTSenLowFault_Cnt = 0;
	}

	if (IGBTSenLowFault_Cnt > 5000)
	{
		IGBTSenLowFault_Cnt = 5100;
		gFault2nd.bit.IGBTSenLowFault = 1;
	}
	else
	{
		gFault2nd.bit.IGBTSenLowFault = 0;
	}
//	if (gTemperature.TempAMBAD2 < 50)
//	{
//		AmbSenLowFault_Cnt++;
//	}
//	else
//	{
//		AmbSenLowFault_Cnt = 0;
//	}
//
//	if (AmbSenLowFault_Cnt > 5000)
//	{
//		AmbSenLowFault_Cnt = 5100;
//		gFault2nd.bit.AmbSenLowFault = 1;
//	}

	if (gTemperature.MotorTempAD >4080)
	{
		Motor1SenHighFault_Cnt++;
	}
	else
	{
		Motor1SenHighFault_Cnt = 0;
	}

	if (Motor1SenHighFault_Cnt > 5000)
	{
		Motor1SenHighFault_Cnt = 5100;
		gFault2nd.bit.Motor1SenHighFault = 1;
	}
	else
	{
		gFault2nd.bit.Motor1SenHighFault = 0;
	}
//	if (gTemperature.MotorTempAD2 >4000)
//	{
//		Motor2SenHighFault_Cnt++;
//	}
//	else
//	{
//		Motor2SenHighFault_Cnt = 0;
//	}
//
//	if (Motor2SenHighFault_Cnt > 5000)
//	{
//		Motor2SenHighFault_Cnt = 5100;
//		gFault2nd.bit.Motor2SenHighFault = 1;
//	}

	if (gTemperature.TempAD >4080)
	{
		IGBTSenHighFault_Cnt++;
	}
	else
	{
		IGBTSenHighFault_Cnt = 0;
	}

	if (IGBTSenHighFault_Cnt > 5000)
	{
		IGBTSenHighFault_Cnt = 5100;
		gFault2nd.bit.IGBTSenHighFault = 1;
	}
	else
	{
		gFault2nd.bit.IGBTSenHighFault = 0;
	}
//	if (gTemperature.TempAMBAD2 >4000)
//	{
//		AmbSenHighFault_Cnt++;
//	}
//	else
//	{
//		AmbSenHighFault_Cnt = 0;
//	}
//
//	if (AmbSenHighFault_Cnt > 5000)
//	{
//		AmbSenHighFault_Cnt = 5100;
//		gFault2nd.bit.AmbSenHighFault = 1;
//	}
}


/*************************************************************
    零漂检验
*************************************************************/
#define IAC_OFFSET_LIMIT    80
void CalcADOffset( void )
{
    if( 1 == gADOffsetInfo.Flag )
        return;

    if(  (STATUS_LOWPOWER != g_RunStatus)
      && (STATUS_READY != g_RunStatus)
      )
    {
        gADOffsetInfo.EnableCount = 0;
        return;
    }

    gADOffsetInfo.EnableCount++;
    gADOffsetInfo.EnableCount = ( gADOffsetInfo.EnableCount > 200 ) ? 200 : gADOffsetInfo.EnableCount;

    if( gADOffsetInfo.EnableCount < 200 )//前200次不检测
    {
        gADOffsetInfo.Count     =   0;

        g_ADConvertSum.offset.f32Ia = 0;
        g_ADConvertSum.offset.f32Ib = 0;
        g_ADConvertSum.offset.f32Ic = 0;
        return;
    }

    g_ADConvertSum.offset.f32Ia  += g_SampleRealValue.f32Ia;
    g_ADConvertSum.offset.f32Ib  += g_SampleRealValue.f32Ib;
    g_ADConvertSum.offset.f32Ic  += g_SampleRealValue.f32Ic;


    gADOffsetInfo.Count++;

    if( gADOffsetInfo.Count >= 2000 )                   //累计2000次，计算采样偏置平均值作为零漂
    {
        if( 0 == gADOffsetInfo.Flag )
        {
            gADOffsetInfo.Flag = 1;
            g_ADConvert.offset.f32Ia  = g_ADConvertSum.offset.f32Ia*0.5e-3f;
            g_ADConvert.offset.f32Ib  = g_ADConvertSum.offset.f32Ib*0.5e-3f;
            g_ADConvert.offset.f32Ic  = g_ADConvertSum.offset.f32Ic*0.5e-3f;
        }

        gADOffsetInfo.Count     =   0;
        g_ADConvertSum.offset.f32Ia =   0;
        g_ADConvertSum.offset.f32Ib =   0;
        g_ADConvertSum.offset.f32Ic =   0;

#if ( 1 == HALL_NUM_3 )
        if( fabs(g_ADConvert.offset.f32Ia) > gLimitParams.IACOffsetLimit || fabs(g_ADConvert.offset.f32Ib) > gLimitParams.IACOffsetLimit || fabs(g_ADConvert.offset.f32Ic) > gLimitParams.IACOffsetLimit || fabs(g_ADConvert.offset.f32Ib) > gLimitParams.IACOffsetLimit )
#else
		if ( (fabs(g_ADConvert.offset.f32Ic) > gLimitParams.IACOffsetLimit)
		   ||(fabs(g_ADConvert.offset.f32Ib) > gLimitParams.IACOffsetLimit)
		   )
#endif
        {
        	gFault3rd.bit.ErrorCurrentCheck = 1;
            DisableDrive();
            gADOffsetInfo.EnableCount = 0;
        }
    }
}


/*************************************************************
    旋变故障检测
*************************************************************/
void RTCheck(void)
{
	static uint16_t S_RTCheckCnt = 0;

	S_RTCheckCnt++;
	if (S_RTCheckCnt >= 1000)
	{
	    if ( gRotorTrans.Status != 0 )
	    {
			DisableDrive();
	    	gFault3rd.bit.ErrorRTTrans = 1;
	    }

	}
}


/*************************************************************
    软件过流检验
*************************************************************/
float gCurSampIa[3] = {0, 0, 0};
float gCurSampIb[3] = {0, 0, 0};
float gCurSampIc[3] = {0, 0, 0};
uint8_t ADC_fault_index = 0;
float sum_gCurFault=0;
float gCurocpIa=0,gCurocpIb=0,gCurocpIc=0;

float Curresult_iu[50]={0,0,0,0,0,0,0,0,0,0,
		                0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0};
float Curresult_iv[50]={0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0};
float Curresult_iw[50]={0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0,
						0,0,0,0,0,0,0,0,0,0};
float Cursum_iu=0,Cursum_iv=0,Cursum_iw=0;
uint8_t Cur_index=0;

void ImmSoftWareErrorDeal( void )
{
//	float temp;
//
//    gCurSampIa[ADC_fault_index%3]=gCurSamp.U;
//    gCurSampIb[ADC_fault_index%3]=gCurSamp.V;
//    gCurSampIc[ADC_fault_index%3]=gCurSamp.W;
//    ADC_fault_index++;
//    ADC_fault_index = (ADC_fault_index%3);
//    if(ADC_fault_index== 2)
//    {
//    	sum_gCurFault = gCurSampIa[0]+gCurSampIa[1]+gCurSampIa[2];
//    	temp = gCurSampIa[0];
//    	if(fabs(gCurSampIa[0])<fabs(gCurSampIa[1]))
//    	{
//    		temp = gCurSampIa[1];
//    	}
//    	if(fabs(temp)<fabs(gCurSampIa[2]))
//    	{
//    		temp = gCurSampIa[2];
//    	}
//    	sum_gCurFault -= temp;
//    	gCurocpIa = sum_gCurFault*0.5;
//
//    	sum_gCurFault = gCurSampIb[0]+gCurSampIb[1]+gCurSampIb[2];
//		temp = gCurSampIb[0];
//		if(fabs(gCurSampIb[0])<fabs(gCurSampIb[1]))
//		{
//			temp = gCurSampIb[1];
//		}
//		if(fabs(temp)<fabs(gCurSampIb[2]))
//		{
//			temp = gCurSampIb[2];
//		}
//		sum_gCurFault -= temp;
//		gCurocpIb = sum_gCurFault*0.5;
//
//		sum_gCurFault = gCurSampIc[0]+gCurSampIc[1]+gCurSampIc[2];
//		temp = gCurSampIc[0];
//		if(fabs(gCurSampIc[0])<fabs(gCurSampIc[1]))
//		{
//			temp = gCurSampIc[1];
//		}
//		if(fabs(temp)<fabs(gCurSampIc[2]))
//		{
//			temp = gCurSampIc[2];
//		}
//		sum_gCurFault -= temp;
//		gCurocpIc = sum_gCurFault*0.5;
//    }

	if(Cur_index<49)
	{
		Cur_index++;
	}
	else
	{
		Cur_index=0;
	}

	Cursum_iu+=gCurSamp.U;
	Cursum_iu-=Curresult_iu[Cur_index];
	Curresult_iu[Cur_index] = gCurSamp.U;
	gCurocpIa = Cursum_iu/50;

	Cursum_iv+=gCurSamp.V;
	Cursum_iv-=Curresult_iv[Cur_index];
	Curresult_iv[Cur_index] = gCurSamp.V;
	gCurocpIb = Cursum_iv/50;

	Cursum_iw+=gCurSamp.W;
	Cursum_iw-=Curresult_iw[Cur_index];
	Curresult_iw[Cur_index] = gCurSamp.W;
	gCurocpIc = Cursum_iw/50;


    if(  (STATUS_RUNNING == g_RunStatus)
      || (STATUS_PARAMEST == g_RunStatus)
      || (STATUS_READY == g_RunStatus)
      )
    {
        if( gCurSamp.U > gLimitParams.OcpInstLimit
            || gCurSamp.V > gLimitParams.OcpInstLimit
            || gCurSamp.W > gLimitParams.OcpInstLimit
            || gCurSamp.U < -gLimitParams.OcpInstLimit
            || gCurSamp.V < -gLimitParams.OcpInstLimit
            || gCurSamp.W < -gLimitParams.OcpInstLimit
            || gLineCur.CurrentFilter > gLimitParams.OcpFilterLimit )
        {
//        if( gCurocpIa > gLimitParams.OcpInstLimit
//            || gCurocpIb > gLimitParams.OcpInstLimit
//            || gCurocpIc > gLimitParams.OcpInstLimit
//            || gCurocpIa < -gLimitParams.OcpInstLimit
//            || gCurocpIb < -gLimitParams.OcpInstLimit
//            || gCurocpIc < -gLimitParams.OcpInstLimit
////            || gLineCur.CurrentFilter > gLimitParams.OcpFilterLimit
//			)
//        {
            DisableDrive();
//            DisableSlavePWM();关波后延迟一会

//            gMainStatus.ErrFlag.bit.SwOCPFlag = 1;
            gFault3rd.bit.SwOverCurrent = 1;

            gAdcResult1_iw = u16AdcResult0_iw;
            gAdcResult1_iv = u16AdcResult1_iv;
            gLineCur.ErrorShow = gLineCur.Current;
        }
    }
}


/*************************************************************
    硬件过流检测
*************************************************************/
void HardWareOCDeal(void)
{
	DisableDrive();								//首先封锁输出
	gFault3rd.bit.HwOverCurrent = 1;
	gLineCur.ErrorShow = MaxUVWCurrent();	//硬件过流，记录故障电流
}

float MaxUVWCurrent( void )
{
	float m_IU, m_IV, m_IW;

	m_IU = fabs(gIUVW.U);
	m_IV = fabs(gIUVW.V);
	m_IW = fabs(gIUVW.W);

	if ( m_IU >= m_IV )
	{
		if ( m_IU >= m_IW )
		{
			return gIUVW.U;
		}
		else
		{
			return gIUVW.W;
		}
	}
	else
	{
		if ( m_IV >= m_IW )
		{
			return gIUVW.V;
		}
		else
		{
			return gIUVW.W;
		}
	}
}


/*************************************************************
	软件过压欠压处理
*************************************************************/
void SoftWareErrorDeal(void)
{
    /***************************低压检测处理****************************/
//	LowVoltCheck();

    if( STATUS_LOWPOWER == g_RunStatus )
	{
		return;										//欠压状态下不判断软件故障
	}

    /***************************过压欠压处理****************************/
	if( gUDC.uDCBigFilter > gLimitParams.inputVoltOvr )//过压判断
	{
	    DisableDrive();
		gFault3rd.bit.SwOverUdc = 1;

	}
	else if( (MODE_ACTDISCHARGE != g_McuStMode) && (gUDC.uDCBigFilter < gLimitParams.inputVoltOff) && (MainContactorStatus == 1)) //欠压判断,使用大滤波电压
	{
		DisableDrive();

		g_RunStatus = STATUS_LOWPOWER;
		gFault3rd.bit.SwUnderUdc = 1;
	}

    /***************************超速处理****************************/
	if( IS_POSITIVE_ROTATE_WITH_SPEED_N )
	{
		if ( ( gRotorSpeed.SpeedApply > 0 && gRotorSpeed.SpeedApply > gLimitParams.speedNLimit )	// negative rotate
	  	|| ( gRotorSpeed.SpeedApply < 0 && gRotorSpeed.SpeedApply < -gLimitParams.speedPLimit ) )	// positive rotate
		{
			DisableDrive();
			gFault3rd.bit.OverSpeed = 1;
		}
	}
	else
	{
		if ( ( gRotorSpeed.SpeedApply > 0 && gRotorSpeed.SpeedApply > gLimitParams.speedPLimit )
	  	|| ( gRotorSpeed.SpeedApply < 0 && gRotorSpeed.SpeedApply < -gLimitParams.speedNLimit ) )
		{
			DisableDrive();
			gFault3rd.bit.OverSpeed = 1;
		}
	}
}
