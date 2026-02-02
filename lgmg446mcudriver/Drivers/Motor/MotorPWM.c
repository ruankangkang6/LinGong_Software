/*
 * MotorPWM.c
 *
 *  Created on: 2024年12月13日
 *      Author: pc
 */
#include "f_public.h"
#include "main.h"

#define PWMSENDTEST 0


//#pragma CODE_SECTION(SendPWM,               "ramfuncs")
//#pragma CODE_SECTION(OutPutPWMVF,           "ramfuncs")
//#pragma CODE_SECTION(OverModulate,          "ramfuncs")
//#pragma CODE_SECTION(SVPWM_Modulate_S7,     "ramfuncs")

PWM_OUT_STRUCT gPWM = {0, 0, 0, 0, 0, 0};
uint16_t g_PwmEnable = 0;                                          //开波使能
uint16_t g_PwmAlreadyOn = 0;

void OutPutPWMVF(void);
void SendPWM(void);
void SVPWM_Modulate_S7( float f32Valpha, float f32Vbeta, float* pf32CompA, float* pf32CompB, float* pf32CompC );
void OverModulate( float *f32t1, float *f32t2 );
float gValpha =0;
float gVbeta = 0;
/*************************************************************
    计算PWM开关信号
*************************************************************/

void OutPutPWMVF(void)
{
    float Valpha, Vbeta;

    float f32ModulationD;
    float f32ModulationQ;
    float f32divUdc;

    if ( 0 == gUDC.uDC )
    {
        f32divUdc = gPWM.gPWMPrdApply;
    }
    else
    {
        f32divUdc = gPWM.gPWMPrdApply/gUDC.uDCBigFilter;
    }

    f32ModulationD = gUMTSet.M*f32divUdc;
    f32ModulationQ = gUMTSet.T*f32divUdc;

    //ParK逆变换
    Valpha = f32ModulationD*gPhase.ICOS - f32ModulationQ*gPhase.ISIN;
    Vbeta  = f32ModulationD*gPhase.ISIN + f32ModulationQ*gPhase.ICOS;

    gValpha = Valpha;
    gVbeta = Vbeta;
    //七段式
    SVPWM_Modulate_S7( Valpha, Vbeta, &(gPWM.U), &(gPWM.V), &(gPWM.W) );

    //P波输出
    SendPWM();
}

/*************************************************************
    SVPWM调制（七段式）
*************************************************************/
void SVPWM_Modulate_S7( float f32Valpha, float f32Vbeta, float* pf32CompA, float* pf32CompB, float* pf32CompC )
{
    float f32Va, f32Vb, f32Vc, f32t1, f32t2;

	float CompareMax = (float)(gPWM.gPWMPrdApply - 270);
	float CompareMin = 270.0;
	float PWM_max = (float)(gPWM.gPWMPrdApply - 1);
	float PWM_min = 1;

    uint32_t u32Sector = 0;


    // Inverse clarke transformation
    f32Va = f32Vbeta;
    f32Vb = (-0.5)*f32Vbeta + Cnst_SQRT3div2*f32Valpha;
    f32Vc = (-0.5)*f32Vbeta - Cnst_SQRT3div2*f32Valpha;

    // 60 degree Sector determination
    if ( f32Va > 0 )
        u32Sector = 1;
    if ( f32Vb > 0 )
        u32Sector = u32Sector + 2;
    if ( f32Vc > 0 )
        u32Sector = u32Sector + 4;


    // X,Y,Z (Va,Vb,Vc) calculations
    f32Va = f32Vbeta;                                   // X = Va
    f32Vb = 0.5*f32Vbeta + Cnst_SQRT3div2*f32Valpha;    // Y = Vb
    f32Vc = 0.5*f32Vbeta - Cnst_SQRT3div2*f32Valpha;    // Z = Vc

    // Sector 0: this is special case for (Ualpha,Ubeta) = (0,0)
    switch( u32Sector )
    {
    case 0:
        *pf32CompA = 0.5f * gPWM.gPWMPrdApply;
        *pf32CompB = 0.5f * gPWM.gPWMPrdApply;
        *pf32CompC = 0.5f * gPWM.gPWMPrdApply;

        break;

        // Sector 1: t1=Z and t2=Y (abc ---> Tb,Ta,Tc)
    case 1:
        f32t1 = f32Vc;
        f32t2 = f32Vb;

        OverModulate(&f32t1, &f32t2);

        *pf32CompB = 0.5f*( gPWM.gPWMPrdApply - f32t1 - f32t2 );// tbon = (1-t1-t2)/2
//        *pf32CompB = *pf32CompB;
        *pf32CompA = *pf32CompB + f32t1;                    // taon = tbon+t1
        *pf32CompC = *pf32CompA + f32t2;                    // tcon = taon+t2

        break;

        // Sector 2: t1=Y and t2=-X (abc ---> Ta,Tc,Tb)
    case 2:
        f32t1 = f32Vb;
        f32t2 = -f32Va;

        OverModulate(&f32t1, &f32t2);

        *pf32CompA = 0.5f*( gPWM.gPWMPrdApply - f32t1 - f32t2 );    // taon = (1-t1-t2)/2
//        *pf32CompA = *pf32CompA;
        *pf32CompC = *pf32CompA + f32t1;                    // tcon = taon+t1
        *pf32CompB = *pf32CompC + f32t2;                    // tbon = tcon+t2

        break;

        // Sector 3: t1=-Z and t2=X (abc ---> Ta,Tb,Tc)
    case 3:
        f32t1 = -f32Vc;
        f32t2 = f32Va;

        OverModulate(&f32t1, &f32t2);

        *pf32CompA = 0.5f*( gPWM.gPWMPrdApply - f32t1 - f32t2 );    // taon = (1-t1-t2)/2
//        *pf32CompA = *pf32CompA;
        *pf32CompB = *pf32CompA + f32t1;                    // tbon = taon+t1
        *pf32CompC = *pf32CompB + f32t2;                    // tcon = tbon+t2

        break;

        // Sector 4: t1=-X and t2=Z (abc ---> Tc,Tb,Ta)
    case 4:
        f32t1 = -f32Va;
        f32t2 = f32Vc;

        OverModulate(&f32t1, &f32t2);

        *pf32CompC = 0.5f*( gPWM.gPWMPrdApply - f32t1 - f32t2 );    // tcon = (1-t1-t2)/2
//        *pf32CompC = *pf32CompC;
        *pf32CompB = *pf32CompC + f32t1;                    // tbon = tcon+t1
        *pf32CompA = *pf32CompB + f32t2;                    // taon = tbon+t2

        break;

        // Sector 5: t1=X and t2=-Y (abc ---> Tb,Tc,Ta)
    case 5:
        f32t1 = f32Va;
        f32t2 = -f32Vb;

        OverModulate(&f32t1, &f32t2);

        *pf32CompB = 0.5f*( gPWM.gPWMPrdApply - f32t1 - f32t2 );    // tbon = (1-t1-t2)/2
//        *pf32CompB = *pf32CompB;
        *pf32CompC = *pf32CompB + f32t1;                    // tcon = tbon+t1
        *pf32CompA = *pf32CompC + f32t2;                    // taon = tcon+t2

        break;

        // Sector 6: t1=-Y and t2=-Z (abc ---> Tc,Ta,Tb)
    case 6:
        f32t1 = -f32Vb;
        f32t2 = -f32Vc;

        OverModulate(&f32t1, &f32t2);

        *pf32CompC = 0.5f*( gPWM.gPWMPrdApply - f32t1 - f32t2 );    // tcon = (1-t1-t2)/2
//        *pf32CompC = *pf32CompC;
        *pf32CompA = *pf32CompC + f32t1;                    // taon = tcon+t1
        *pf32CompB = *pf32CompA + f32t2;                    // tbon = taon+t2

        break;

    default:
        break;
    }

    *pf32CompA += gDeadBand.CompU;
    *pf32CompB += gDeadBand.CompV;
    *pf32CompC += gDeadBand.CompW;

	if(*pf32CompA > CompareMax)
	{
		*pf32CompA = PWM_max;
	}
	else if (*pf32CompA < CompareMin)
	{
		*pf32CompA = PWM_min;
	}
	else
	{
		;
	}

	if(*pf32CompB > CompareMax)
	{
		*pf32CompB = PWM_max;
	}
	else if (*pf32CompB < CompareMin)
	{
		*pf32CompB = PWM_min;
	}
	else
	{
		;
	}
	if(*pf32CompC > CompareMax)
	{
		*pf32CompC = PWM_max;
	}
	else if (*pf32CompC < CompareMin)
	{
		*pf32CompC = PWM_min;
	}
	else
	{
		;
	}
}

/*************************************************************
    过调限制
*************************************************************/
void OverModulate( float *f32t1, float *f32t2 )
{
    float divLen;

    if( *f32t1 + *f32t2 > gPWM.gPWMPrdApply )
    {
        if( *f32t1 > gPWM.gPWMPrdApply && *f32t1 >= *f32t2 )
        {
            *f32t1 = gPWM.gPWMPrdApply;
            *f32t2 = 0;
        }
        else if( *f32t2 > gPWM.gPWMPrdApply && *f32t2 >= *f32t1 )
        {
            *f32t1 = 0;
            *f32t2 = gPWM.gPWMPrdApply;
        }
        else
        {
            divLen = gPWM.gPWMPrdApply/( *f32t1 + *f32t2 );
            *f32t1 *= divLen;
            *f32t2 *= divLen;
        }
    }
}

/*************************************************************
    发送P波
*************************************************************/
void SendPWM()
{
    //-------------------------- PWM波接口确认。
//    EPwm1Regs.TBPRD = gPWM.gPWMPrdApply;
//    EPwm2Regs.TBPRD = gPWM.gPWMPrdApply;
//    EPwm3Regs.TBPRD = gPWM.gPWMPrdApply;
//
//    EPwm1Regs.CMPA.half.CMPA = gPWM.U;
//    if( IS_NEG_SEQ_CTRL_EN )
//    {
//        EPwm2Regs.CMPA.half.CMPA = gPWM.W;
//        EPwm3Regs.CMPA.half.CMPA = gPWM.V;
//    }
//    else
//    {
//        EPwm2Regs.CMPA.half.CMPA = gPWM.V;
//        EPwm3Regs.CMPA.half.CMPA = gPWM.W;
//    }

    if( 1 == g_PwmEnable && 0 == g_PwmAlreadyOn )
    {
        EnableDrive();

        g_PwmEnable = 0;
    }

#if (PWMSENDTEST == 1)
    	TIM1->ARR = 9000;
    	TIM8->ARR = 18000;
    	gPWM.U = gPWM.gPWMPrdApply - 4500;
    	gPWM.V = gPWM.gPWMPrdApply - 4500;
    	gPWM.W = gPWM.gPWMPrdApply - 4500;
    	TIM_SetChannalCompare(TIM1,1,gPWM.U);
    	if( IS_NEG_SEQ_CTRL_EN )
    	{
    		TIM_SetChannalCompare(TIM1,2,gPWM.W);
    		TIM_SetChannalCompare(TIM1,3,gPWM.V);
    	}
    	else
    	{
    		TIM_SetChannalCompare(TIM1,2,gPWM.V);
    		TIM_SetChannalCompare(TIM1,3,gPWM.W);
    	}
    	TIM1->CCER = TIM1->CCER|0x0555;
    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
        HAL_GPIO_WritePin(M1_PWM_EN_GPIO_Port,M1_PWM_EN_Pin,0);
#else

	TIM1->ARR = gPWM.gPWMPrdApply;
	TIM8->ARR = (gPWM.gPWMPrdApply*2);
	TIM_SetChannalCompare(TIM1,1,(gPWM.gPWMPrdApply - gPWM.U));
	if( IS_NEG_SEQ_CTRL_EN )
	{
		TIM_SetChannalCompare(TIM1,2,(gPWM.gPWMPrdApply - gPWM.W));
		TIM_SetChannalCompare(TIM1,3,(gPWM.gPWMPrdApply - gPWM.V));
	}
	else
	{
		TIM_SetChannalCompare(TIM1,2,(gPWM.gPWMPrdApply - gPWM.V));
		TIM_SetChannalCompare(TIM1,3,(gPWM.gPWMPrdApply - gPWM.W));
	}
#endif

}

void EnableDrive(void)
{
	PWM_enable();
    g_PwmAlreadyOn = 1;
    HAL_GPIO_WritePin(M1_PWM_EN_GPIO_Port,M1_PWM_EN_Pin,0);
}

void DisableDrive(void)
{
#if (PWMSENDTEST == 0)
	PWM_disable();
    g_PwmAlreadyOn = 0;
    HAL_GPIO_WritePin(M1_PWM_EN_GPIO_Port,M1_PWM_EN_Pin,1);
#endif

}
