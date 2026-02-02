/*
 * MotorAdc.c
 *
 *  Created on: 2024年12月17日
 *      Author: pc
 */
#include "f_public.h"
#include "main.h"

#include "Global_Variables.h"

//#pragma CODE_SECTION(ImmSoftWareErrorDeal,  "ramfuncs")
//#pragma CODE_SECTION(CalcADOffset,      "ramfuncs")

uint16_t gADCResetTime = 0;

AD_CONVERT g_ADConvert = {{0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}};
AD_CONVERT g_ADConvertSum = {{0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0}};
AD_SAMPLE  g_SampleRealValue = {0, 0, 0, 0, 0, 0, 0, 0};
AD_SAMPLE  g_SampleDValue = {0, 0, 0, 0, 0, 0, 0, 0};
AD_SAMPLE  g_FirstSampleValue = {0, 0, 0, 0, 0, 0, 0, 0};
AD_OFFSET_STRUCT  gADOffsetInfo = {0, 0, 0};//检测零漂使用的结构
UDC_STRUCT gUDC = {0, 0, 0};

float ACC_AD = 0;
float AD_Ctrl_Vin = 0;
float Ctrl_Vin = 0;

//void InitAdc(void);//ADC外设初始化
void GetAdcResult(void);
void ADCProcess(void);

uint16_t ADCresult_iw[4]={0,0,0,0};
uint16_t ADCresult_iv[4]={0,0,0,0};
uint16_t ADCsum_iw=0,ADCsum_iv=0;
uint8_t ADC_index=0;

uint16_t gAdcResult1_iw = 0;
uint16_t gAdcResult1_iv = 0;
uint32_t AdcRandDat;
/*************************************************************
  ADC采样
*************************************************************/
#define AD_DEFAULT_AC_OFFSET 2048
void GetAdcResult(void)
{
	static float s_IGBTTempAD = 0;
	static float s_MotorTempAD = 0;
	if(ADC_index<1)
	{
		ADC_index++;
	}
	else
	{
		ADC_index=0;
	}
	AdcRandDat = AdcRandDat << 2;
	AdcRandDat |= uhADCxConvertedValue[8];

	ADCsum_iw+=uhADCxConvertedValue[8];								//M1 W相电流
	ADCsum_iw-=ADCresult_iw[ADC_index];
	ADCresult_iw[ADC_index] = uhADCxConvertedValue[8];
	u16AdcResult0_iw = ADCsum_iw>>1;
	ADCsum_iv+=uhADCxConvertedValue[9];								//M1 W相电流
	ADCsum_iv-=ADCresult_iv[ADC_index];
	ADCresult_iv[ADC_index] = uhADCxConvertedValue[9];
	u16AdcResult1_iv = ADCsum_iv>>1;

//	u16AdcResult0_iw = (uhADCxConvertedValue[8]&0x0FFF);
//	u16AdcResult1_iv = (uhADCxConvertedValue[9]&0x0FFF);

	u16AdcResult2_tempMotor = (uhADCxConvertedValue[2]&0x0FFF);		//M1 电机温度
	u16AdcResult3_tempIGBT = (uhADCxConvertedValue[3]&0x0FFF);		//M1 功率板温度
	u16AdcResult4_KSI = (uhADCxConvertedValue[4]&0x0FFF);			//KSI电压采样
	u16AdcResult5_Accpedal = (uhADCxConvertedValue[5]&0x0FFF);		//加速踏板
	u16AdcResult6_SteeringAngle = (uhADCxConvertedValue[1]&0x0FFF);	//转角电位计
	u16AdcResult7_MainRelay_Cur = (uhADCxConvertedValue[7]&0x0FFF);	//主接触器电流采样
	u16AdcResult8_Drv3_Cur = (uhADCxConvertedValue[0]&0x0FFF);		//
	u16AdcResult9_VDC = (uhADCxConvertedValue[6]&0x0FFF);			//DC_LINK电压采样
	ACC_AD = 0.5*(ACC_AD +(float)u16AdcResult5_Accpedal);
	if(ACC_enable == 0)
	{
		ACC_AD = 0;
	}

	if( IS_NEG_SEQ_CTRL_EN )
	{
		g_SampleDValue.f32Ib	=	(float)u16AdcResult0_iw*1.1f  - AD_DEFAULT_AC_OFFSET;
		g_SampleDValue.f32Ic	=	(float)u16AdcResult1_iv*1.1f  - AD_DEFAULT_AC_OFFSET;
	}
	else
	{
		g_SampleDValue.f32Ib	=	(float)u16AdcResult1_iv*1.1f  - AD_DEFAULT_AC_OFFSET;
		g_SampleDValue.f32Ic	=	(float)u16AdcResult0_iw*1.1f  - AD_DEFAULT_AC_OFFSET;
	}

    g_SampleDValue.f32Vdc   =   (float)u16AdcResult9_VDC/*adcCResult2_vdc*/;//(float)ADC_RESULT_V_DC;
    g_SampleDValue.f32Temp  =   (float)u16AdcResult3_tempIGBT;///*adcAResult3_tempIGBT*/;//(float)ADC_RESULT_TEMP;     //IGBT temp

    g_SampleDValue.f32AI4   =   (float)u16AdcResult2_tempMotor;//*1.1f;//ADC_AI4;     //motor temp
    g_SampleDValue.f32AI5   =   0;//ADC_AI5;     //motor temp1


    g_SampleRealValue.f32Ia     =   g_SampleDValue.f32Ia   * g_ADConvert.gain.f32Ia   - g_ADConvert.offset.f32Ia;
    g_SampleRealValue.f32Ib     =   g_SampleDValue.f32Ib   * g_ADConvert.gain.f32Ib   - g_ADConvert.offset.f32Ib;
    g_SampleRealValue.f32Ic     =   g_SampleDValue.f32Ic   * g_ADConvert.gain.f32Ic   - g_ADConvert.offset.f32Ic;

	g_SampleRealValue.f32Ia = - g_SampleRealValue.f32Ic - g_SampleRealValue.f32Ib;

    gCurSamp.U = g_SampleRealValue.f32Ia;
    gCurSamp.V = g_SampleRealValue.f32Ib;
    gCurSamp.W = g_SampleRealValue.f32Ic;

//	ImmSoftWareErrorDeal();               //软件过流检测


    if( 1 == gu16ADCounter )
    {
        g_FirstSampleValue.f32Ia    =   g_SampleDValue.f32Ia   * g_ADConvert.gain.f32Ia   - g_ADConvert.offset.f32Ia;
        g_FirstSampleValue.f32Ib    =   g_SampleDValue.f32Ib   * g_ADConvert.gain.f32Ib   - g_ADConvert.offset.f32Ib;
        g_FirstSampleValue.f32Ic    =   g_SampleDValue.f32Ic   * g_ADConvert.gain.f32Ic   - g_ADConvert.offset.f32Ic;
        g_FirstSampleValue.f32Vdc   =   g_SampleDValue.f32Vdc  * g_ADConvert.gain.f32Vdc;
        g_FirstSampleValue.f32Temp  =   g_SampleDValue.f32Temp * g_ADConvert.gain.f32Temp - g_ADConvert.offset.f32Temp;
        g_FirstSampleValue.f32AI4   =   g_SampleDValue.f32AI4 * g_ADConvert.gain.f32AI4 - g_ADConvert.offset.f32AI4;

    }
    else
    {
//        g_SampleRealValue.f32Ia     =   g_SampleDValue.f32Ia   * g_ADConvert.gain.f32Ia   - g_ADConvert.offset.f32Ia;
//        g_SampleRealValue.f32Ib     =   g_SampleDValue.f32Ib   * g_ADConvert.gain.f32Ib   - g_ADConvert.offset.f32Ib;
//        g_SampleRealValue.f32Ic     =   g_SampleDValue.f32Ic   * g_ADConvert.gain.f32Ic   - g_ADConvert.offset.f32Ic;
        g_SampleRealValue.f32Vdc    =   g_SampleDValue.f32Vdc  * g_ADConvert.gain.f32Vdc;

        g_SampleRealValue.f32Temp   =   g_SampleDValue.f32Temp * g_ADConvert.gain.f32Temp - g_ADConvert.offset.f32Temp;
        g_SampleRealValue.f32AI4    =   g_SampleDValue.f32AI4 * g_ADConvert.gain.f32AI4 - g_ADConvert.offset.f32AI4;


//        g_SampleRealValue.f32Ia     =   0.5f*( g_SampleRealValue.f32Ia   +  g_FirstSampleValue.f32Ia );
//        g_SampleRealValue.f32Ib     =   0.5f*( g_SampleRealValue.f32Ib   +  g_FirstSampleValue.f32Ib );
//        g_SampleRealValue.f32Ic     =   0.5f*( g_SampleRealValue.f32Ic   +  g_FirstSampleValue.f32Ic );
        g_SampleRealValue.f32Vdc    =   0.5f*( g_SampleRealValue.f32Vdc  +  g_FirstSampleValue.f32Vdc );

        g_SampleRealValue.f32Temp   =   0.5f*( g_SampleRealValue.f32Temp +  g_FirstSampleValue.f32Temp );
        g_SampleRealValue.f32AI4    =   0.5f*( g_SampleRealValue.f32AI4  +  g_FirstSampleValue.f32AI4 );


//		g_SampleRealValue.f32Ia = - g_SampleRealValue.f32Ic - g_SampleRealValue.f32Ib;
//
//        gCurSamp.U = g_SampleRealValue.f32Ia;
//        gCurSamp.V = g_SampleRealValue.f32Ib;
//        gCurSamp.W = g_SampleRealValue.f32Ic;
//
//		ImmSoftWareErrorDeal();               //软件过流检测

        gUDC.uDC            =   g_SampleRealValue.f32Vdc;
        gUDC.uDCFilter      =   0.875f*gUDC.uDCFilter + ( 1 - 0.875f )*gUDC.uDC;
        gUDC.uDCBigFilter   =   0.985f*gUDC.uDCBigFilter + ( 1 - 0.985f )*gUDC.uDC;

        if( gADCResetTime < 100 )
        {
        	s_IGBTTempAD       = g_SampleRealValue.f32Temp;
        	s_MotorTempAD  = g_SampleRealValue.f32AI4;
        }
        else
        {
        	s_IGBTTempAD       = ( 1 - 0.05f )*s_IGBTTempAD + 0.05f*g_SampleRealValue.f32Temp;
        	s_MotorTempAD  = ( 1 - 0.01f )*s_MotorTempAD + 0.01f*g_SampleRealValue.f32AI4;
        }

        gTemperature.TempAD = s_IGBTTempAD;
        gTemperature.MotorTempAD = s_MotorTempAD;
    }

	AD_Ctrl_Vin = 0.5*(AD_Ctrl_Vin +(float)u16AdcResult4_KSI);
	Ctrl_Vin = AD_Ctrl_Vin * 0.0496f;

//	SenLowFaultCheck();
}

/*************************************************************
    ADC校正处理
*************************************************************/
void ADCProcess(void)
{
    gADCResetTime = gADCResetTime + 10;
    gADCResetTime = ( gADCResetTime > 1000 ) ? 1000 : gADCResetTime;

}
