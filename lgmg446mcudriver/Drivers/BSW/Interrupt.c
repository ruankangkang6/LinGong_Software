#include "main.h"
#include "Global_Variables.h"
#include "Interrupt.h"
#include "f_public.h"
#include "MotorCalibration.h"

uint16_t gu16ADCounter = 0;

void ADC_ISR(void)
{
    gu16ADCounter++;

    GetAdcResult();                           //ADC采样转换

	if( 1 == gu16ADCounter )
	{
		ImmSoftWareErrorDeal();               //软件过流检测

		RDC_SAMPLE_H;

		gRotorTrans.SPIStartFlag = 1;
		RotorTransCalVel();                  //速度计算

		if (  (0 == IS_CALIBRATED)
		   && (g_RunStatus == STATUS_RUNNING)
		   && (g_McuModeRe != ACTDISCHARGE_CMD)
		   )
		{
//			PMFluxWeakDeal();
			PMFluxWeakDealEx();              //弱磁输出计算
		}


		CalcADOffset();                      //零漂检验

		PMSetFwPara1();                      //Us传递

		CalcLineCurr();                      //相电流计算

        SelectAcrPar();                      //电流环PI参数切换

		CalcCsrDecoupleVal();                //UdUq给定补偿计算
	}
	else if( gu16ADCounter >= 2 )
	{
        RotorTransReadPos();    // 读位置
        RDC_SAMPLE_L;
        RotorTransSamplePos();  // 位置信息锁存, record time

        gPWM.gPWMPrdApply = gPWM.gPWMPrd;

        if( g_RunStatus != STATUS_PARAMEST )  //STATUS_GET_PAR_PMSM != g_PMEstCmdgMainStatus.RunStep
        {
            CalOutputPhase();               //电角度计算

            TransformCurrent();             //电流变换

            VCCsrControlEx();               //UdUq给定计算
//            CalRatioFromVot(); //无用函数

            OutPutPWMVF();                  //PWM信号计算
        }
        else
        {
            RunParamEstInIsr();             //参数辨识PWM
        }

        gu16ADCounter = 0;
    }
	else
	{
		;
	}
}

void Main2msMotor(void)
{
	static uint16_t S_time2msCnt = 0;
	static uint16_t S_time5msCnt = 0;
	static uint16_t S_time10msCnt = 0;
	static uint16_t S_time20msCnt = 0;
	static uint16_t S_time100msCnt = 99;

    ADCProcess();                  // ADC矫正处理

	ParGetFromFunction();          // 获取控制板储存的参数

	ParameterChange();             // 一些参数计算，运行前的准备

	CalcActTorque();               //计算反馈扭矩

	RunningDeal();                 //Running状态处理

	S_time2msCnt++;
	if(S_time2msCnt >= 2)
	{
		S_time2msCnt = 0;
		RunCasePMEst();                //参数辨识
	}

	TemperatureCheck();            //温度检测

	StallProtect();                //堵转保护

	SoftWareErrorDeal();           //软件故障

	if( IS_CALIBRATED )
	{
		CalcMaxTorqueRange(gRotorSpeed.SpeedApply);  //查当前转速下电动和回馈的最大扭矩
	}

	S_time100msCnt++;
	if(S_time100msCnt >= 100)
	{
		S_time100msCnt = 0;
		VehicleDataDeal();       //仪表数据初始化
	}

	GetVehicleInformation();    //叉车控制命令处理

	ForkLiftHourCnt();           //叉车小时计

	RunStModeDeal(gDmdCanRe);              //运行状态判断
}
