#ifndef FLUXWEAK_H
#define FLUXWEAK_H

//---------extern "C"是C++中的修饰的语句，一般在C++使用C语言中的文件时使用（表示是按照C语言的方式进行编译的）。----------
//-------- __cplusplus是C++默认的宏定义，C中没有，刚好可以以此预编译解决C和C++混用头文件问题（C没有extern "C"，会报错）。--------
//#ifdef __cplusplus
//extern "C"{
//#endif


typedef struct PM_CSR_DEF
{	
	float	UAmp;
}PM_CSR;

typedef struct MTPA_DEF
{
    int16_t i16IqSetting;      //标定Iq电流设置%(上位机输入)
    float Iq;              //标定Iq电流实际值（i16IqSetting*0.001f*sqrt2*gMotorInfo.Current)

    //以下实际无作用？
	int16_t i16CurrSetting;    //标定Is电流设置%(上位机输入)
	float Curr;            //标定Is电流实际值(i16CurrSetting*0.01f*sqrt2*gMotorInfo.Current)
	uint16_t u16ThetaSetting;  //转矩角设置(上位机输入)
	float Theta;           //转矩角实际值 ，角度范围90-180 (u16ThetaSetting*1.745329252e-3 + Value_Pi2)
	float CosVal;          //转矩角cos值
	float SinVal;          //转矩角sin值
}MTPA;

typedef struct PM_FW_DEF
{	
	float IdMax1;            //最大弱磁电流限制
	float VoutAmpLpf;
	float IdForTorq;         //MTPA弱磁控制Id计算值
	float IdForTorqFilter;   //MTPA弱磁控制Id计算值滤波
	float AdId;
	float IqLpf;
	PIReg FwPI;
	uint16_t StopIncIqByIdRef;
	uint16_t StopIncIqByVout;
	uint16_t StopIncIqByVoutCnt;
}PM_FW;

typedef struct PM_FW_OUT_DEF
{	
	float  IdSet;		// 电流环D轴给定值
	float  IqSet;		// 电流环Q轴给定值
	float  UqComp;	// Q轴补偿电压
	float  UdComp;	// D轴补偿电压
}PM_FW_OUT;

extern PM_FW		gPMFwCtrl;
extern PM_FW_OUT    gPMFwOut;
extern PM_CSR       gCsrPI;
extern MTPA			gMTPA;
extern uint16_t gu16MaxFwIdSetting;
extern float gCsrUAmp;

extern void PMFwInit(void);
extern void PMMaxTorqCtrl( float iqSet );
extern void PMFluxWeakDealEx( void );
extern void PMSetFwPara1(void);


//#ifdef __cplusplus
//}
//#endif

#endif  
