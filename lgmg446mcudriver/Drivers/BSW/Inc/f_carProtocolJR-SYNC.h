#ifndef __F_CAR_PROTOCOL_JR_H__
#define __F_CAR_PROTOCOL_JR_H__


#define PROT_FLAG	0

#include "main.h"
#include "f_mcuctrl.h"

#define MOTOR_CLOSE_MODE			0x00000   	//0
#define MOTOR_ERR_MODE				0x00001		//1/
#define MOTOR_BRAKE_MODE			0x01010		//10
#define MOTOR_FORWARD_MODE			0x10010		//18
#define MOTOR_BACKWARD_MODE			0x11010		//26
#define MOTOR_SPEED_MODE			0x00011		//3/
#define MOTOR_ZERO_MODE				0x00010		//2/

#define BALT_MODE_OPENHV				0x00
#define BALT_MODE_PRECHAR				0x01
#define BALT_MODE_NORMALH				0x10
#define BALT_MODE_VALID					0x11

#define SOFTWARE_VERSION     0x1E         // 软件版本号

#define PROJECT_VERSION     0x0         // 项目版本

#define FAULT_CODE_NMB	33

typedef enum ERRCODE_JR_STATUS_ENUM
{	
	E_ENORMAL = 0,
	E_IGBTFAULT = 101,				//101/ģ�����
	E_SELFCHECK,					//102/�Լ����
	E_CLOWVOLTSTS,					//103/������Ƿѹ����
	E_MOVTEMPSTS,					//104 �������
	E_IGBTOVCURSTS,					//105 ���������
	E_IGBTOVTEMPSTS,				//106 IGBT����
	E_MOVSPDSTS,					//107���ٹ���
	E_COVVOLTSTS,					//108/ĸ�߹�ѹ
	E_CTRANFAULTSTS,				//109 ����������
	E_CANFAULTSTS,					//110 can����

	E_RSVD0 = 121,					//E_IGBTOVCURSTS = 121,			//121 ����
	E_RSVD1,						//E_MOVTEMPSTS,					//122 ����
	E_RSVD2,						//E_IGBTOVTEMPSTS,				//123 ����
	E_RSVD3,						//E_MOVSPDSTS,					//124 ����
	E_RSVD4,						//E_COVVOLTSTS,					//125 ����
	E_FANSTS,						//E_COVVOLTSTS,					//126 ����
	
	E_IGBTTEMPWARNING = 141,		//141  ����
	E_MTEMPWARNING,					//142 ����
	E_CLOWVOLTWARNING,				//143 ����
	E_COVVOLTWARNING,				//144 ����
	E_TEMPSENWARNING				//145 ����		
}ERRCODE_JR_STATUS_ATTRIBUTE;

typedef struct CAR_JR_VCU2MCU_STRUCT  	//0x0CFF08EF
{ 		
	struct	CAR_JR_VCU2MCU_MXPARAM_STRUCT	
	{	
	   union CAR_JR_VCU2MCU_MXPARAM0
	   {
		   uint16_t  all;
		   struct CAR_JR_VCU2MCU_MXPARAM0_BITS	   
		   {	 
				uint16_t  VMotEn		   :1;
	   			uint16_t  VReqMode	   :4;
				uint16_t	rsvd0 		   :3;
   
			    uint16_t  VReqTroqL	   :8;
		   }bit;
	   }m_vcu2mcu_0;

	   union CAR_JR_VCU2MCU_MXPARAM1
	   {
		   uint16_t  all;
		   struct CAR_JR_VCU2MCU_MXPARAM1_BITS	   
		   {	
			   uint16_t   VReqTroqH	   :8;
			   uint16_t	VReqSpdL	   :8;
		   }bit;
	   }m_vcu2mcu_1;

		union CAR_JR_VCU2MCU_MXPARAM2
   		{
		   uint16_t  all;
		   struct CAR_JR_VCU2MCU_MXPARAM2_BITS	   
		   {	
			   uint16_t	VReqSpdH	   :8;
			   
			   uint16_t	DearSignal 	   :4;
			   uint16_t	BrakeSignal    :4;
		   }bit;
   		}m_vcu2mcu_2;

		union CAR_JR_VCU2MCU_MXPARAM3
   		{
		   uint16_t  all;
		   struct CAR_JR_VCU2MCU_MXPARAM3_BITS	   
		   {	      
			   uint16_t FanDuty 		   :8;
			   
			   uint16_t rsvd1 		   :4;
			   uint16_t LifeCounter1 	   :4;
		   }bit;
   		}m_vcu2mcu_3;
	}vcu2mcu_mxparam;

	struct	CAR_JR_VCU2MCU_MYPARAM_STRUCT		//0x0CFF18EF
	{	
	   union CAR_JR_VCU2MCU_MYPARAM0
	   {
		   uint16_t  all;
		   struct CAR_JR_VCU2MCU_MYPARAM0_BITS	   
		   {	   
			   uint16_t VMaxSpdLimitL		:8;
			   uint16_t VMaxSpdLimitH		:8;
		   }bit;
	   }m_vcu2mcu_0;

	   union CAR_JR_VCU2MCU_MYPARAM1
	   {
		   uint16_t  all;
		   struct CAR_JR_VCU2MCU_MYPARAM1_BITS	   
		   {	   
			   uint16_t VMinSpdLimitL		:8;
			   uint16_t VMinSpdLimitH		:8;
		   }bit;
	   }m_vcu2mcu_1;

		union CAR_JR_VCU2MCU_MYPARAM2
   		{
		   uint16_t  all;
		   struct CAR_JR_VCU2MCU_MYPARAM2_BITS	   
		   {	
			   uint16_t PosTorqGradientL		:8;
			   uint16_t PosTorqGradientH		:8;
		   }bit;
   		}m_vcu2mcu_2;	

		union CAR_JR_VCU2MCU_MYPARAM3
   		{
		   uint16_t  all;
		   struct CAR_JR_VCU2MCU_MYPARAM3_BITS	   
		   {	   
			   uint16_t rsvd0 				:8;
			   
			   uint16_t rsvd1 				:4;
			   uint16_t LifeCounter2 	   		:4;
		   }bit;
   		}m_vcu2mcu_3;
	}vcu2mcu_myparam;

	struct	CAR_JR_VCU2MCU_MZPARAM_STRUCT		//0x0CFF28EF
	{	
	   union CAR_JR_VCU2MCU_MZPARAM0
	   {
		   uint16_t  all;
		   struct CAR_JR_VCU2MCU_MZPARAM0_BITS	   
		   {	   
			   uint16_t VMaxTroqLimitL		:8;
			   uint16_t VMaxTroqLimitH		:8;
		   }bit;
	   }m_vcu2mcu_0;

	   union CAR_JR_VCU2MCU_MZPARAM1
	   {
		   uint16_t  all;
		   struct CAR_JR_VCU2MCU_MZPARAM1_BITS	   
		   {	   
			   uint16_t VMinTroqLimitL		:8;
			   uint16_t VMinTroqLimitH		:8;
		   }bit;
	   }m_vcu2mcu_1;

		union CAR_JR_VCU2MCU_MZPARAM2
   		{
		   uint16_t  all;
		   struct CAR_JR_VCU2MCU_MZPARAM2_BITS	   
		   {	
			   uint16_t NegTorqGradientL		:8;
			   uint16_t NegTorqGradientH		:8;
		   }bit;
   		}m_vcu2mcu_2;	

		union CAR_JR_VCU2MCU_MZPARAM3
   		{
		   uint16_t  all;
		   struct CAR_JR_VCU2MCU_MZPARAM3_BITS	   
		   {	   
			   uint16_t rsvd0 				:8;
			   
			   uint16_t rsvd1 				:4;
			   uint16_t LifeCounter3 	   		:4;
		   }bit;
   		}m_vcu2mcu_3;
	}vcu2mcu_mzparam;
} VCU2MCU_FRAME_JR;


typedef struct PARAM_MCU2VCU_FRAME_JR          //0x0CFF0008              
{	
	struct	CAR_JR_MCU2VCU_MXPARAM_STRUCT		
	{
		union CAR_JR_MCU2VCU_MXPARAM0
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MXPARAM0_BITS		
			{			
				uint16_t  CCurrentHvH			:8;
				uint16_t  CCurrentHvL			:8;
			}bit;
		}m_mcu2vcu_0;
		
		union CAR_JR_MCU2VCU_MXPARAM1
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MXPARAM1_BITS		
			{		
				uint16_t  COutputTroqH		:8;
				uint16_t  COutputTroqL		:8;
			}bit;
		}m_mcu2vcu_1;
		
		union CAR_JR_MCU2VCU_MXPARAM2
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MXPARAM2_BITS		
			{		
				uint16_t  CSpdH				:8;
				uint16_t  CSpdL				:8;
			}bit;
		}m_mcu2vcu_2;

		union CAR_JR_MCU2VCU_MXPARAM3
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MXPARAM3_BITS		
			{	
				uint16_t	rsvd0 					:4;
				uint16_t	life					:4;
				
				uint16_t	CState					:4;
				uint16_t	rsvd1 					:4;
			}bit;
		}m_mcu2vcu_3;						
	}mcu2vcu_mxparam;
	
	struct	CAR_JR_MCU2VCU_MYPARAM_STRUCT	//0x0CFF0108	
	{
		union CAR_JR_MCU2VCU_MYPARAM0
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MYPARAM0_BITS 	
			{
   				int16_t       CMaxTorqH			:8;   	// 0:7
				int16_t		CMaxTorqL 			:8;
			}bit;
		}m_mcu2vcu_0;
		
		union CAR_JR_MCU2VCU_MYPARAM1
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MYPARAM1_BITS 	
			{
   				int16_t       CMinTorqH			:8;   	// 0:7
				int16_t		CMinTorqL 			:8; 	// 8:15
			}bit;
		}m_mcu2vcu_1;

		union CAR_JR_MCU2VCU_MYPARAM2
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MYPARAM2_BITS 	
			{
				uint16_t	CCTemp				:8;
				uint16_t	CMTemp				:8;
			}bit;
		}m_mcu2vcu_2;

		union CAR_JR_MCU2VCU_MYPARAM3                    
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MYPARAM3_BITS		
			{	 
				uint16_t	rsvd 				:4;
				uint16_t	life				:4;
				
				uint16_t	CETemp				:8;
			}bit;
		}m_mcu2vcu_3;					
	}mcu2vcu_myparam;


	struct	CAR_JR_MCU2VCU_MZPARAM_STRUCT		  //0x0CFF0208
	{
		union CAR_JR_MCU2VCU_MZPARAM0
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MZPARAM0_BITS		
			{	
    			uint16_t  	CVoltageHvH		:8;
				uint16_t		CVoltageHvL		:8;
			}bit;
		}m_mcu2vcu_0;
		
		union CAR_JR_MCU2VCU_MZPARAM1
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MZPARAM1_BITS		
			{	
				uint16_t		CFaultLevel		:4;
				uint16_t		CIgbtEnable		:1; 	//pwmon
				uint16_t		CShutDowmReq	:1; 	//MCU��������������ʱ���͵������µ�ָ��
				uint16_t		rsvd			:2;
				
    			uint16_t  	CVoltageLv		:8;
		 
			}bit;
		}m_mcu2vcu_1;
		
		union CAR_JR_MCU2VCU_MZPARAM2
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MZPARAM2_BITS		
			{		
				uint16_t	CFaultCodeH			:8;
				uint16_t	CFaultCodeL			:8;
			}bit;
		}m_mcu2vcu_2;

		union CAR_JR_MCU2VCU_MZPARAM3
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MZPARAM3_BITS		
			{	  
				uint16_t	CPhaseCurH			:8;
				uint16_t	CPhaseCurL			:8;
			}bit;
		}m_mcu2vcu_3;
	}mcu2vcu_mzparam;
	
	struct	CAR_JR_MCU2VCU_MUPARAM_STRUCT		//0x0CFF0308
	{
		union CAR_JR_MCU2VCU_MUPARAM0
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MUPARAM0_BITS		
			{	
				uint16_t	rsvd					:3;
				uint16_t	CHwVer					:5;
				
				uint16_t	CDevType				:6;
				uint16_t	CVehiType				:2;
			}bit;
		}m_mcu2vcu_0;
		
		union CAR_JR_MCU2VCU_MUPARAM1
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MUPARAM1_BITS		
			{	
				uint16_t	CMaxPower				:8;
				uint16_t	CSwVer					:8;
			}bit;
		}m_mcu2vcu_1;
		
		union CAR_JR_MCU2VCU_MUPARAM2
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MUPARAM2_BITS		
			{	
				uint16_t	CSwMonth				:8;
				uint16_t	CSwYear					:8;
			}bit;
		}m_mcu2vcu_2;

		union CAR_JR_MCU2VCU_MUPARAM3
		{
			uint16_t	all;
			struct CAR_JR_MCU2VCU_MUPARAM3_BITS		
			{	 
				uint16_t	rsvd					:8;
				uint16_t	CSwDate					:8;
			}bit;
		}m_mcu2vcu_3;
	}mcu2vcu_muparam;
}MCU2VCU_FRAME_JR;

typedef struct
{
	uint8_t Bind_GPS_ON:       1;
	uint8_t UnBind_GPS_OFF:     1;
	uint8_t Vec_Lock_LV1:   1;
	uint8_t Vec_Lock_LV2:   1;
	uint8_t Vec_UnLock:     1;
	uint8_t Bind_Advanced:       1;
	uint8_t UnBind_ALL:       1;
	uint8_t rsvd:           1;
	uint8_t  Tbox_GPSID1;
	uint8_t  Tbox_GPSID2;
	uint8_t  Tbox_GPSID3;
	uint8_t  Tbox_GPSID4;
	uint8_t  Tbox_GPSID5;
	uint8_t  Tbox_GPSID6;
	uint8_t  Tbox_GPSID7;
}IL_Tbox_DRV_MCU_0x6B68A1_R;
typedef union
{
	uint8_t byte[8];
	IL_Tbox_DRV_MCU_0x6B68A1_R dat;
}IL_Tbox_DRV_MCU_0x6B68A1_U;
extern IL_Tbox_DRV_MCU_0x6B68A1_U uIlRxTboxDrvMcu0x6B68A1;

typedef struct
{
	uint8_t  BYTE0;
	uint8_t  Tbox_GPSID1;
	uint8_t  Tbox_GPSID2;
	uint8_t  Tbox_GPSID3;
	uint8_t  Tbox_GPSID4;
	uint8_t  Tbox_GPSID5;
	uint8_t  Tbox_GPSID6;
	uint8_t  Tbox_GPSID7;
}IL_Tbox_DRV_MCU_0x6B68A2_R;
typedef union
{
	uint8_t byte[8];
	IL_Tbox_DRV_MCU_0x6B68A2_R dat;
}IL_Tbox_DRV_MCU_0x6B68A2_U;
extern IL_Tbox_DRV_MCU_0x6B68A2_U uIlRxTboxDrvMcu0x6B68A2;

void InitValueCommJR(void);
void CarJRFrame0InfoUpdate(void);
void CarJRFrame1InfoUpdate(void);
void CarJRFrame2InfoUpdate(void);
void CarJRFrame3InfoUpdate(void);
void CarJRErrInfoUpdate(void);
void CarJRFrameMeterInfoUpdate(void);
void CarJRFrameELECQUATInfoUpdate(void);
void CarJRFrameVEHSPDInfoUpdate(void);
void CarJRFrameWORKHURInfoUpdate(void);
void CarJRFrameVEHFLTInfoUpdate(void);
void CarJRFrameAGLInfoUpdate(void);
void CarJRFrameSPDInfoUpdate(void);
void CarJRFrameMETER1InfoUpdate(void);
void CarJRFrameMETER2InfoUpdate(void);
void CarJRFrameTBOXInfoUpdate(void);
void CarJRDriverInfo1Update(void);
void CarJRDriverInfo2Update(void);
void CarJRDriverInfo3Update(void);
void CarJRDriverInfo4Update(void);
void CarJRDriverTboxUpdate(void);


void CarJRRcvFramePumpInfo1Deal(void);
void CarJRRcvFramePumpInfo2Deal(void);
void CarJRRcvFramePump2TboxDeal(void);
void CarJRRcvFrameCalibrationCmd(void);
void CarJRRcvUpdateSoftware(void);
void CarJRRcvFrame1Deal(void);
void CarJRRcvFrame2Deal(void);
void CarJRRcvFrame3Deal(void);
void CarJRRcvFrameBMSDeal(void);
void CarJRRcvFrameBMS0Deal(void);
void CarJRRcvFrameBMS1Deal(void);
void CarJRRcvFrameBMS2Deal(void);
void CarJRRcvFrameBMS3Deal(void);
void CarJRRcvFrameBMS4Deal(void);
void CarJRRcvFrameBMS5Deal(void);
void CarJRRcvFrameMETERDeal(void);
void CarJRRcvFrameMETER2Deal(void);
void CarJRRcvFrameMETER3Deal(void);

//void CarJRRcvFrameTboxDeal(void);
void CarJRRcvFrameTboxA1Deal(void);
void CarJRRcvFrameTboxA2Deal(void);

float CalculateTorqueNeedToMcuJR( float f, float t );
float CalculateTorqueNeedToVcuJR( float f, float t );


extern uint16_t				g_u16dirsts_jr;
extern MCU2VCU_FRAME_JR  	g_mcu2vcu_jr;
extern VCU2MCU_FRAME_JR 	g_vcu2mcu_jr;
extern float 				g_f32torqaim_jr;

#endif  //__F_CARPROTOCOL_JR_H__




