/*
 * MotorInclude.h
 *
 *  Created on: 2024年12月4日
 *      Author: pc
 */

#ifndef SOURCE_MOTOR_INCLUDE_MOTORINCLUDE_H_
#define SOURCE_MOTOR_INCLUDE_MOTORINCLUDE_H_

#include "MotorAdc.h"
//#include "MotorCalibration.h"
#include "MotorVC.h"
#include "MotorPWM.h"
#include "MotorRotorPos.h"
#include "MotorCarrier.h"

#define     MOTOR_SEL           75           //软件版本

#define IS_POSITIVE_ROTATE_WITH_SPEED_N	 0   //0：程序正转速车辆前进，1：程序负转速车辆前进


#define     HALL_200            0            //200A 霍尔
#define     HALL_300            1            //300A 霍尔
#define     HALL_500            2            //500A 霍尔
#define     HALL_800            3            //800A 霍尔
#define     HALL_600            4            //600A 霍尔
#define		HALL_600P			5		     //600AP

#define     HALL                HALL_800
#define     HALL_NUM_3          0            //0：两相霍尔采样，1：三相霍尔采样




//标定模式下，如何识别使能
#define     CODE_MODE_CALIBRATED_ON_ROAD_RUNNING           0    //整车程序
#define     CODE_MODE_CALIBRATED_ON_TEST_BENCH_RUNNING     1    //测试程序
#define     CODE_MODE_UNCALIBRATED_ON_ROAD_RUNNING         2    //整车程序(非查表)
#define     CODE_MODE_UNCALIBRATED                         3    //标定程序

#define     CODE_MODE   CODE_MODE_CALIBRATED_ON_ROAD_RUNNING

#if ( CODE_MODE == CODE_MODE_CALIBRATED_ON_ROAD_RUNNING )
#define IS_CALIBRATED   1                                       //已标定
#elif ( CODE_MODE == CODE_MODE_CALIBRATED_ON_TEST_BENCH_RUNNING )
#define IS_CALIBRATED   1                                       //已标定
#elif ( CODE_MODE == CODE_MODE_UNCALIBRATED_ON_ROAD_RUNNING )
#define IS_CALIBRATED   0                                       //未标定
#else
#define IS_CALIBRATED   0                                       //未标定
#endif


#define Value_2Pi       6.283185307180f                       // 2*pi
#define Value_4Pi3      4.188790204786f                       // 4*pi/3
#define Value_Pi        3.141592653590f                       // pi
#define Value_2Pi3      2.094395102393f                       // 2*pi/3
#define Value_Pi2       1.570796326795f                       // pi/2
#define Value_Pi3       1.047197551197f                       // pi/3
#define Value_Pi4       0.785398163398f                       // pi/4
#define Value_Pi6       0.523598775598f                       // pi/5

#define sqrt3           1.7320508075688772935274463415059f    // sqrt(3)
#define sqrt2           1.4142135623730950488016887242097f    // sqrt(2)
#define Cnst_1div3      0.33333333333333333333333333333333f   // 1/3
#define Cnst_1divsqrt3  0.57735026918962576450914878050196f   // 1/sqrt(3)
#define Cnst_1divsqrt2  0.707106781f                          // 1/sqrt(2)
#define Cnst_SQRT3div2  0.86602540378443864676372317075294f   // sqrt(3)/2
#define Cnst_2divsqrt3  1.154700538f                          // 2/sqrt(3)
#define Cnst_2div3      0.666666667f                          // 2/3
#define Cnst_1div60     0.016666667f                          // 1/60

extern void GetSpdOrTorIdIqRef(void);
extern void RunDataIntit(void);

extern void DisableDrive(void);
extern void EnableDrive(void);
extern void InitForMotorApp(void);

extern uint16_t gu16ADCounter;
extern uint16_t g_u16SpeedToTorqExitCnt;


#endif /* SOURCE_MOTOR_INCLUDE_MOTORINCLUDE_H_ */
