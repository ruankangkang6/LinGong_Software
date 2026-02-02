#ifndef __MOTOR_CALIBRATION_H__
#define __MOTOR_CALIBRATION_H__

extern void GetCalibIdqRef( float f, float t, float *pIdVal, float *pIqVal );
extern float GetCalibTorque( float f, float IdVal, float IqVal );
//extern void MotorCalibrationTest( void );
extern void CalcMaxTorqueRange( float f );

extern float gf32MaxMotorTorque;
extern float gf32MaxGeneratorTorque;
#endif
