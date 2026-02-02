/*
 * MotorRotorPos.h
 *
 *  Created on: 2024年12月20日
 *      Author: pc
 */
#ifndef SOURCE_MOTOR_INCLUDE_MOTORROTORPOS_H_
#define SOURCE_MOTOR_INCLUDE_MOTORROTORPOS_H_

typedef struct ROTOR_TRANS_STRUCT_DEF{
    uint32_t   TimeBak;                //上一次测速的基准时间
    uint32_t   DetaTime;
    int16_t     RTPos;
    float RTPosDelt;
    uint16_t    Poles;                  //编码器的极对数
    int16_t     ConFlag;                //
    float Pos;
    float PrevPos1;
    float PrevPos0;
    int16_t     PosBak;
    int16_t     DetaPos;
    float FreqFeed;
    uint16_t    Flag;
    int16_t     Coff;
    uint8_t    Status;
    float PosComp;
    float ParkPosComp;
    int16_t     LimitCnt;
    int16_t     SPIStartFlag;
    float PosCompCoef;
}ROTOR_TRANS_STRUCT;

typedef struct ROTOR_POS_TRIP_STRUCT_DEF
{
    uint16_t  isTrip;
    float prevPos1;
    float prevPos0;
    float currPos;
    float currSpeed;
}ROTOR_POS_TRIP_STRUCT;

typedef struct IPM_POSITION_STRUCT_DEF {
    float RotorPos;               //转子位置角
    float RotorZero;              //编码器零点位置角
}IPM_POSITION_STRUCT;//永磁同步电机和转子角度相关的结构

extern void RotorTransSamplePos(void);
extern void RotorTransReadPos(void);
extern void RotorTransCalVel(void);

extern IPM_POSITION_STRUCT      gIPMPos;
extern ROTOR_TRANS_STRUCT       gRotorTrans;
extern ROTOR_POS_TRIP_STRUCT    gRotorPosTrip;

extern uint16_t gu16RtPosUnreadFlag;
extern float g_SpeedFilter;
#endif /* SOURCE_MOTOR_INCLUDE_MOTORROTORPOS_H_ */
