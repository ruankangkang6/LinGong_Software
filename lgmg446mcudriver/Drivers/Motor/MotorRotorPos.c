/*
 * MotorRotorPos.c
 *
 *  Created on: 2024年12月20日
 *      Author: pc
 */
#include "f_public.h"
#include "main.h"

void RotorTransSamplePos(void);
void RotorTransReadPos(void);
void RotorTransCalVel(void);

//#pragma CODE_SECTION(RotorTransCalVel,          "ramfuncs")
//#pragma CODE_SECTION(RotorTransReadPos,         "ramfuncs")
//#pragma CODE_SECTION(RotorTransSamplePos,       "ramfuncs")

IPM_POSITION_STRUCT     gIPMPos;
ROTOR_TRANS_STRUCT      gRotorTrans;
ROTOR_POS_TRIP_STRUCT   gRotorPosTrip;

uint16_t gu16RtPosUnreadFlag = 0;    //SPI信号丢失标志
float g_SpeedFilter = 0;          //速度滤波
/*************************************************************
    旋转变压器获取位置函数，通过SPI读取位置信号
*************************************************************/
void RotorTransReadPos(void)
{
	uint16_t tmpPos = 0;

//    if( 0 == (SPI3->SR&0x01)) //SPI信号丢失
//    {
//        gRotorTrans.Pos += gRotorTrans.RTPosDelt;//SPI信号丢失，旋变信号延续上周期步进值进行增减
//        gRotorTrans.Pos = RoundArcAngle(gRotorTrans.Pos);
//        gu16RtPosUnreadFlag = 1;
//        gRotorTrans.LimitCnt++;
//        if ( gRotorTrans.LimitCnt >= 3 )  //连续检查3次丢失，报旋变故障并关波
//        {
//            DisableDrive();
////            gMainStatus.ErrFlag.bit.RTErrSpi = 1;
//            gFault3rd.bit.ErrorRTTrans = 1;
//        }
//        return;
//    }

    gRotorTrans.LimitCnt = 0;
    gu16RtPosUnreadFlag = 0;

    MS5910_RDPos(&tmpPos);   //读取旋变信息

    gRotorTrans.RTPos = tmpPos;          //旋变位置信息 0-65535

    if( gRotorTrans.Poles <= 0 )        // 处理编码器极对数问题
    {
        gRotorTrans.Poles = 1;
    }

    gRotorTrans.Pos = Value_2Pi*gMotorInfo.Poles*tmpPos/( gRotorTrans.Poles*65535 ); //旋变位置信息单位换算为PI，并等效为电角度变化
    gRotorTrans.RTPosDelt = gRotorTrans.Pos - gRotorTrans.PrevPos0;//相邻两周期位置查差值(变化量)


    gRotorTrans.PosComp  =  Value_2Pi*gRotorTrans.PosCompCoef*gRotorSpeed.SpeedApply*gBasePar.Ts; //位置信息补偿


    gRotorTrans.ParkPosComp     =   0;//Value_2Pi*gRotorTrans.PosCompCoef*gRotorSpeed.SpeedApply*gBasePar.Ts; //位置信息补偿


    if ( fabs(gRotorSpeed.SpeedApply) > 10 && fabs( gRotorTrans.Pos - gRotorTrans.PrevPos0 ) > 4*fabs(gRotorTrans.PosComp) )//位置信息变化过大
    {
        if ( ( gRotorSpeed.SpeedApply > 0 && gRotorTrans.Pos > gRotorTrans.PrevPos0 )
          || ( gRotorSpeed.SpeedApply < 0 && gRotorTrans.Pos < gRotorTrans.PrevPos0 ) )
        {
            gRotorPosTrip.isTrip    =   1;//位置信号变化过大标志

            //用于外发观察
            gRotorPosTrip.prevPos1  =   gRotorTrans.PrevPos1;
            gRotorPosTrip.prevPos0  =   gRotorTrans.PrevPos0;
            gRotorPosTrip.currPos   =   gRotorTrans.Pos;
            gRotorPosTrip.currSpeed =   gRotorSpeed.SpeedApply;

        }
    }

    gRotorTrans.PrevPos1 = gRotorTrans.PrevPos0;
    gRotorTrans.PrevPos0 = gRotorTrans.Pos;
    MS5910_RDFLT(&gRotorTrans.Status);

    RTCheck();
}

/*************************************************************
    旋转变压器获取位置前的数据采样处理，同时记录准确的时间间隔
    每0.45ms检测一次速度，下溢中断处理
*************************************************************/
void RotorTransSamplePos(void)
{
	uint32_t mTime;
    uint32_t mDetaTime;
    mTime = SysTick->VAL;//GetTime();  //时间累加

//    mDetaTime = labs((long)(gRotorTrans.TimeBak - mTime));
    if(gRotorTrans.TimeBak > mTime)
    {
    	mDetaTime = ((long)(gRotorTrans.TimeBak - mTime));
    }
    else
    {
    	mDetaTime = ((long)(SysTick->LOAD - mTime + gRotorTrans.TimeBak));
    }
    gRotorTrans.Flag = 0;
    if( mDetaTime >= 81000) //时间差值
    {
        gRotorTrans.Flag        =   1;//经过0.45ms标志
        gRotorTrans.DetaTime    =   mDetaTime;
        gRotorTrans.TimeBak     =   mTime;
    }
}

/*************************************************************
    旋转变压器情况下，计算速度函数
*************************************************************/
float gf32DetaPos = 0;
void RotorTransCalVel(void)
{
    float f32DetaPos = 0;
    float f32Speed = 0;

    if( 0 == gRotorTrans.Flag )
    {
        return; //0.45MS计算时间没有到不计算速度
    }

    //跨越65535时怎么办，下一时刻20-上一时刻65500
    //int是16位，过65535时数据被截取，截取数据正好与未过数据一样，它们相减不能超过32767，不然相减数据变为负数。
    gRotorTrans.DetaPos = (int16_t)((uint16_t)gRotorTrans.RTPos - (uint16_t)gRotorTrans.PosBak);//每过0.45ms，旋变位置信息的变化量

    gRotorTrans.PosBak  = gRotorTrans.RTPos;

    //------------------------------------
    // 求电角度变化情况
    //------------------------------------
//  gMotorInfo.Poles = 4;
//  gRotorTrans.Poles = 4;
    f32DetaPos = (float)gRotorTrans.DetaPos*gMotorInfo.Poles/gRotorTrans.Poles;  //电角度变化 = 旋角变化*P电极对数/P旋极对数
    gf32DetaPos = f32DetaPos;
    //----------------------------------------
    // 采样得到的角度变化为0时:逐渐减小速度
    //----------------------------------------
    if( 0 == gRotorTrans.DetaPos )
    {
        gRotorTrans.Coff++;
        if( gRotorTrans.Coff > 32 )
        {
            gRotorTrans.Coff = 32;
            f32Speed = 0;
        }
        else
        {
            f32Speed = gRotorTrans.FreqFeed/gRotorTrans.Coff;
        }

        gRotorTrans.FreqFeed = 0;   //pz debug
    }
    else
    {
        gRotorTrans.Coff = 0;

        f32DetaPos = fabs(f32DetaPos);

        //计算速度频率
        f32Speed = f32DetaPos*180*15.259f/gRotorTrans.DetaTime;   // 15.259 = 1000*1000/4096 (0.00045s-450.之间1000*1000)，f电 = x/65535/0.00045s = x*15.259/450。uint:HZ
        if( gRotorTrans.DetaPos < 0 )
        {
            f32Speed = -f32Speed;
        }
    }

    gRotorSpeed.Speed               =   f32Speed;   //未滤波的速度频率
    gRotorTrans.FreqFeed            =   ( 1 - g_SpeedFilter )*gRotorTrans.FreqFeed + g_SpeedFilter*f32Speed;
    gRotorSpeed.SpeedApply          =   gRotorTrans.FreqFeed;  //一次滤波
    gRotorSpeed.SpeedApplyFilter    =   0.99f*gRotorSpeed.SpeedApplyFilter + ( 1 - 0.99f )*gRotorSpeed.SpeedApply;//二次滤波

    gRotorSpeed.SpeedApplyFilter500msSum += gRotorSpeed.Speed; //速度累加
    ++gRotorSpeed.SpeedApplyFilter500msCnt;
    if ( gRotorSpeed.SpeedApplyFilter500msCnt >= 200 )
    {
        gRotorSpeed.SpeedApplyFilter500msZ2     =   gRotorSpeed.SpeedApplyFilter500msZ1;
        gRotorSpeed.SpeedApplyFilter500msZ1     =   gRotorSpeed.SpeedApplyFilter500ms;

        if ( 400 == gRotorSpeed.SpeedApplyFilter500msCnt )
        {
            gRotorSpeed.SpeedApplyFilter500ms   =   gRotorSpeed.SpeedApplyFilter500msSum*0.001f; //累计1000次，大概450ms的速度平均值
        }
        else
        {
            gRotorSpeed.SpeedApplyFilter500ms   =   gRotorSpeed.SpeedApplyFilter500msSum/gRotorSpeed.SpeedApplyFilter500msCnt;
        }

        gRotorSpeed.SpeedApplyFilter500msSum    =   0;
        gRotorSpeed.SpeedApplyFilter500msCnt    =   0;
    }
}
