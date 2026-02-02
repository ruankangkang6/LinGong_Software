//=====================================================================
//
// 模块主要功能：通用模块
//
//=====================================================================

#include "f_public.h"

//#pragma CODE_SECTION(PIRegulator,   "ramfuncs")
//#pragma CODE_SECTION(Saturatefloat,       "ramfuncs")
//#pragma CODE_SECTION(Saturateint16_t,         "ramfuncs")

//-------全局变量---------
CanReDmdStruct gDmdCanRe         = {0, 0, 0, 0, 0, 0, 0, 0};                   //CAN接收控制数据
CanSedDmdStruct gDmdCanSed       = {0, 0, 0, 0, 0};                            //CAN发送控制数据
INV_STRUCT gInvInfo              = {0, 0, 0, 0, 0, 0, 0};                      //变频器信息
MOTOR_STRUCT gMotorInfo          = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
                                    0, 0, 0, 0, 0, 0, 0};                                  //电机参数信息

extern int16_t gASR_OUTkp;
extern int16_t gASR_OUTki;
/*************************************************************
        最大最小限制(float类型数据)
*************************************************************/
void Saturatefloat( float* Var, float Max, float Min )
{
    if( *Var > Max )
    {
        *Var = Max;
    }

    if( *Var < Min )
    {
        *Var = Min;
    }
}

/*************************************************************
        最大最小限制(uint16_t类型数据)
*************************************************************/
void Saturateuint16_t( uint16_t* Var, uint16_t Max, uint16_t Min )
{
    if( *Var > Max )
    {
        *Var = Max;
    }

    if( *Var < Min )
    {
        *Var = Min;
    }
}

/*************************************************************
            最大最小限制(int16_t类型数据)
*************************************************************/
void Saturateint16_t( int16_t* Var, int16_t Max, int16_t Min )
{
    if( *Var > Max )
    {
        *Var = Max;
    }

    if( *Var < Min )
    {
        *Var = Min;
    }
}


/*************************************************************
    PI调节器1
*************************************************************/
void PIRegulator( PIReg* pReg )
{
    pReg->ErrZ = pReg->Err;
    pReg->Err = pReg->Ref - pReg->Fb;
    Saturatefloat( &pReg->Err, pReg->ErrLmt, -pReg->ErrLmt );

    gASR_OUTkp =  (pReg->Kp*(pReg->Err - pReg->ErrZ)) * 10;
    gASR_OUTki =  (pReg->Ki*pReg->Err) * 10;

    pReg->PIOut += pReg->Kp*(pReg->Err - pReg->ErrZ) + pReg->Ki*pReg->Err;
//    pReg->PIOut += pReg->Kp*pReg->Err + pReg->Ki*pReg->Err;

    Saturatefloat( &pReg->PIOut, pReg->MaxPIOut, pReg->MinPIOut );
}

/*************************************************************
    PI调节器2
*************************************************************/
void PIRegulatorEx( PIReg* pReg )
{
    float m_KiErrLmt;

    pReg->ErrZ = pReg->Err;
    pReg->Err = pReg->Ref - pReg->Fb;
    Saturatefloat( &pReg->Err, pReg->ErrLmt, -pReg->ErrLmt );

    m_KiErrLmt = pReg->Err;
    Saturatefloat( &m_KiErrLmt, 20, -20 );

    pReg->PIOut += pReg->Kp*(pReg->Err - pReg->ErrZ) + pReg->Ki*m_KiErrLmt;

    Saturatefloat( &pReg->PIOut, pReg->MaxPIOut, pReg->MinPIOut );
}


/*************************************************************
    加减速直线计算
*************************************************************/
void LineChangeCalc(LINE_CHANGE_STRUCT *p)
{
    float delta;

    if (!p->tickerAll)
    {
        p->curValue = p->aimValue;
    }
    else
    {
        delta = p->maxValue/p->tickerAll;

        if (p->aimValue > p->curValue)
        {
            p->curValue += delta;
            if (p->curValue > p->aimValue)
                p->curValue = p->aimValue;
        }
        else
        {
            p->curValue -= delta;
            if (p->curValue < p->aimValue)
                p->curValue = p->aimValue;
        }
    }
}
