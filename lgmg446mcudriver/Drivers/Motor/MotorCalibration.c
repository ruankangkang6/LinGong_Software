/*
 * MotorCalibration.c
 *
 *  Created on: 2024年12月5日
 *      Author: pc
 */
#include "f_public.h"
#include "main.h"
#include "CalibrationData_Liugong2in1_Drive.h"   //电机数据头文件单独包含在‘MotorCalibration.c’，减多余的变量定义域

//#pragma CODE_SECTION(interp2,                   "ramfuncs")
//#pragma CODE_SECTION(CalcTorqueIndex,           "ramfuncs")
//#pragma CODE_SECTION(GetCalibIdqRef,            "ramfuncs")
//#pragma CODE_SECTION(GetIdqRefBoundary,         "ramfuncs")
//#pragma CODE_SECTION(GetInterpValueEx,          "ramfuncs")
//#pragma CODE_SECTION(GetTorqueOnGrid,           "ramfuncs")
//#pragma CODE_SECTION(InstallCalibParams,        "ramfuncs")
//#pragma CODE_SECTION(InstallCalibParams_LV,     "ramfuncs")
//#pragma CODE_SECTION(CalcMaxTorque,             "ramfuncs")
//#pragma CODE_SECTION(GetCalibIdqValEx,          "ramfuncs")
//#pragma CODE_SECTION(GetIdOnGrid,               "ramfuncs")
//#pragma CODE_SECTION(GetIqOnGrid,               "ramfuncs")

float gf32MaxMotorTorque = 0;
float gf32MaxGeneratorTorque = 0;
static uint32_t s_u16IsLowVoltCalib = 0;


struct POINT_DEF
{
    uint32_t i;
    uint32_t j;
    float x;
    float y;
    float z;
};

struct INTERP2_DEF
{
    struct POINT_DEF p11;
    struct POINT_DEF p12;
    struct POINT_DEF p21;
    struct POINT_DEF p22;
    struct POINT_DEF p;
};


static const uint32_t *Calib_pTorqLenTab = 0;
static uint32_t Calib_SpeedTabLen = 0;
static float Calib_TorqueDelta = 0;
static const float *Calib_pSpeedTab = 0;
static const float *Calib_NM_pMaxTorqueTab = 0;
static const float *Calib_NG_pMaxTorqueTab = 0;
static const float *Calib_PM_pMaxTorqueTab = 0;
static const float *Calib_PG_pMaxTorqueTab = 0;
static const float **Calib_NM_ppIdTab = 0;
static const float **Calib_NG_ppIdTab = 0;
static const float **Calib_PM_ppIdTab = 0;
static const float **Calib_PG_ppIdTab = 0;
static const float **Calib_NM_ppIqTab = 0;
static const float **Calib_NG_ppIqTab = 0;
static const float **Calib_PM_ppIqTab = 0;
static const float **Calib_PG_ppIqTab = 0;


static void InstallCalibParams( void );
static void InstallCalibParams_LV( void );


void interp2( struct INTERP2_DEF *pPoints )
{
    float zVal1 = 0;
    float zVal2 = 0;

    if( pPoints->p21.x == pPoints->p11.x )
    {
        zVal1 = 0.5*(pPoints->p11.z + pPoints->p21.z);
    }
    else
        zVal1 = ( ( pPoints->p21.x - pPoints->p.x )*pPoints->p11.z + ( pPoints->p.x - pPoints->p11.x )*pPoints->p21.z )/( pPoints->p21.x - pPoints->p11.x );

    if( pPoints->p21.x == pPoints->p11.x )
    {
        zVal2 = 0.5*(pPoints->p12.z + pPoints->p22.z);
    }
    else
        zVal2 = ( ( pPoints->p21.x - pPoints->p.x )*pPoints->p12.z + ( pPoints->p.x - pPoints->p11.x )*pPoints->p22.z )/( pPoints->p21.x - pPoints->p11.x );

    if( pPoints->p12.y == pPoints->p11.y )
    {
        pPoints->p.z = 0.5*( zVal1 + zVal2 );
    }
    else
        pPoints->p.z = ( ( pPoints->p12.y - pPoints->p.y )*zVal1 + ( pPoints->p.y - pPoints->p11.y )*zVal2 )/( pPoints->p12.y - pPoints->p11.y );
}

void CalcTorqueIndex( float t, uint32_t tqLen, uint32_t *j0, uint32_t *j1 )
{
    float torque = 0;
    uint32_t i = 0;

    if ( 0 == tqLen || 1 == tqLen )
    {
        *j0 = 0;
        *j1 = 0;

        return;
    }

    for( i = 1; i < tqLen; ++i )
    {
        torque += Calib_TorqueDelta;

        if( torque >= t )
        {
            break;
        }
    }

    if ( i >= tqLen )
    {
        i = tqLen - 1;
    }

    *j0 = i - 1;
    *j1 = i;
}

void GetIdqRefBoundary( float f, float t, const float* pMaxTorqueTab, struct POINT_DEF *pPoint1, struct POINT_DEF *pPoint2 )
{
    int16_t i = 0;
    int16_t spdIdx = 0;
    uint32_t tqLen = 0;

    float fmin = 0;
    float fmax = 0;

    float tmin = 0;
    float tmax = 0;

    fmax = Calib_pSpeedTab[Calib_SpeedTabLen - 1];

    if( f <= fmin )
    {
        spdIdx = 0;
        tqLen = Calib_pTorqLenTab[spdIdx];

        tmin = 0;
        tmax = pMaxTorqueTab[spdIdx];

        if( t < tmin )
        {
            pPoint1->i = 0;
            pPoint1->j = 0;
            pPoint2->i = 0;
            pPoint2->j = 0;
        }
        else if( t >= tmax )
        {
            pPoint1->i = 0;
            pPoint1->j = tqLen - 1;
            pPoint2->i = 0;
            pPoint2->j = tqLen - 1;
        }
        else
        {
            pPoint1->i = 0;
            pPoint2->i = 0;

            CalcTorqueIndex( t, tqLen, &(pPoint1->j), &(pPoint2->j) );
        }
    }
    else if( f >= fmax )
    {
        spdIdx = Calib_SpeedTabLen - 1;

        tqLen = Calib_pTorqLenTab[spdIdx];
        tmin = 0;
        tmax = pMaxTorqueTab[spdIdx];

        if( t >= tmax )
        {
            pPoint1->i = Calib_SpeedTabLen - 1;
            pPoint1->j = tqLen - 1;
            pPoint2->i = Calib_SpeedTabLen - 1;
            pPoint2->j = tqLen - 1;
        }
        else if( t <= tmin )
        {
            pPoint1->i = Calib_SpeedTabLen - 1;
            pPoint1->j = 0;
            pPoint2->i = Calib_SpeedTabLen - 1;
            pPoint2->j = 0;
        }
        else
        {
            pPoint1->i = Calib_SpeedTabLen - 1;
            pPoint2->i = Calib_SpeedTabLen - 1;

            CalcTorqueIndex( t, tqLen, &(pPoint1->j), &(pPoint2->j) );
        }
    }
    else
    {
        for( i = 1; i < Calib_SpeedTabLen; ++i )
        {
            if( Calib_pSpeedTab[i] >= f )
            {
                break;
            }
        }

    //  spdIdx = i;
        spdIdx = i - 1; // modified
        pPoint1->i = i - 1;
        pPoint2->i = i;
        tqLen = Calib_pTorqLenTab[spdIdx];

        CalcTorqueIndex( t, tqLen, &(pPoint1->j), &(pPoint2->j) );
    }

}

float GetTorqueOnGrid(uint32_t spdIdx, uint32_t tqIdx, const float *pMaxTorqueTab)
{
    return tqIdx*Calib_TorqueDelta;
}

float GetIdOnGrid(uint32_t spdIdx, uint32_t tqIdx, const float **pIdTab, const float *pMaxTorqueTab)
{
    float ret = 0;

    if( tqIdx >= Calib_pTorqLenTab[spdIdx] - 1 )
    {
        float Id0, Id1;
        float Tq0, Tq1, Tq;

        Id0 = pIdTab[spdIdx][Calib_pTorqLenTab[spdIdx] - 2];
        Id1 = pIdTab[spdIdx][Calib_pTorqLenTab[spdIdx] - 1];
        Tq0 = Calib_TorqueDelta*(Calib_pTorqLenTab[spdIdx] - 2);
        Tq1 = pMaxTorqueTab[spdIdx];
        Tq  = Calib_TorqueDelta*(Calib_pTorqLenTab[spdIdx] - 1);

        if( Tq1 == Tq0 )
            ret = Id0;
        else
            ret = (Id1 - Id0)/(Tq1 - Tq0)*(Tq - Tq0) + Id0;
    }
    else
    {
        ret = pIdTab[spdIdx][tqIdx];
    }

    return ret;
}

float GetIqOnGrid(uint32_t spdIdx, uint32_t tqIdx, const float **pIqTab, const float *pMaxTorqueTab)
{
    float ret = 0;

    if( tqIdx >= Calib_pTorqLenTab[spdIdx] - 1 )
    {
        float Iq0, Iq1;
        float Tq0, Tq1, Tq;

        Iq0 = pIqTab[spdIdx][Calib_pTorqLenTab[spdIdx] - 2];
        Iq1 = pIqTab[spdIdx][Calib_pTorqLenTab[spdIdx] - 1];
        Tq0 = Calib_TorqueDelta*(Calib_pTorqLenTab[spdIdx] - 2);
        Tq1 = pMaxTorqueTab[spdIdx];
        Tq  = Calib_TorqueDelta*(Calib_pTorqLenTab[spdIdx] - 1);

        if( Tq1 == Tq0 )
            ret = Iq0;
        else
            ret = (Iq1 - Iq0)/(Tq1 - Tq0)*(Tq - Tq0) + Iq0;
    }

    else
    {
        ret = pIqTab[spdIdx][tqIdx];
    }

    return ret;
}

void GetInterpValueEx( float f, float t, struct POINT_DEF *pPoint1, struct POINT_DEF *pPoint2,\
    const float **pIdTab, const float **pIqTab, const float* pMaxTorqueTab, float *pDVal, float *pQVal )
{
    float maxId;
    struct INTERP2_DEF inter2Dat;

    if( pPoint1->i == pPoint2->i && pPoint1->j == pPoint2->j )
    {
        uint32_t spdIdx = pPoint1->i;
        uint32_t tqIdx  = pPoint1->j;
        if( tqIdx >= Calib_pTorqLenTab[spdIdx] - 1 )
        {
            tqIdx = Calib_pTorqLenTab[spdIdx] - 1;
        }

        *pDVal = pIdTab[spdIdx][tqIdx];
        *pQVal = pIqTab[spdIdx][tqIdx];

        return;
    }

    inter2Dat.p11.x = Calib_pSpeedTab[pPoint1->i];
    inter2Dat.p11.y = GetTorqueOnGrid( pPoint1->i, pPoint1->j, pMaxTorqueTab );
    inter2Dat.p11.z = GetIdOnGrid(pPoint1->i, pPoint1->j, pIdTab, pMaxTorqueTab);

    inter2Dat.p12.x = Calib_pSpeedTab[pPoint1->i];
    inter2Dat.p12.y = GetTorqueOnGrid( pPoint1->i, pPoint2->j, pMaxTorqueTab );
    inter2Dat.p12.z = GetIdOnGrid(pPoint1->i, pPoint2->j, pIdTab, pMaxTorqueTab);

    inter2Dat.p21.x = Calib_pSpeedTab[pPoint2->i];
    inter2Dat.p21.y = GetTorqueOnGrid( pPoint2->i, pPoint1->j, pMaxTorqueTab );
    inter2Dat.p21.z = GetIdOnGrid(pPoint2->i, pPoint1->j, pIdTab, pMaxTorqueTab);

    inter2Dat.p22.x = Calib_pSpeedTab[pPoint2->i];
    inter2Dat.p22.y = GetTorqueOnGrid( pPoint2->i, pPoint2->j, pMaxTorqueTab );
    inter2Dat.p22.z = GetIdOnGrid(pPoint2->i, pPoint2->j, pIdTab, pMaxTorqueTab);

    inter2Dat.p.x = f;
    inter2Dat.p.y = t;
    inter2Dat.p.z = 0;

    interp2(&inter2Dat);

    /////////////////////////////// added 20170901 ///////////////////////////////////////////
    if( pPoint2->j >= Calib_pTorqLenTab[pPoint2->i] - 1 )
    {
        maxId = pIdTab[pPoint2->i][Calib_pTorqLenTab[pPoint2->i] - 1];
        if( pPoint2->j <= Calib_pTorqLenTab[pPoint1->i] - 1 )
        {
            if( maxId > pIdTab[pPoint1->i][pPoint2->j] )
                maxId = pIdTab[pPoint1->i][pPoint2->j];
        }
    }
    else
    {
        maxId = pIdTab[pPoint2->i][pPoint2->j];
    }

    if( pPoint1->j >= Calib_pTorqLenTab[pPoint1->i] - 1 )
    {
        if ( maxId > pIdTab[pPoint1->i][Calib_pTorqLenTab[pPoint1->i] - 1] )
        {
            maxId = pIdTab[pPoint1->i][Calib_pTorqLenTab[pPoint1->i] - 1];
        }
    }
    else
    {
        if ( maxId > pIdTab[pPoint1->i][pPoint1->j] )
        {
            maxId = pIdTab[pPoint1->i][pPoint1->j];
        }
    }

    if ( inter2Dat.p.z < maxId )
    {
        inter2Dat.p.z = maxId;
    }
    //////////////////////////////////////////////////////////////////////////
    *pDVal = inter2Dat.p.z;

    inter2Dat.p11.z = GetIqOnGrid(pPoint1->i, pPoint1->j, pIqTab, pMaxTorqueTab);
    inter2Dat.p12.z = GetIqOnGrid(pPoint1->i, pPoint2->j, pIqTab, pMaxTorqueTab);
    inter2Dat.p21.z = GetIqOnGrid(pPoint2->i, pPoint1->j, pIqTab, pMaxTorqueTab);
    inter2Dat.p22.z = GetIqOnGrid(pPoint2->i, pPoint2->j, pIqTab, pMaxTorqueTab);

    interp2(&inter2Dat);
    *pQVal = inter2Dat.p.z;
}

float CalcMaxTorque( float f, const float* pMaxTorqueTab )
{
    int16_t i = 0;

    float f1 = 0;
    float f2 = 0;
    float tmax1 = 0;
    float tmax2 = 0;

    if( 0 == s_u16IsLowVoltCalib && f <= BASE_SPEED_VALUE )
    {
        return RATED_TORQUE_VALUE;
    }
    else if( f >= Calib_pSpeedTab[Calib_SpeedTabLen - 1] )
    {
        return pMaxTorqueTab[Calib_SpeedTabLen - 1];
    }
    else
    {
        for( i = 1; i < Calib_SpeedTabLen; ++i )
        {
            if( Calib_pSpeedTab[i] >= f )
            {
                break;
            }
        }

        f1 = Calib_pSpeedTab[i - 1];
        f2 = Calib_pSpeedTab[i];

        if ( 0 == s_u16IsLowVoltCalib && f1 <= BASE_SPEED_VALUE && f2 > BASE_SPEED_VALUE )
        {
            f1 = BASE_SPEED_VALUE;
            tmax1 = RATED_TORQUE_VALUE;;
        }
        else
            tmax1 = pMaxTorqueTab[i - 1];

        tmax2 = pMaxTorqueTab[i];

        return ((f - f1)*(tmax2 - tmax1)/(f2 - f1) + tmax1);
    }
}


void GetCalibIdqValEx( float f, float t, float *pIdVal, float *pIqVal )
{
    struct POINT_DEF point1, point2;
    const float **ppIdTab = 0;
    const float **ppIqTab = 0;
    const float *pMaxTorqueTab = 0;
    float tmax;
    float tabs;
    float fInput;

    // if negative rotate is calibrated, then f = -f, t = -t;
    if( IS_NEGATIVE_SPEED_CALIBRATED )
    {
        f = -f;
        t = -t;
    }

    fInput = f;
    if ( f < 0 ) f = -f;

    if( ( t >= 0 && fInput >= 0 ) || ( t < 0 && fInput < 0 ) )
    {
        pMaxTorqueTab = Calib_PM_pMaxTorqueTab;
        tmax = CalcMaxTorque( f, pMaxTorqueTab );
        tabs = t;
        if( tabs < 0 ) tabs = -tabs;
        if( tabs > tmax ) tabs = tmax;
//        gLineCur.TorqueShow = tabs;
        GetIdqRefBoundary( f, tabs, pMaxTorqueTab, &point1, &point2 );
        ppIdTab = Calib_PM_ppIdTab;
        ppIqTab = Calib_PM_ppIqTab;
        GetInterpValueEx( f, tabs, &point1, &point2, ppIdTab, ppIqTab, pMaxTorqueTab, pIdVal, pIqVal );

        if ( fInput < 0 )
        {
            *pIqVal = -*pIqVal;
        }
    }
    else
    {
        pMaxTorqueTab = Calib_PG_pMaxTorqueTab;
        tmax = CalcMaxTorque( f, pMaxTorqueTab );
        tabs = t;
        if( tabs < 0 ) tabs = -tabs;
        if( tabs > tmax ) tabs = tmax;
//        gLineCur.TorqueShow = tabs;
        GetIdqRefBoundary( f, tabs, pMaxTorqueTab, &point1, &point2 );
        ppIdTab = Calib_PG_ppIdTab;
        ppIqTab = Calib_PG_ppIqTab;
        GetInterpValueEx( f, tabs, &point1, &point2, ppIdTab, ppIqTab, pMaxTorqueTab, pIdVal, pIqVal );

        if ( fInput < 0 )
        {
            *pIqVal = -*pIqVal;
        }

    }

}

void GetCalibIdqRef( float f, float t, float *pIdVal, float *pIqVal )
{
    float IdRefL, IdRefH;
    float IqRefL, IqRefH;

    if ( gUDC.uDCBigFilter >= CALIB_RATED_VOLT )//- CALIB_VOLT_RATED_LOW_DELT )
    {
        InstallCalibParams();
        GetCalibIdqValEx(f, t, pIdVal, pIqVal);
    }
    else if ( gUDC.uDCBigFilter <= CALIB_LOW_VOLT )
    {
        InstallCalibParams_LV();
        GetCalibIdqValEx(f, t, pIdVal, pIqVal);
    }
    else
    {
        InstallCalibParams_LV();
        GetCalibIdqValEx(f, t, &IdRefL, &IqRefL);
        InstallCalibParams();
        GetCalibIdqValEx(f, t, &IdRefH, &IqRefH);
        *pIdVal = ( IdRefL - IdRefH )/( CALIB_LOW_VOLT - CALIB_RATED_VOLT )*( gUDC.uDCBigFilter - CALIB_RATED_VOLT ) + IdRefH;
        *pIqVal = ( IqRefL - IqRefH )/( CALIB_LOW_VOLT - CALIB_RATED_VOLT )*( gUDC.uDCBigFilter - CALIB_RATED_VOLT ) + IqRefH;
    }
}


void InstallCalibParams( void )
{
    Calib_SpeedTabLen       =   CALIB_SPEED_LEN;
    Calib_pSpeedTab         =   CALIB_SPEED_TABLE;
    Calib_TorqueDelta       =   TORQUE_DELTA;
    Calib_pTorqLenTab       =   TORQUE_LEN_TABLE;
    Calib_NM_pMaxTorqueTab  =   MAX_TORQUE_TABLE_M;
    Calib_NG_pMaxTorqueTab  =   MAX_TORQUE_TABLE_G;
    Calib_PM_pMaxTorqueTab  =   MAX_TORQUE_TABLE_M;
    Calib_PG_pMaxTorqueTab  =   MAX_TORQUE_TABLE_G;
    Calib_NM_ppIdTab        =   CALIB_ID_TABLE_M;
    Calib_NG_ppIdTab        =   CALIB_ID_TABLE_G;
    Calib_PM_ppIdTab        =   CALIB_ID_TABLE_M;
    Calib_PG_ppIdTab        =   CALIB_ID_TABLE_G;
    Calib_NM_ppIqTab        =   CALIB_IQ_TABLE_M;
    Calib_NG_ppIqTab        =   CALIB_IQ_TABLE_G;
    Calib_PM_ppIqTab        =   CALIB_IQ_TABLE_M;
    Calib_PG_ppIqTab        =   CALIB_IQ_TABLE_G;

    s_u16IsLowVoltCalib     =   0;
}

void InstallCalibParams_LV( void )
{
    Calib_SpeedTabLen       =   CALIB_SPEED_LEN_LV;
    Calib_pSpeedTab         =   CALIB_SPEED_TABLE_LV;
    Calib_TorqueDelta       =   TORQUE_DELTA_LV;
    Calib_pTorqLenTab       =   TORQUE_LEN_TABLE_LV;
    Calib_NM_pMaxTorqueTab  =   MAX_TORQUE_TABLE_M_LV;
    Calib_NG_pMaxTorqueTab  =   MAX_TORQUE_TABLE_G_LV;
    Calib_PM_pMaxTorqueTab  =   MAX_TORQUE_TABLE_M_LV;
    Calib_PG_pMaxTorqueTab  =   MAX_TORQUE_TABLE_G_LV;
    Calib_NM_ppIdTab        =   CALIB_ID_TABLE_M_LV;
    Calib_NG_ppIdTab        =   CALIB_ID_TABLE_G_LV;
    Calib_PM_ppIdTab        =   CALIB_ID_TABLE_M_LV;
    Calib_PG_ppIdTab        =   CALIB_ID_TABLE_G_LV;
    Calib_NM_ppIqTab        =   CALIB_IQ_TABLE_M_LV;
    Calib_NG_ppIqTab        =   CALIB_IQ_TABLE_G_LV;
    Calib_PM_ppIqTab        =   CALIB_IQ_TABLE_M_LV;
    Calib_PG_ppIqTab        =   CALIB_IQ_TABLE_G_LV;

    s_u16IsLowVoltCalib     =   1;
}


void CalcMaxTorqueRange( float f )
{
    const float *pMaxTorqueTab = 0;

    float maxTML, maxTMH;
    float maxTGL, maxTGH;

    if ( f < 0 ) f = -f;

    if ( gUDC.uDCBigFilter >= CALIB_RATED_VOLT )//- 5 )
    {
        InstallCalibParams();
        pMaxTorqueTab = Calib_PM_pMaxTorqueTab;
        gf32MaxMotorTorque = CalcMaxTorque( f, pMaxTorqueTab );

        pMaxTorqueTab = Calib_PG_pMaxTorqueTab;
        gf32MaxGeneratorTorque = CalcMaxTorque( f, pMaxTorqueTab );
    }
    else if ( gUDC.uDCBigFilter <= CALIB_LOW_VOLT )
    {
        InstallCalibParams_LV();
        pMaxTorqueTab = Calib_PM_pMaxTorqueTab;
        gf32MaxMotorTorque = CalcMaxTorque( f, pMaxTorqueTab );

        pMaxTorqueTab = Calib_PG_pMaxTorqueTab;
        gf32MaxGeneratorTorque = CalcMaxTorque( f, pMaxTorqueTab );
    }
    else
    {
        InstallCalibParams_LV();
        pMaxTorqueTab = Calib_PM_pMaxTorqueTab;
        maxTML = CalcMaxTorque( f, pMaxTorqueTab );

        pMaxTorqueTab = Calib_PG_pMaxTorqueTab;
        maxTGL = CalcMaxTorque( f, pMaxTorqueTab );

        InstallCalibParams();
        pMaxTorqueTab = Calib_PM_pMaxTorqueTab;
        maxTMH = CalcMaxTorque( f, pMaxTorqueTab );

        pMaxTorqueTab = Calib_PG_pMaxTorqueTab;
        maxTGH = CalcMaxTorque( f, pMaxTorqueTab );

        gf32MaxMotorTorque = ( maxTML - maxTMH )/( CALIB_LOW_VOLT - CALIB_RATED_VOLT )*( gUDC.uDCBigFilter - CALIB_RATED_VOLT ) + maxTMH;
        gf32MaxGeneratorTorque = ( maxTGL - maxTGH )/( CALIB_LOW_VOLT - CALIB_RATED_VOLT )*( gUDC.uDCBigFilter - CALIB_RATED_VOLT ) + maxTGH;
    }
}

static float T_QRY_ID_BUFF[CALIB_TORQUE_LEN] = { 0 };
static float T_QRY_IQ_BUFF[CALIB_TORQUE_LEN] = { 0 };

#define MM_MOTOR 1
#define MM_GENERATOR 0

static const float* pIdTab0 = 0;
static const float* pIqTab0 = 0;
static const float* pIdTab1 = 0;
static const float* pIqTab1 = 0;

void BuildIdqTable( uint32_t spdIdx, float f, uint32_t isNM )
{
    uint32_t i = 0;
    uint32_t j = 0;
    float iq0 = 0;
    float iq1 = 0;
    float id0 = 0;
    float id1 = 0;



    float f32Temp = 0;

    i = spdIdx;

    if( 0 == i )
    {
        if( isNM )
        {
            pIdTab0 = Calib_NM_ppIdTab[i];
            pIqTab0 = Calib_NM_ppIqTab[i];
        }
        else
        {
            pIdTab0 = Calib_NG_ppIdTab[i];
            pIqTab0 = Calib_NG_ppIqTab[i];
        }

        for( j = 0; j < CALIB_TORQUE_LEN; ++j )
        {
            T_QRY_ID_BUFF[j] = pIdTab0[j];
            T_QRY_IQ_BUFF[j] = pIqTab0[j];
        }
    }
    else if( i >= Calib_SpeedTabLen - 1 )
    {
        i = Calib_SpeedTabLen - 1;

        if( isNM )
        {
            pIdTab0 = Calib_NM_ppIdTab[i];
            pIqTab0 = Calib_NM_ppIqTab[i];
        }
        else
        {
            pIdTab0 = Calib_NG_ppIdTab[i];
            pIqTab0 = Calib_NG_ppIqTab[i];
        }

        for( j = 0; j < CALIB_TORQUE_LEN; ++j )
        {
            if( j < Calib_pTorqLenTab[i] )
            {
                T_QRY_ID_BUFF[j] = pIdTab0[j];
                T_QRY_IQ_BUFF[j] = pIqTab0[j];
            }
            else
            {
                T_QRY_ID_BUFF[j] = pIdTab0[Calib_pTorqLenTab[i] - 1];
                T_QRY_IQ_BUFF[j] = pIqTab0[Calib_pTorqLenTab[i] - 1];
            }
        }
    }
    else
    {

        if( isNM )
        {
            pIdTab0 = Calib_NM_ppIdTab[i - 1];
            pIqTab0 = Calib_NM_ppIqTab[i - 1];
            pIdTab1 = Calib_NM_ppIdTab[i];
            pIqTab1 = Calib_NM_ppIqTab[i];
        }
        else
        {
            pIdTab0 = Calib_NG_ppIdTab[i - 1];
            pIqTab0 = Calib_NG_ppIqTab[i - 1];
            pIdTab1 = Calib_NG_ppIdTab[i];
            pIqTab1 = Calib_NG_ppIqTab[i];
        }

        for( j = 0; j < CALIB_TORQUE_LEN; ++j )
        {
            if( j < Calib_pTorqLenTab[i - 1] )
            {
                id0 = pIdTab0[j];
                iq0 = pIqTab0[j];
            }
            else
            {
                id0 = pIdTab0[Calib_pTorqLenTab[i - 1] - 1];
                iq0 = pIqTab0[Calib_pTorqLenTab[i - 1] - 1];
            }

            if( j < Calib_pTorqLenTab[i] )
            {
                id1 = pIdTab1[j];
                iq1 = pIqTab1[j];
            }
            else
            {
                id1 = pIdTab1[Calib_pTorqLenTab[i] - 1];
                iq1 = pIqTab1[Calib_pTorqLenTab[i] - 1];
            }

            f32Temp = ( f - Calib_pSpeedTab[i - 1] )/( Calib_pSpeedTab[i] - Calib_pSpeedTab[i - 1] );

            T_QRY_ID_BUFF[j] = ( id1 - id0 )*f32Temp + id0;
            T_QRY_IQ_BUFF[j] = ( iq1 - iq0 )*f32Temp + iq0;

       }

    }
}



float GetTorqueOnSpeed( uint32_t spdIdx, uint32_t isNM, float f, float IdVal, float IqVal )
{
    uint32_t di, qi;
    uint32_t i;

    uint32_t ignoreT2 = 0;

    float id0, iq0, id1, iq1;
    float t1, t2, t = 0;
    float tMax;

    i = spdIdx;
    qi = 0;

    if( 0 == i )
    {
        i = 1;
    }

    if( isNM )
    {
        while( qi < Calib_pTorqLenTab[i - 1] && IqVal < T_QRY_IQ_BUFF[qi] )
        {
            ++qi;
        }
    }
    else
    {
        while( qi < Calib_pTorqLenTab[i - 1] && IqVal > T_QRY_IQ_BUFF[qi] )
        {
            ++qi;
        }
    }

    if( qi > 0 )
    {
        iq0 = T_QRY_IQ_BUFF[qi - 1];
        iq1 = T_QRY_IQ_BUFF[qi];
    }
    else
    {
        iq0 = T_QRY_IQ_BUFF[qi];
        iq1 = T_QRY_IQ_BUFF[qi];
    }

    di = 0;
    while( di < Calib_pTorqLenTab[i - 1] && IdVal < T_QRY_ID_BUFF[di] )
    {
        ++di;
    }

    if( di > 0 )
    {
        id0 = T_QRY_ID_BUFF[di - 1];
        id1 = T_QRY_ID_BUFF[di];
    }
    else
    {
        id0 = T_QRY_ID_BUFF[di];
        id1 = T_QRY_ID_BUFF[di];
    }


    if( 0 == qi && 0 == di )
        t = 0;
    else
    {
        if( isNM )
        {
            t1 = Calib_NM_pMaxTorqueTab[i - 1];
            t2 = Calib_NM_pMaxTorqueTab[i - 0];
        }
        else
        {
            t1 = Calib_NG_pMaxTorqueTab[i - 1];
            t2 = Calib_NG_pMaxTorqueTab[i - 0];
        }

        tMax = ( t2 - t1 )/( Calib_pSpeedTab[i] - Calib_pSpeedTab[i - 1] )*( f - Calib_pSpeedTab[i - 1] ) + t1;

        if( 0 == qi )
        {
            t1 = 0;
        }
        else if( fabs( iq1 - iq0 ) > 3 )
        {
            t1 = Calib_TorqueDelta/( iq1 - iq0 )*( IqVal - iq0 ) + Calib_TorqueDelta*( qi - 1 );
        }
        else
        {
            t1 = Calib_TorqueDelta*( qi - 1 );
        }

        if( 0 == di )
        {
            t2 = 0;
        }
        else if( fabs( id1 - id0 ) > 3 )
        {
            t2 = Calib_TorqueDelta/( id1 - id0 )*( IdVal - id0 ) + Calib_TorqueDelta*( di - 1 );
        }
        else
        {
            t2 = Calib_TorqueDelta*( di - 1 );
        }

        if( fabs( IdVal ) < 10 || ignoreT2 )
        {
            t = t1;
        }
        else
        {
            t  = 0.5*( t1 + t2 );
        }

        if( t > tMax ) t = tMax;


    }

    return t;
}


float GetTorqueOnSpeedEx( uint32_t spdIdx, uint32_t isNM, float f, float IdVal, float IqVal )
{
    uint32_t di, qi;
    uint32_t i;

    uint32_t ignoreT2 = 0;

    float id0, iq0, id1, iq1;
    float t1, t2, t = 0;
    float tMax;

    i = spdIdx;
    qi = 0;

    if( 0 == i )
    {
        i = 1;
    }

    if( isNM )
    {
        while( qi < Calib_pTorqLenTab[i - 1] && IqVal > T_QRY_IQ_BUFF[qi] )
        {
            ++qi;
        }
    }
    else
    {
        while( qi < Calib_pTorqLenTab[i - 1] && IqVal < T_QRY_IQ_BUFF[qi] )
        {
            ++qi;
        }
    }

    if( qi > 0 )
    {
        iq0 = T_QRY_IQ_BUFF[qi - 1];
        iq1 = T_QRY_IQ_BUFF[qi];
    }
    else
    {
        iq0 = T_QRY_IQ_BUFF[qi];
        iq1 = T_QRY_IQ_BUFF[qi];
    }

    di = 0;
    while( di < Calib_pTorqLenTab[i - 1] && IdVal < T_QRY_ID_BUFF[di] )
    {
        ++di;
    }

    if( di > 0 )
    {
        id0 = T_QRY_ID_BUFF[di - 1];
        id1 = T_QRY_ID_BUFF[di];
    }
    else
    {
        id0 = T_QRY_ID_BUFF[di];
        id1 = T_QRY_ID_BUFF[di];
    }


    if( 0 == qi && 0 == di )
        t = 0;
    else
    {
        if( isNM )
        {
            t1 = Calib_NM_pMaxTorqueTab[i - 1];
            t2 = Calib_NM_pMaxTorqueTab[i - 0];
        }
        else
        {
            t1 = Calib_NG_pMaxTorqueTab[i - 1];
            t2 = Calib_NG_pMaxTorqueTab[i - 0];
        }

        tMax = ( t2 - t1 )/( Calib_pSpeedTab[i] - Calib_pSpeedTab[i - 1] )*( f - Calib_pSpeedTab[i - 1] ) + t1;

        if( 0 == qi )
        {
            t1 = 0;
        }
        else if( fabs( iq1 - iq0 ) > 3 )
        {
            t1 = Calib_TorqueDelta/( iq1 - iq0 )*( IqVal - iq0 ) + Calib_TorqueDelta*( qi - 1 );
        }
        else
        {
            t1 = Calib_TorqueDelta*( qi - 1 );
        }

        if( 0 == di )
        {
            t2 = 0;
        }
        else if( fabs( id1 - id0 ) > 3 )
        {
            t2 = Calib_TorqueDelta/( id1 - id0 )*( IdVal - id0 ) + Calib_TorqueDelta*( di - 1 );
        }
        else
        {
            t2 = Calib_TorqueDelta*( di - 1 );
        }

        if( fabs( IdVal ) < 10 || ignoreT2 )
        {
            t = t1;
        }
        else
        {
            t  = 0.5*( t1 + t2 );
        }

        if( t > tMax ) t = tMax;


    }

    return t;
}

float GetCalibTorqueProc( float f, float IdVal, float IqVal )
{
    float t = 0;
    float fabs = 0;
    uint32_t i = 0;


    if( f < 0 ) fabs = -f;
    else fabs = f;

    i = 0;
    while( i < Calib_SpeedTabLen && Calib_pSpeedTab[i] < fabs )
    {
        ++i;
    }

    if( i >= Calib_SpeedTabLen )
        i = Calib_SpeedTabLen - 1;

    BuildIdqTable( i, fabs, MM_MOTOR );

    if( f <= 0 && IqVal <= T_QRY_IQ_BUFF[0] )
    {
        t = GetTorqueOnSpeed( i, MM_MOTOR, fabs, IdVal, IqVal );
        t = -t;

    }
    else if( f >= 0 && -IqVal <= T_QRY_IQ_BUFF[0] )
    {
        t = GetTorqueOnSpeed( i, MM_MOTOR, fabs, IdVal, -IqVal );
    }
    else
    {
        BuildIdqTable( i, fabs, MM_GENERATOR);
        t = GetTorqueOnSpeed( i, MM_GENERATOR, fabs, IdVal, IqVal );

        if( f > 0 )
            t = -t;
    }

    return t;
}

float GetCalibTorqueProcEx( float f, float IdVal, float IqVal )
{
    float t = 0;
    float fabs = 0;
    uint32_t i = 0;


    if( f < 0 ) fabs = -f;
    else fabs = f;

    i = 0;
    while( i < Calib_SpeedTabLen && Calib_pSpeedTab[i] < fabs )
    {
        ++i;
    }

    if( i >= Calib_SpeedTabLen )
        i = Calib_SpeedTabLen - 1;

    BuildIdqTable( i, fabs, MM_MOTOR );

    if( f <= 0 && -IqVal >= T_QRY_IQ_BUFF[0] )
    {
        t = GetTorqueOnSpeedEx( i, MM_MOTOR, fabs, IdVal, -IqVal );
        t = -t;

    }
    else if( f >= 0 && IqVal >= T_QRY_IQ_BUFF[0] )
    {
        t = GetTorqueOnSpeedEx( i, MM_MOTOR, fabs, IdVal, IqVal );
    }
    else
    {
        BuildIdqTable( i, fabs, MM_GENERATOR);

        if (f < 0)
        {
            t = GetTorqueOnSpeedEx( i, MM_GENERATOR, fabs, IdVal, -IqVal );
        }
        else
        {
            t = GetTorqueOnSpeedEx( i, MM_GENERATOR, fabs, IdVal, IqVal );
        }

        if( f > 0 )
            t = -t;
    }

    return t;
}


float GetCalibTorque( float f, float IdVal, float IqVal )
{
    float t, tL, tH;

    if ( gUDC.uDCBigFilter >= CALIB_RATED_VOLT - 5 )
    {
        InstallCalibParams();
//      t = GetCalibTorqueProc(f, IdVal, IqVal);
        t = GetCalibTorqueProcEx(f, IdVal, IqVal);
    }
    else if ( gUDC.uDCBigFilter <= CALIB_LOW_VOLT + 5 )
    {
        InstallCalibParams_LV();
//      t = GetCalibTorqueProc(f, IdVal, IqVal);
        t = GetCalibTorqueProcEx(f, IdVal, IqVal);
    }
    else
    {
        InstallCalibParams_LV();
//      tL = GetCalibTorqueProc(f, IdVal, IqVal);
        tL = GetCalibTorqueProcEx(f, IdVal, IqVal);
        InstallCalibParams();
//      tH = GetCalibTorqueProc(f, IdVal, IqVal);
        tH = GetCalibTorqueProcEx(f, IdVal, IqVal);
        t = ( tL - tH )/( CALIB_LOW_VOLT - CALIB_RATED_VOLT )*( gUDC.uDCBigFilter - CALIB_RATED_VOLT ) + tH;
    }

    return t;
}

