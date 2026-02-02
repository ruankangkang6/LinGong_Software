//=====================================================================
//
// ģ����Ҫ���ܣ�Ť�ز���-����
//
//=====================================================================
#include "f_public.h"

TORQUE_REF_COMP_STRUCT g_TorqRefComp = {0, 0, 0, 0, 0, 0};

void CalcCompTorque( void );
void InitTRefComp( void );

uint16_t g_TorqRefCompFlag = 0;
/*************************************************************
    Ť�ز�������
*************************************************************/
void CalcCompTorque( void )
{
    g_TorqRefComp.TRefCompErrSpd    =   gRotorSpeed.SpeedApplyFilter - gRotorSpeed.SpeedApply;

    if( fabs(g_TorqRefComp.TRefCompErrSpd) > 10 )
    {
    	g_TorqRefCompFlag = 1;
        g_TorqRefComp.TRefCompSpdRef += 0.004*g_TorqRefComp.TRefCompSpdRef;
    }
    else
    {
    	g_TorqRefCompFlag = 0;
        g_TorqRefComp.TRefCompSpdRef = gRotorSpeed.SpeedApplyFilter;
    }


    g_TorqRefComp.TRefCompErrSpd =  g_TorqRefComp.TRefCompSpdRef - gRotorSpeed.SpeedApply;

    if( fabs(g_TorqRefComp.TRefCompErrSpd) <= 30 )
    {
        g_TorqRefComp.TRefCompKp     = 2.5f;//1.2f;
        g_TorqRefComp.TRefCompVal    = g_TorqRefComp.TRefCompKp*g_TorqRefComp.TRefCompErrSpd;
        g_TorqRefComp.TRefCompMaxVal = 10;//6;
        g_TorqRefComp.TRefCompMinVal = -10;//-6;
    }
    else
    {
    	g_TorqRefCompFlag = 2;
        g_TorqRefComp.TRefCompKp  = 4.5f;
        if( g_TorqRefComp.TRefCompErrSpd > 30 )
        {
            g_TorqRefComp.TRefCompVal = g_TorqRefComp.TRefCompKp*( g_TorqRefComp.TRefCompErrSpd - 5 );
        }
        else
        {
            g_TorqRefComp.TRefCompVal = g_TorqRefComp.TRefCompKp*( g_TorqRefComp.TRefCompErrSpd + 5 );
        }

        g_TorqRefComp.TRefCompMaxVal = 20;//15;
        g_TorqRefComp.TRefCompMinVal = -20;//-15;

    }

    Saturatefloat(&g_TorqRefComp.TRefCompVal, g_TorqRefComp.TRefCompMaxVal, g_TorqRefComp.TRefCompMinVal);
}


/*************************************************************
    Ť�ز���������ʼ��
*************************************************************/
void InitTRefComp( void )
{
    g_TorqRefComp.TRefCompKp        =   6.0f;   //1.00f;
    g_TorqRefComp.TRefCompVal       =   0;
    g_TorqRefComp.TRefCompMaxVal    =   0;
    g_TorqRefComp.TRefCompMinVal    =   0;
    g_TorqRefComp.TRefCompErrSpd    =   0;
    g_TorqRefComp.TRefCompSpdRef    =   0;
}


