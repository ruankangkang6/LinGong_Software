/*
 * Monitor.c
 *
 *  Created on: Feb 28, 2025
 *      Author:
 */
#include "main.h"


int xy=10;
Uint16 x2=0;
double x3 = 0.0;

const StructMonitorInfo DefPower_Module_Fault_G=
{
 0x3000,                                   /* CAN address */
 {
  0,                                    /* Access level */
  0,                                    /* Read-Write attr */
  0,                                    /* Reset */
  1,                                    /* Sign */
  0,                                    /* Decimal num */
  1,                                    /* Reboot */
  1,                                    /* Rev */
 },
 &(xy),
 &(Count100ms),/* RAM */
};



const StructMonitorInfo* MonitorTbl[] =
{
         &DefPower_Module_Fault_G,
};
