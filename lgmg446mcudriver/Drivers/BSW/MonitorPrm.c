/*
 * MonitorPrm.c
 *
 *  Created on: Feb 27, 2025
 *      Author: wangc
 */


#include "main.h"

int Rev=0;
int32 Rev01=1;
int32 Rev02=2;
int32 Rev03=3;
int32 Rev04=4;
Uint16 Rev1=0;
double Rev2=0;

const StructPrmInfo PnDefMotorType =
{
    0x0010,                                   /* CAN address */
    0x0010,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorType),                            /* Prm */
    &(Rev1),
    &(Rev2),
   1,                                        /* default */
};
const StructPrmInfo PnDefMotorPower =
{
    0x0011,                                   /* CAN address */
    0x0011,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorPower),                            /* Prm */
    &(Rev1),
    &(Rev2),
   30,                                        /* default */
};
const StructPrmInfo PnDefMotorVotage =
{
    0x0012,                                   /* CAN address */
    0x0012,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorVotage),                            /* Prm */
    &(Rev1),
    &(Rev2),
   80,                                        /* default */
};
const StructPrmInfo PnDefMotorCurrent =
{
    0x0013,                                   /* CAN address */
    0x0013,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorCurrent),                            /* Prm */
    &(Rev1),
    &(Rev2),
   800,                                        /* default */
};
const StructPrmInfo PnDefMotorFrequency =
{
    0x0014,                                   /* CAN address */
    0x0014,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorFrequency),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMotorRpm =
{
    0x0015,                                   /* CAN address */
    0x0015,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorRpm),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMotorMaxCurrent =
{
    0x0016,                                   /* CAN address */
    0x0016,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorMaxCurrent),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefRotorTransPoles =
{
    0x0017,                                   /* CAN address */
    0x0017,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(RotorTransPoles),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMotorLd =
{
    0x0018,                                   /* CAN address */
    0x0018,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorLd),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMotorLq =
{
    0x0019,                                   /* CAN address */
    0x0019,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorLq),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMotorRs =
{
    0x001A,                                   /* CAN address */
    0x001A,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorRs),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMotorBemf =
{
    0x001B,                                   /* CAN address */
    0x001B,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorBemf),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMotorZeroPosCompVal =
{
    0x001C,                                   /* CAN address */
    0x001C,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorZeroPosCompVal),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMotorZeroPosition =
{
    0x001D,                                   /* CAN address */
    0x001D,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorZeroPosition),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefAsrKpH =
{
    0x001E,                                   /* CAN address */
    0x001E,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(AsrKpH),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefAsrKiH =
{
    0x001F,                                   /* CAN address */
    0x001F,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(AsrKiH),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefAsrKpL =
{
    0x0020,                                   /* CAN address */
    0x0020,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(AsrKpL),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefAsrKiL =
{
    0x0021,                                   /* CAN address */
    0x0021,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(AsrKiL),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefAsrChgFrqL =
{
    0x0022,                                   /* CAN address */
    0x0022,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(AsrChgFrqL),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefAsrChgFrqH =
{
    0x0023,                                   /* CAN address */
    0x0023,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(AsrChgFrqH),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefSpeedFilter =
{
    0x0024,                                   /* CAN address */
    0x0024,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(SpeedFilter),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefAcrKp =
{
    0x0025,                                   /* CAN address */
    0x0025,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(AcrKp),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};

const StructPrmInfo PnDefAcrKi =
{
    0x0026,                                   /* CAN address */
    0x0026,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(AcrKi),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMaxFeedbackCurr =
{
    0x0027,                                   /* CAN address */
    0x0027,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MaxFeedbackCurr),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefAccTorTime =
{
    0x0028,                                   /* CAN address */
    0x0028,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(AccTorTime),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefDecTorTime =
{
    0x0029,                                   /* CAN address */
    0x0029,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(DecTorTime),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMaxFrq =
{
    0x002A,                                   /* CAN address */
    0x002A,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MaxFrq),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefUpperFrq =
{
    0x002B,                                   /* CAN address */
    0x002B,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(UpperFrq),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefLowerFrq =
{
    0x002C,                                   /* CAN address */
    0x002C,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(LowerFrq),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};

const StructPrmInfo PnDefAccFrqTime =
{
    0x002D,                                   /* CAN address */
    0x002D,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(AccFrqTime),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefDecFrqTime =
{
    0x002E,                                   /* CAN address */
    0x002E,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(DecFrqTime),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefCarrierFrq =
{
    0x002F,                                   /* CAN address */
    0x002F,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(CarrierFrq),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefInputVoltOn =
{
    0x0030,                                   /* CAN address */
    0x0030,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(InputVoltOn),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefInputVoltOff =
{
    0x0031,                                   /* CAN address */
    0x0031,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(InputVoltOff),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefInputVoltOvr =
{
    0x0032,                                   /* CAN address */
    0x0032,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(InputVoltOvr),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefSpeedPLimit =
{
    0x0033,                                   /* CAN address */
    0x0033,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(SpeedPLimit),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefSpeedNLimit =
{
    0x0034,                                   /* CAN address */
    0x0034,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(SpeedNLimit),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefOcpInstLimit =
{
    0x0035,                                   /* CAN address */
    0x0035,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(OcpInstLimit),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefOcpFilterLimit =
{
    0x0036,                                   /* CAN address */
    0x0036,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(OcpFilterLimit),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefAMBTempDerateBeg =
{
    0x0037,                                   /* CAN address */
    0x0037,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(AMBTempDerateBeg),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefAMBTempDerateEnd =
{
    0x0038,                                   /* CAN address */
    0x0038,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(AMBTempDerateEnd),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefIGBTTempLimit =
{
    0x0038,                                   /* CAN address */
    0x0038,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(IGBTTempLimit),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMotorTempLimit =
{
    0x0039,                                   /* CAN address */
    0x0039,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorTempLimit),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefIGBTTempDerateBeg =
{
    0x003A,                                   /* CAN address */
    0x003A,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(IGBTTempDerateBeg),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefIGBTTempDerateEnd =
{
    0x003B,                                   /* CAN address */
    0x003B,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(IGBTTempDerateEnd),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMotorTempDerateBeg =
{
    0x003C,                                   /* CAN address */
    0x003C,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorTempDerateBeg),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefMotorTempDerateEnd =
{
    0x003D,                                   /* CAN address */
    0x003D,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(MotorTempDerateEnd),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefSafetyVolt =
{
    0x003E,                                   /* CAN address */
    0x003E,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(SafetyVolt),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefTimeoutSec =
{
    0x003F,                                   /* CAN address */
    0x003F,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(TimeoutSec),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo PnDefFunEnFlag =
{
    0x0040,                                   /* CAN address */
    0x0040,                                       /* EEPROM address */
    {
        0,                                    /* Access level */
        2,                                    /* Read-Write attr */
        0,                                    /* Reset */
        0,                                    /* Sign */
        0,                                    /* Decimal num */
        0,                                    /* Reboot */
        0,                                    /* Rev */
    },
    &(FunEnFlag),                            /* Prm */
    &(Rev1),
    &(Rev2),
   300,                                        /* default */
};
const StructPrmInfo* PnPrmTbl[] =
{
		&PnDefMotorType,
		&PnDefMotorPower,
		&PnDefMotorVotage,
		&PnDefMotorCurrent,
		&PnDefMotorFrequency,
		&PnDefMotorRpm,
		&PnDefMotorMaxCurrent,
		&PnDefRotorTransPoles,
		&PnDefMotorLd,
		&PnDefMotorLq,
		&PnDefMotorRs,
		&PnDefMotorBemf,
		&PnDefMotorZeroPosCompVal,
		&PnDefMotorZeroPosition,
		&PnDefAsrKpH,
		&PnDefAsrKiH,
		&PnDefAsrKpL,
		&PnDefAsrKiL,
		&PnDefAsrChgFrqL,
		&PnDefAsrChgFrqH,
		&PnDefSpeedFilter,
		&PnDefAcrKp,
		&PnDefAcrKi,
		&PnDefMaxFeedbackCurr,
		&PnDefAccTorTime,
		&PnDefDecTorTime,
		&PnDefMaxFrq,
		&PnDefUpperFrq,
		&PnDefLowerFrq,
		&PnDefAccFrqTime,
		&PnDefDecFrqTime,
		&PnDefCarrierFrq,
		&PnDefInputVoltOn,
		&PnDefInputVoltOff,
		&PnDefInputVoltOvr,
		&PnDefSpeedPLimit,
		&PnDefSpeedNLimit,
		&PnDefOcpInstLimit,
		&PnDefOcpFilterLimit,
		&PnDefAMBTempDerateBeg,
		&PnDefAMBTempDerateEnd,
		&PnDefIGBTTempLimit,
		&PnDefMotorTempLimit,
		&PnDefIGBTTempDerateBeg,
		&PnDefIGBTTempDerateEnd,
		&PnDefMotorTempDerateBeg,
		&PnDefMotorTempDerateEnd,
		&PnDefSafetyVolt,
		&PnDefTimeoutSec,
		&PnDefFunEnFlag,
};

const Uint16 CanAddrOffset[] =
{
    0,                                        /* Pn00xx */
//    9,                                       /* Pn01xx */
};

