/*
 * PrmMgr.h
 *
 *  Created on: Mar 4, 2025
 *      Author:
 */

#ifndef BSW_INC_PRMMGR_H_
#define BSW_INC_PRMMGR_H_

typedef long               	int32;

#define MOTORTYPE			0
#define MOTORPOWER			1
#define MOTORVOLTAGE		2
#define MOTORCURRENT		3
#define MOTORFREQUENCY		4
#define MOTORRPM			5
#define MOTORMAXCURRENT		6
#define ROTORTRANSPOLES		7
#define MOTORLD				8
#define MOTORLQ				9
#define MOTORRS				10
#define MOTORBEMF			11
#define MOTORZEROPOSCOMVAL	12
#define MOTORZEROPOSITION	13
#define ASRKPH				14
#define ASRKIH				15
#define ASRKPL				16
#define ASRKIL				17
#define ASRCHGFRQL			18
#define ASRCHGFRQH			19
#define SPEEDFILTER			20
#define ACRKP				21
#define ACRKI				22
#define MAXFEEDBACKCURR		23
#define ACCTORTIME			24
#define DECTORTIME			25
#define MAXFRQ				26
#define UPPERFRQ			27
#define LOWERFRQ			28
#define ACCFRQTIME			29
#define DECFRQTIME			30
#define CARRIERFRQ			31
#define INPUTVOLTON			32
#define INPUTVOLTOFF		33
#define INPUTVOLTOVR		34
#define SPEEDPLIMIT			35
#define SPEEDNLIMIT			36
#define OCPINSTLIMIT		37
#define OCPFILTERLIMIT		38
#define AMBTEMPDERATEBEG	39
#define AMBTEMPDERATEEND	40
#define IGBTTEMPLIMIT		41
#define MOTORTEMPLIMIT		42
#define IGBTTEMPDERATEBEG	43
#define IGBTTEMPDERATEEND	44
#define MOTORTEMPDERATEBEG	45
#define MOTORTEMPDERATEEND	46
#define SAFETYVOLT			47
#define TIMEOUTSEC			48
#define FUNENFLAG			49
#define MOTORNUM			50
#define MOTORPEAKPOWER		51
#define MOTORMAXRPM			52
#define MOTORPOLES			53
#define MOTORMAXTORQUE		54
#define MOTORMAXID			55
#define MOTORMAXFEDBACKCUR	56
#define INVPOWER			57
#define INVVOLTAGE			58
#define INVCURRENT			59
#define INVMAXCURRENT		60
#define FWDSPDLIMIT			61
#define REVSPDLIMIT			62
#define MAXTORQLIMIT		63
#define MINTORQLIMIT		64
#define ROTORZEROESTONLY	65
#define SPDCTRLENABLE		66
#define IACOFFSETLIMIT		67
#define GAINCHGSEL			68
#define CHGSPD				69
#define CHGCURR				70
#define CHGHYSTERVAL		71
#define IDKPH				72
#define IDKIH				73
#define IDKPL				74
#define IDKIL				75
#define IQKPH				76
#define IQKIH				77
#define IQKPL				78
#define IQKIL				79
#define LVUNDERVOL			80
#define LVOVRVOL			81



Uint16  GetPrmEepromAddress(Uint16 TblIndex);
Uint32* GetPrmRamAddress(Uint16 TblIndex);
int32 GetMonitorValue(Uint16 Addr, int32 *Value);
Uint16 GetPrmValue(Uint16 Addr, int32* Value);
Uint16  NvmReadInt32(Uint16 Address, Uint32* u32Data);
void    SetPrmDefaultValue(Uint16 TblIndex);
void NvmReadAllData(void);
Uint16 SetPrmValue(Uint16 Addr, int32 Value);
Uint16 GetTotalFuncNum(void);
void InitAllEepData(void);
void SaveAllPrmDefaultToNvm(void);
void parameter_init(void);
void parameter_update(void);


#endif /* BSW_INC_PRMMGR_H_ */
