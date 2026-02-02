/*
 * Monitor.h
 *
 *  Created on: Feb 28, 2025
 *      Author:
 */

#ifndef BSW_INC_MONITOR_H_
#define BSW_INC_MONITOR_H_


//#define  TOTAL_MONITOR_NUM  68   /* All Funcode Num */
//#define  TOTAL_MONITOR_GROUP  3    /* All Group Num */


typedef struct
{
    Uint16  AccessLevel                      :3;    /* 访问级别 0 客户 1 FAE 2 SE 3 AE 4 超级用户 */
    Uint16  ReadWriteAttr                    :2;    /* 读写属性 0 只读 1 只写 2 可读写 */
    Uint16  fixpoint                    :2;    /* 0 1 1 15定标  2 24定标*/
   Uint16  Sign                             :1;    /* 0 有符号  1 无符号 */
    Uint16  DecimalNum                       :3;    /* 小数位数 0 无小数 1~7位小数 */
    Uint16  NeedReboot                       :1;    /* 生效方式  0 不需要重启  1 要重启 */
    Uint16  Rev1;                                    /*类型 0 int32 1 Uint16 2 int16*/
} StructMonitorAttribute;

typedef struct
{
   Uint16 CanAddr;
    StructMonitorAttribute  Attr;
    int  *RamPtr;
    Uint16  *RamPtr2;
    double  *RamPtr3;
    volatile Uint32  *RamPtr4;
    Uint32 (*FuncPtr)(Uint32 Cmd, Uint32 Data, Uint32 Digit);
} StructMonitorInfo;

#define INT16_FROM_2BYTE( Byte0, Byte1 )        ((int16)((Uint16)(Byte0) + ((Uint16)(Byte1) << 8)))
#define UINT16_FROM_2BYTE( Byte0, Byte1 )   ((Uint16)((Uint16)(Byte0) + ((Uint16)(Byte1) << 8)))
#define INT32_FROM_4BYTE( Byte0, Byte1, Byte2, Byte3 )      ((int32)((Uint16)(Byte0) + ((Uint16)(Byte1) << 8) + ((Uint32)(Byte2) << 16) + ((Uint32)(Byte3) << 24)))
#define UINT32_FROM_4BYTE( Byte0, Byte1, Byte2, Byte3 )     ((Uint32)((Uint16)(Byte0) + ((Uint16)(Byte1) << 8) + ((Uint32)(Byte2) << 16) + ((Uint32)(Byte3) << 24)))



#endif /* BSW_INC_MONITOR_H_ */
