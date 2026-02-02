/*
 * MonitorPrm.h
 *
 *  Created on: Feb 27, 2025
 *      Author:
 */

#ifndef BSW_INC_MONITORPRM_H_
#define BSW_INC_MONITORPRM_H_

typedef uint16_t       Uint16;
typedef uint32_t      Uint32;

typedef struct
{
    Uint16  AccessLevel                      :3;    /* 访问级别 0 客户 1 FAE 2 SE 3 AE 4 超级用户 */
       Uint16  ReadWriteAttr                    :2;    /* 读写属性 0 只读 1 只写 2 可读写 */
       Uint16  fixpoint                    :2;    /* 0 1 1 15定标  2 24定标*/
      Uint16  Sign                             :1;    /* 0 有符号  1 无符号 */
       Uint16  DecimalNum                       :3;    /* 小数位数 0 无小数 1~7位小数 */
       Uint16  NeedReboot                       :1;    /* 生效方式  0 不需要重启  1 要重启 */
       Uint16  Rev1;                                    /*类型 0 int 1 Uint16 2 int16*/
} StructPrmAttribute;

typedef struct
{
    Uint16 CanAddr;
    Uint16 EepromAddr;
    StructPrmAttribute  Attr;
    int  *RamPtr;
    Uint16  *RamPtr2;
    Uint32  *RamPtr3;
   double  DefaultValue;
//    int32  UpperLimit;
//    int32  LowerLimit;
//    Uint32 (*FuncPtr)(Uint32 Cmd, Uint32 Data, Uint32 Digit);
} StructPrmInfo;


//#define  TOTAL_FUNC_NUM  11    /* All Funcode Num */
//#define  TOTAL_FUNC_GROUP  1   /* All Group Num */


extern const  StructPrmInfo* PnPrmTbl[];
extern const  Uint16 CanAddrOffset[];




#endif /* BSW_INC_MONITORPRM_H_ */
