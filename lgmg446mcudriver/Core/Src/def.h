#ifndef __DEF_H__
#define __DEF_H__

#ifndef NULL
#define NULL  0
#endif
#define FALSE 0
#define TRUE  1

typedef unsigned char  UINT8;
typedef unsigned short UINT16;  
typedef unsigned long  UINT32;
typedef char  INT8;
typedef short INT16;
typedef long  INT32;	

typedef char  byte;
typedef short word;
typedef unsigned long ulong;

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;  
typedef unsigned long long uint64;
//typedef char int8;
typedef short int16;
typedef long int32;

typedef unsigned char U08;
typedef unsigned short U16;
typedef unsigned long U32;
typedef char S08;
typedef short S16;
typedef long S32;

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;

typedef unsigned char u8;
typedef unsigned short u16;


typedef void (*pFUN)(void);	

typedef unsigned char bool;
typedef unsigned char BOOL;


#ifndef __always_inline
#define __always_inline         __forceinline static 
#endif

#define PLATFORM_CODE		(4)
#define VERSION_TYPE		(34)
#define PUBLISH_VER			(21u)
#define DEBUG_VER			(1u)

#endif
