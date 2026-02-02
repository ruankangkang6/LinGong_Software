#ifndef __FBL_CFG_H__
#define __FBL_CFG_H__

#define FBL_UPGRADE_FLAG  (*(volatile U32 *)(0x2001FFF0))
#define APP_RUNNING_FLAG  (*(volatile U32 *)(0x2001FFF4))
#define APP_CAN_BAUD_RATE (*(volatile U32 *)(0x2001FFF8))
#define ECU_CAN_ADDRESS   (*(volatile U32 *)(0x2001FFFC))

#define BOOT_ADDR   (0x08000000)

#define APP_A_ADDR      (0x08010800)
#define APPA_MSG_ADD    (0x08010000)
#define APP_B_ADDR      (0x08010800)
#define APPB_MSG_ADD    (0x08010000)
#define APP_FLASH_SIZE  (0x80000 - 0x10000 - 0x800)

#define FBL_MSG_FLG     (0xAA55A500)
#define APPA_RUN_FLG    (0xAA55A541)
#define APPB_RUN_FLG    (0xAA55A542)
#define FBL_UPG_FLG     (0x55AAA5A0)

#define FBL_TO_APP_DELAY		(50u)
#endif
