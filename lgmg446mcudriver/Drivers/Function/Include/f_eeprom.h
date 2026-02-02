/*
 * f_eeprom.h
 *
 *  Created on: 2024年12月13日
 *      Author: pc
 */

#ifndef MCU_CONTROL_REV2_SOURCE_FUNCTION_INCLUDE_F_EEPROM_H_
#define MCU_CONTROL_REV2_SOURCE_FUNCTION_INCLUDE_F_EEPROM_H_

#define SaveOneFuncCode(index)  (funcCodeOneWriteIndex[funcCodeOneWriteNum++] = (index))
//#define GetCodeIndex(code)    ((uint16_t)((&(code)) - (&(funcCode.all[0]))))


extern uint16_t funcCodeOneWriteNum;
extern uint16_t funcCodeOneWriteIndex[];

#endif /* MCU_CONTROL_REV2_SOURCE_FUNCTION_INCLUDE_F_EEPROM_H_ */
