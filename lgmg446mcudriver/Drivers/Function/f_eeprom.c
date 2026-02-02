/*
 * f_eeprom.c
 *
 *  Created on: 2024年12月13日
 *      Author: pc
 */
#include "include/f_public.h"

// 一个一个的往EEPROM写数据。
// 目前使用funcCodeOneWriteIndex写功能码的情况有：
// 1. 一般的，menu3OnEnter保存一个功能码(键盘、通讯)
// 2. 当修改最大频率(等等)，以最大频率为限值的功能码也要修改。。。，目前最多为14个。
// 3. 掉电记忆
// 4. 停机记忆
// 5. 清除记录参数
#define FUNCCODE_ONE_WRITE_NUMBER_MAX       200     //+= 100个应该足够，但是仍存在越界的可能。剩余的防止越界
uint16_t funcCodeOneWriteNum = 0;
uint16_t funcCodeOneWriteIndex[FUNCCODE_ONE_WRITE_NUMBER_MAX];

