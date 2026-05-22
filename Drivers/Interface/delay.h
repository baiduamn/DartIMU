#ifndef _DELAY_H
#define _DELAY_H

#include "main.h"  // 包含 HAL 库头文件
//#include <sys.h>
void delay_init(uint8_t SYSCLK);           // 初始化（可选）
void delay_us(uint32_t nus);     // 微秒延时

#endif

