#ifndef __BASIC_H
#define __BASIC_H

#include "DSP2833x_Device.h"   // DSP2833x 头文件
#include "DSP2833x_Examples.h" // DSP2833x 例子相关头文件

// 一个NOP指令大约持续6.67ns（在150MHz下）
#define __NOP()   asm(" NOP")

// 简单的延时函数，通过循环占用CPU，达到延时功能
void delay(void);

#endif /* __BASIC_H */
