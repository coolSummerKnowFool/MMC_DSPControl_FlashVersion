#ifndef __EXIT_H
#define __EXIT_H

#include "DSP2833x_Device.h"   // DSP2833x 头文件
#include "DSP2833x_Examples.h" // DSP2833x 例子相关头文件

#define KEY_A0_STATUS (GpioDataRegs.GPBDAT.bit.GPIO49)
#define KEY_A1_STATUS (GpioDataRegs.GPBDAT.bit.GPIO50)
#define KEY_A2_STATUS (GpioDataRegs.GPBDAT.bit.GPIO51)

enum KEY_PRESS_NUM{
    KEY_STATUS_DOWN,
    KEY_STATUS_CENTER,
    KEY_STATUS_RIGHT,
    KEY_STATUS_LEFT,
    KEY_STATUS_UP
};

void EXTI1_Init(void);
interrupt void EXTI1_IRQn(void);

void EXTI2_Init(void);
interrupt void EXTI2_IRQn(void);

void EXTI4_Init(void);
interrupt void EXTI4_IRQn(void);

void KEY_Init(void);
void EXTI3_Init(void);
interrupt void EXTI3_IRQn(void);

#endif /* __EXIT_H */
