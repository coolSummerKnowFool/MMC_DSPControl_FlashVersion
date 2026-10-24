#ifndef __TIM_H
#define __TIM_H

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

void TIM0_Init(float Freq, float Period);
void TIM0_Start(void);
void TIM0_Stop(void);
interrupt void TIM0_IRQn(void);

void TIM1_Init(float Freq, float Period);
interrupt void TIM1_IRQn(void);

void TIM2_Init(float Freq, float Period);
interrupt void TIM2_IRQn(void);

#endif /* __TIM_H */
