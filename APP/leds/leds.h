#ifndef _LEDS_H_
#define _LEDS_H_

#include "DSP2833x_Device.h"     // DSP2833x 头文件
#include "DSP2833x_Examples.h"   // DSP2833x 例子相关头文件


#define LED1_OFF		    (GpioDataRegs.GPASET.bit.GPIO23=1)
#define LED1_ON			    (GpioDataRegs.GPACLEAR.bit.GPIO23=1)
#define LED1_TOGGLE		    (GpioDataRegs.GPATOGGLE.bit.GPIO23=1)

#define LED2_OFF		    (GpioDataRegs.GPASET.bit.GPIO22=1)
#define LED2_ON			    (GpioDataRegs.GPACLEAR.bit.GPIO22=1)
#define LED2_TOGGLE		    (GpioDataRegs.GPATOGGLE.bit.GPIO22=1)

#define LED3_OFF		    (GpioDataRegs.GPASET.bit.GPIO21=1)
#define LED3_ON			    (GpioDataRegs.GPACLEAR.bit.GPIO21=1)
#define LED3_TOGGLE		    (GpioDataRegs.GPATOGGLE.bit.GPIO21=1)

#define LED4_OFF		    (GpioDataRegs.GPASET.bit.GPIO20=1)
#define LED4_ON			    (GpioDataRegs.GPACLEAR.bit.GPIO20=1)
#define LED4_TOGGLE		    (GpioDataRegs.GPATOGGLE.bit.GPIO20=1)

#define TEST_GPIO_OFF	    (GpioDataRegs.GPBSET.bit.GPIO63=1)
#define TEST_GPIO_ON	    (GpioDataRegs.GPBCLEAR.bit.GPIO63=1)
#define SOFT_PWM3_TOGGLE    (GpioDataRegs.GPBTOGGLE.bit.GPIO63=1)

#define SOFT_PWM1_TOGGLE    (GpioDataRegs.GPATOGGLE.bit.GPIO24=1)
#define SOFT_PWM2_TOGGLE    (GpioDataRegs.GPATOGGLE.bit.GPIO25=1)

void LED_Init();

#endif /* LED_H_ */
