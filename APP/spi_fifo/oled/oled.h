#ifndef __OLED_H
#define __OLED_H

#include "DSP2833x_Device.h"   // DSP2833x 头文件
#include "DSP2833x_Examples.h" // DSP2833x 例子相关头文件

#define OLED_RES_LOW (GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1)
#define OLED_RES_HIGH (GpioDataRegs.GPBSET.bit.GPIO52 = 1)

// #define OLED_CLK_LOW (GpioDataRegs.GPBCLEAR.bit.GPIO56 = 1)
// #define OLED_CLK_HIGH (GpioDataRegs.GPBSET.bit.GPIO56 = 1)

// #define OLED_MOSI_LOW (GpioDataRegs.GPBCLEAR.bit.GPIO54 = 1)
// #define OLED_MOSI_HIGH (GpioDataRegs.GPBSET.bit.GPIO54 = 1)

#define OLED_DC_CMD (GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1)
#define OLED_DC_DATA (GpioDataRegs.GPBSET.bit.GPIO53 = 1)

void OLED_Init(void);
void OLED_Filltest(Uint16 data);
void Picture_display(const Uint16 *ptr_pic, Uint16 colStart, Uint16 pageStart, Uint16 line, Uint16 col);
void PictureContrastDisplay(const Uint16 *ptr_pic, Uint16 colStart, Uint16 pageStart, Uint16 line, Uint16 col);


#endif /* __OLED_H */
