#ifndef __AD760X_H
#define __AD760X_H

#include "DSP2833x_Device.h"     // DSP2833x 头文件
#include "DSP2833x_Examples.h"   // DSP2833x 例子相关头文件

#define AD760x_RESET_HIGH	    (GpioDataRegs.GPBSET.bit.GPIO42=1)
#define AD760x_RESET_LOW		(GpioDataRegs.GPBCLEAR.bit.GPIO42=1)
#define AD760x_RESET_TOGGLE		(GpioDataRegs.GPBTOGGLE.bit.GPIO42=1)

#define AD760x_CONVSTA_B_HIGH	    (GpioDataRegs.GPBSET.bit.GPIO34=1)
#define AD760x_CONVSTA_B_LOW		(GpioDataRegs.GPBCLEAR.bit.GPIO34=1)
#define AD760x_CONVSTA_B_TOGGLE		(GpioDataRegs.GPBTOGGLE.bit.GPIO34=1)

#define AD760x_DIR_AD_2_DSP	    (GpioDataRegs.GPBSET.bit.GPIO43=1)
#define AD760x_DIR_DSP_2_AD		(GpioDataRegs.GPBCLEAR.bit.GPIO43=1)
#define AD760x_DIR_TOGGLE		(GpioDataRegs.GPBTOGGLE.bit.GPIO43=1)

enum AD760x_CS_ADDRESS_CHOOSE
{
    CHOOSE_ZONE0_ADDR = 0,
    CHOOSE_ZONE6_ADDR = 1,
    CHOOSE_ZONE7_ADDR = 2
};

#define AD7606C_ID_VERSION 0x2F  // 输出为0x2F22
#define AD7606C_EXTI_CONFIG_MODE 0x00 // 写入这个地址退出配置寄存器的模式，开始进行数模转换
#define AD7606C_BANDWIDTH 0x07   // 高带宽模式，默认为0x00全关闭，需要开启哪个通道就把对应位设置为1，bit0-bit7分别对应CH1-CH8
#define AD7606C_OVERSAMPLING 0x08   // 过采样模式
// 0000: ±2.5 V single-ended range.
// 0001: ±5 V single-ended range.
// 0010: ±6.25 V single-ended range.
// 0011: ±10 V single-ended range.
// 0100: ±12.5 V single-ended range.
// 0101: 0 V to 5 V single-ended range.
// 0110: 0 V to 10 V single-ended range.
// 0111: 0 V to 12.5 V single-ended range.
// 1000: ±5 V differential range.
// 1001: ±10 V differential range.
// 1010: ±12.5 V differential range.
// 1011: ±20 V differential range.
#define AD7606C_RANGE_CH1_CH2 0x03 // CH1-CH2电压范围设置，默认±10V单端输入，需要改成什么样见上面
#define AD7606C_RANGE_CH3_CH4 0x04 // CH3-CH4电压范围设置，默认±10V单端输入，需要改成什么样见上面
#define AD7606C_RANGE_CH5_CH6 0x05 // CH5-CH6电压范围设置，默认±10V单端输入，需要改成什么样见上面
#define AD7606C_RANGE_CH7_CH8 0x06 // CH7-CH8电压范围设置，默认±10V单端输入，需要改成什么样见上面
#define AD7606C_CH1_PHASE 0x19 // CH1相位设置，默认0x00，每加一延迟1us进行采样
#define AD7606C_CH2_PHASE 0x1A // CH2相位设置，默认0x00，每加一延迟1us进行采样
#define AD7606C_CH3_PHASE 0x1B // CH3相位设置，默认0x00，每加一延迟1us进行采样
#define AD7606C_CH4_PHASE 0x1C // CH4相位设置，默认0x00，每加一延迟1us进行采样
#define AD7606C_CH5_PHASE 0x1D // CH5相位设置，默认0x00，每加一延迟1us进行采样
#define AD7606C_CH6_PHASE 0x1E // CH6相位设置，默认0x00，每加一延迟1us进行采样
#define AD7606C_CH7_PHASE 0x1F // CH7相位设置，默认0x00，每加一延迟1us进行采样
#define AD7606C_CH8_PHASE 0x20 // CH8相位设置，默认0x00，每加一延迟1us进行采样

void StartAD760x(void);
void AD760x_Init(void);
void AD760x_GPIO_Protect(char value);
void AD760x_Write_Register(char reg, Uint16 addr, Uint16 data);
Uint16 AD760x_Read_Register(char reg, Uint16 addr);
void AD760x_Regester_Init(char reg);
#endif /* DAAAE9F4_816F_4EB3_808D_EE8EDB88923A */
