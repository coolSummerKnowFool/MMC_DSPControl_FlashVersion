#include "ad760x.h"
#include "exti.h"
#include "dma_xintf.h"

#pragma CODE_SECTION(StartAD760x, ".TI.ramfunc")

void AD760x_GPIO_Init(void)
{
    EALLOW;                                  // 关闭写保护
    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1; // 开启GPIO时钟

    // ADREST 端口配置
    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0; // 设置为通用GPIO功能
    GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;  // 方向设置为1时输出，设置为0时输入
    GpioCtrlRegs.GPBPUD.bit.GPIO42 = 1;  // 不使能GPIO上拉电阻

    // CONVSTA/B 端口配置
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;

    // AD_DIR 端口配置
    // DIR为0时，(B→A)，方向为从DSP到AD7606
    // DIR为1时，(A→B)，方向为从AD7606到DSP
    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO43 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO43 = 1;

    GpioDataRegs.GPBCLEAR.bit.GPIO42 = 1;
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    GpioDataRegs.GPBSET.bit.GPIO43 = 1;

    // ADRD_OE 端口配置
    // 先默认关闭信号二极管
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0; // 设置为通用GPIO功能
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = 1;  // 方向设置为1时输出，设置为0时输入
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 1;  // 不使能GPIO上拉电阻
    GpioDataRegs.GPBSET.bit.GPIO40 = 1; // ADRD_OE为1时关断信号二极管

    // AD_START_RD 端口配置
    // 默认配置为输入引脚
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0; // 设置为通用GPIO功能
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1;  // 不使能GPIO上拉电阻
    GpioCtrlRegs.GPBDIR.bit.GPIO41 = 0;  // 方向设置为1时输出，设置为0时输入
    GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 0;

    EDIS; // 开启写保护
}

// ADRD_OE为1时关断信号二极管，为0时导通信号二极管
void AD760x_GPIO_Protect(char value)
{
    EALLOW;
    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1; // 开启GPIO时钟
    // 保护
    if (value)
    {
        GpioCtrlRegs.GPBDIR.bit.GPIO41 = 1;  // 方向设置为1时输出，设置为0时输入
    }
    // 导通
    else
    {
        GpioCtrlRegs.GPBDIR.bit.GPIO41 = 0;  // 方向设置为1时输出，设置为0时输入
        GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 0;
        GpioDataRegs.GPBCLEAR.bit.GPIO40 = 1; // ADRD_OE为0时导通信号二极管
    }
    EDIS;
}

void AD760x_Write_Register(char reg, Uint16 addr, Uint16 data)
{
    Uint16 cmd = ((addr & 0x7F) << 8) | (data & 0xFF);
    AD760x_DIR_DSP_2_AD;
    DELAY_US(0.2);
    switch (reg)
    {
    case CHOOSE_ZONE0_ADDR:
        *AD7606_ZONE0_ADDR = cmd;
        break;
    case CHOOSE_ZONE6_ADDR:
        *AD7606_ZONE6_ADDR = cmd;
        break;
    case CHOOSE_ZONE7_ADDR:
        *AD7606_ZONE7_ADDR = cmd;
        break;
    default:
        break;
    }
    DELAY_US(0.2);
    AD760x_DIR_AD_2_DSP;
    DELAY_US(0.2);
}

Uint16 AD760x_Read_Register(char reg, Uint16 addr)
{
    Uint16 value = 0;
    Uint16 cmd = ((addr & 0x7F) | 0x80) << 8; // 读命令
    switch (reg)
    {
    case CHOOSE_ZONE0_ADDR:
        AD760x_DIR_DSP_2_AD;
        DELAY_US(0.2);
        *AD7606_ZONE0_ADDR = cmd;
        DELAY_US(0.2);
        AD760x_DIR_AD_2_DSP;
        DELAY_US(0.2);
        value = *AD7606_ZONE0_ADDR;
        DELAY_US(0.2);
        break;
    case CHOOSE_ZONE6_ADDR:
        AD760x_DIR_DSP_2_AD;
        DELAY_US(0.2);
        *AD7606_ZONE6_ADDR = cmd;
        DELAY_US(0.2);
        AD760x_DIR_AD_2_DSP;
        DELAY_US(0.2);
        value = *AD7606_ZONE6_ADDR;
        DELAY_US(0.2);
        break;
    case CHOOSE_ZONE7_ADDR:
        AD760x_DIR_DSP_2_AD;
        DELAY_US(0.2);
        *AD7606_ZONE7_ADDR = cmd;
        DELAY_US(0.2);
        AD760x_DIR_AD_2_DSP;
        DELAY_US(0.2);
        value = *AD7606_ZONE7_ADDR;
        DELAY_US(0.2);
        break;
    default:
        break;
    }
    return value;
}

void AD760x_Regester_Init(char reg)
{
    AD760x_Read_Register(reg, 0x2F);  // 读取任意寄存器进入配置模式
    AD760x_Write_Register(reg, AD7606C_RANGE_CH1_CH2, 0x88);
    AD760x_Write_Register(reg, AD7606C_RANGE_CH3_CH4, 0x88);
    AD760x_Write_Register(reg, AD7606C_RANGE_CH5_CH6, 0x88);
    AD760x_Write_Register(reg, AD7606C_RANGE_CH7_CH8, 0x88);
    AD760x_Write_Register(reg, AD7606C_OVERSAMPLING, 0x00); // 过采样1倍
    AD760x_Write_Register(reg, AD7606C_CH1_PHASE, 0x04);  // 延迟5us进行采样
    AD760x_Write_Register(reg, AD7606C_CH2_PHASE, 0x04);
    AD760x_Write_Register(reg, AD7606C_CH3_PHASE, 0x04);
    AD760x_Write_Register(reg, AD7606C_CH4_PHASE, 0x04);
    AD760x_Write_Register(reg, AD7606C_CH5_PHASE, 0x04);
    AD760x_Write_Register(reg, AD7606C_CH6_PHASE, 0x04);
    AD760x_Write_Register(reg, AD7606C_CH7_PHASE, 0x04);
    AD760x_Write_Register(reg, AD7606C_CH8_PHASE, 0x04);
    AD760x_Write_Register(reg, AD7606C_EXTI_CONFIG_MODE, 0x00); // 退出配置模式，开始进行数模转换

    // switch (reg)
    // {
    // case CHOOSE_ZONE0_ADDR:
    //     AD760x_Read_Register(CHOOSE_ZONE0_ADDR, 0x2F);  // 读取任意寄存器进入配置模式
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_RANGE_CH1_CH2, 0x88);
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_RANGE_CH3_CH4, 0x88);
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_RANGE_CH5_CH6, 0x88);
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_RANGE_CH7_CH8, 0x88);
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_CH1_PHASE, 0x02);  // 延迟2us进行采样
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_CH2_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_CH3_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_CH4_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_CH5_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_CH6_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_CH7_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_CH8_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE0_ADDR, AD7606C_EXTI_CONFIG_MODE, 0x00); // 退出配置模式，开始进行数模转换
    //     break;
    // case CHOOSE_ZONE6_ADDR:
    //     AD760x_Read_Register(CHOOSE_ZONE6_ADDR, 0x2F);  // 读取任意寄存器进入配置模式
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_RANGE_CH1_CH2, 0x88);
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_RANGE_CH3_CH4, 0x88);
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_RANGE_CH5_CH6, 0x88);
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_RANGE_CH7_CH8, 0x88);
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_CH1_PHASE, 0x02);  // 延迟2us进行采样
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_CH2_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_CH3_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_CH4_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_CH5_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_CH6_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_CH7_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_CH8_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE6_ADDR, AD7606C_EXTI_CONFIG_MODE, 0x00); // 退出配置模式，开始进行数模转换
    //     break;
    // case CHOOSE_ZONE7_ADDR:
    //     AD760x_Read_Register(CHOOSE_ZONE7_ADDR, 0x2F);  // 读取任意寄存器进入配置模式
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_RANGE_CH1_CH2, 0x88);
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_RANGE_CH3_CH4, 0x88);
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_RANGE_CH5_CH6, 0x88);
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_RANGE_CH7_CH8, 0x88);
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_CH1_PHASE, 0x02);  // 延迟2us进行采样
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_CH2_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_CH3_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_CH4_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_CH5_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_CH6_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_CH7_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_CH8_PHASE, 0x02);
    //     AD760x_Write_Register(CHOOSE_ZONE7_ADDR, AD7606C_EXTI_CONFIG_MODE, 0x00); // 退出配置模式，开始进行数模转换
    //     break;
    // default:
    //     break;
    // }
}

void StartAD760x(void)
{
    // RESET信号至少拉高50ns完成复位
    AD760x_RESET_HIGH;
    DELAY_US(0.2);
    AD760x_RESET_LOW;
    DELAY_US(0.2);

    // 检测到CONVSTA/CONVSTB上升沿后每个通道开始进行数模转换
    AD760x_CONVSTA_B_LOW;
    DELAY_US(0.2);
    AD760x_CONVSTA_B_HIGH;
}

void AD760x_Init(void)
{
    AD760x_GPIO_Init();
    AD760x_GPIO_Protect(0);
    // SetOSModel(OS_NO_MODEL);
    EXTI1_Init();
    EXTI2_Init();
    EXTI4_Init();

    //完成DMA与XINTF的配置
    Init_DMA_AD7606();

#ifdef AD760x_SOFTWARE
    AD760x_Software_GPIO_Init();
#endif
    
}
