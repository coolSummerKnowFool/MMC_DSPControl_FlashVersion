#include "leds.h"

/*******************************************************************************
 * 函 数 名         : LED_Init
 * 函数功能         : LED初始化函数
 * 输    入         : 无
 * 输    出         : 无
 *******************************************************************************/
void LED_Init(void)
{
	EALLOW; // 关闭写保护
	SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;// 开启GPIO时钟

	//LED1端口配置
	GpioCtrlRegs.GPAMUX2.bit.GPIO23=0;
	GpioCtrlRegs.GPADIR.bit.GPIO23=1;
	GpioCtrlRegs.GPAPUD.bit.GPIO23=0;
	
	//LED2端口配置
	GpioCtrlRegs.GPAMUX2.bit.GPIO22=0;
	GpioCtrlRegs.GPADIR.bit.GPIO22=1;
	GpioCtrlRegs.GPAPUD.bit.GPIO22=0;
	
	//LED3端口配置
	GpioCtrlRegs.GPAMUX2.bit.GPIO21=0;
	GpioCtrlRegs.GPADIR.bit.GPIO21=1;
	GpioCtrlRegs.GPAPUD.bit.GPIO21=0;
	
	//LED4端口配置
	GpioCtrlRegs.GPAMUX2.bit.GPIO20=0;//设置为通用GPIO功能
	// 方向设置为1时输出，设置为0时输入
	GpioCtrlRegs.GPADIR.bit.GPIO20=1;//设置GPIO方向为输出
	GpioCtrlRegs.GPAPUD.bit.GPIO20=0;//使能GPIO上拉电阻(此时引脚默认为高电平)
	
	GpioCtrlRegs.GPBMUX2.bit.GPIO63=0;
	GpioCtrlRegs.GPBDIR.bit.GPIO63=1;
	GpioCtrlRegs.GPBPUD.bit.GPIO63=1;


	GpioDataRegs.GPASET.bit.GPIO20=1;
	GpioDataRegs.GPASET.bit.GPIO21=1;
	GpioDataRegs.GPASET.bit.GPIO22=1;
	GpioDataRegs.GPASET.bit.GPIO23=1;

	GpioDataRegs.GPBSET.bit.GPIO63=1;

	//软件版PWM1
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;//设置为通用GPIO功能
	// 方向设置为1时输出，设置为0时输入
	GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;//设置GPIO方向为输出
	GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;//使能GPIO上拉电阻(此时引脚默认为高电平)

	//软件版PWM2
	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;//设置为通用GPIO功能
	GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;//设置GPIO方向为输出
	GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;//使能GPIO上拉电阻(此时引脚默认为高电平)

	GpioDataRegs.GPASET.bit.GPIO24 = 1;
	GpioDataRegs.GPASET.bit.GPIO25 = 1;

	EDIS; // 开启写保护
}
