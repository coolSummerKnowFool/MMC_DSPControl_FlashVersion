#include "tim.h"
#include "leds.h"
#include "menu.h"
#include "printf.h"
#include "dma_xintf.h"

#pragma CODE_SECTION(TIM0_IRQn, ".TI.ramfunc")
#pragma CODE_SECTION(TIM1_IRQn, ".TI.ramfunc")

//定时器0初始化函数
//Freq：CPU时钟频率（150MHz）
//Period：定时周期值，单位us
void TIM0_Init(float Freq, float Period)
{
	EALLOW;

	SysCtrlRegs.PCLKCR3.bit.CPUTIMER0ENCLK = 1; // CPU Timer 0

	//设置定时器0的中断入口地址为中断向量表的INT0
	PieVectTable.TINT0 = &TIM0_IRQn;

	//指向定时器0的寄存器地址
	CpuTimer0.RegsAddr = &CpuTimer0Regs;
	// 确保定时器0为停止状态
	CpuTimer0Regs.TCR.bit.TSS = 1;

	//设置定时器0的周期寄存器值（设置为最大值，没啥用，下面的配置函数会覆盖掉）
	// CpuTimer0Regs.PRD.all  = 0xFFFFFFFF;
	//设置定时器预定标计数器值为0
    // 下面两个寄存器分别代表TDDRH:TDDR和PSCH:PSC的低8位和高8位，也就是每个寄存器中小的8位属于TDDRH:TDDR，大的8位属于PSCH:PSC，
    // 两个寄存器拼起来才能组成完整的TDDRH:TDDR和PSCH:PSC
    // 1. PSCH:PSC 作为递减计数器，每个 SYSCLKOUT 周期减1。
    // 2. 当 PSCH:PSC 减到0时，触发两个动作：
    //      重载：PSCH:PSC 从 TDDRH:TDDR 自动获取新的初值。
    //      递减：主计数器 TIMH:TIM 减1。
    // 3. 随后，PSCH:PSC 用新的初值开始新一轮的递减计数，如此循环往复。
    // 所以周期计算公式为：定时周期 = (PRD + 1) × (TDDRH:TDDR + 1) / SYSCLKOUT
	CpuTimer0Regs.TPR.all  = 0;
	CpuTimer0Regs.TPRH.all = 0;
	// 重载使能，运行这一句会将TDDRH:TDDR的值重新载入PSCH:PSC，所以如果需要预分频的话，记得在这句之前设置，
    // 而且只需要设置TDDRH和TDDR的值就行，不需要设置PSCH和PSC
	CpuTimer0Regs.TCR.bit.TRB = 1;
	// Reset interrupt counters:
	// CpuTimer0.InterruptCount = 0;
    // 上面没有提及或者配置不完全的位都在这个函数中被配置了
	ConfigCpuTimer(&CpuTimer0, Freq, Period);

    // 使能PIE控制器中对应组的中断 (例如INT1.7)
    // 这条代码告诉PIE：如果INT1.7有中断，请转发给CPU
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// 使能CPU级别的中断组 (INT1)
    // 这条代码告诉CPU：请接收来自INT1组的中断请求
	IER |= M_INT1;
    
	// 开始定时器功能
	// CpuTimer0Regs.TCR.bit.TSS=0;

    EDIS;
}

void TIM0_Start(void)
{
    EALLOW;
    CpuTimer0Regs.TCR.bit.TRB = 1;   // 触发重载，从0开始计数
    CpuTimer0Regs.TCR.bit.TIF = 1;   // 清中断标志
    CpuTimer0Regs.TCR.bit.TSS = 0;   // 启动定时器
    EDIS;
}

// 停止定时器0
void TIM0_Stop(void)
{
    EALLOW;
    CpuTimer0Regs.TCR.bit.TSS = 1;   // 停止计时器
    EDIS;
}

interrupt void TIM0_IRQn(void)
{
	CpuTimer0Regs.TCR.bit.TIF = 1;
	switch (autoRefreshClass)
	{
	case NoChooseClass:
		break;
	
	case UpperBridgeArmClass:
		// 更新ADC数据
		sprintf(menuCaptionStr1, "1:%u 2:%u", AD7606_3_Buf[0], AD7606_3_Buf[1]);
		sprintf(menuCaptionStr2, "3:%u 4:%u", AD7606_3_Buf[2], AD7606_3_Buf[3]);
		sprintf(menuCaptionStr3, "5:%.2f 6:%.2f", AD7606_3_Buf[4] * 5.0 / 32768, AD7606_3_Buf[5] * 5.0 / 32768);
		sprintf(menuCaptionStr4, "7:%.2f 8:%.2f", AD7606_3_Buf[6] * 5.0 / 32768, AD7606_3_Buf[7] * 5.0 / 32768);
		updataMeunFlag = CurrentUpdata;
		break;

	case LowerBridgeArmClass:
		break;

	case PWMConfigClass:
		sprintf(menuCaptionStr1, "<  状态: %s  >", (GpioDataRegs.GPADAT.bit.GPIO12 ? "关" : "开"));
		sprintf(menuCaptionStr2, "周期计数: %u", EPwm1Regs.TBPRD);
		updataMeunFlag = CurrentUpdata;
		break;
		
	default:
		break;
	}
	updataMeunFlag = 1;
	PieCtrlRegs.PIEACK.bit.ACK1=1;
}

void TIM1_Init(float Freq, float Period)
{
	EALLOW;
	SysCtrlRegs.PCLKCR3.bit.CPUTIMER1ENCLK = 1; // CPU Timer 1
	
	PieVectTable.XINT13 = &TIM1_IRQn;

	CpuTimer1.RegsAddr = &CpuTimer1Regs;
	CpuTimer1Regs.TCR.bit.TSS = 1;

	// 这两句有点多余
	// CpuTimer1Regs.PRD.all  = 0xFFFFFFFF;
	// CpuTimer1.InterruptCount = 0;

	CpuTimer1Regs.TPR.all  = 0;
	CpuTimer1Regs.TPRH.all = 0;
	CpuTimer1Regs.TCR.bit.TRB = 1;
	ConfigCpuTimer(&CpuTimer1, Freq, Period);

	IER |= M_INT13;

	CpuTimer1Regs.TCR.bit.TSS=0;

	EDIS;
}

interrupt void TIM1_IRQn(void)
{
	CpuTimer1Regs.TCR.bit.TIF = 1;
	// LED3_TOGGLE;
	SOFT_PWM1_TOGGLE;
}

void TIM2_Init(float Freq, float Period)
{
	EALLOW;
	SysCtrlRegs.PCLKCR3.bit.CPUTIMER2ENCLK = 1; // CPU Timer 2

	PieVectTable.TINT2 = &TIM2_IRQn;

	CpuTimer2.RegsAddr = &CpuTimer2Regs;
	// CpuTimer2Regs.PRD.all  = 0xFFFFFFFF;
	// CpuTimer2.InterruptCount = 0;
	CpuTimer2Regs.TPR.all  = 0;
	CpuTimer2Regs.TPRH.all = 0;
	CpuTimer2Regs.TCR.bit.TSS = 1;
	CpuTimer2Regs.TCR.bit.TRB = 1;
	ConfigCpuTimer(&CpuTimer2, Freq, Period);

	IER |= M_INT14;

	CpuTimer2Regs.TCR.bit.TSS=0;

	EDIS;
}

interrupt void TIM2_IRQn(void)
{
	CpuTimer2Regs.TCR.bit.TIF = 1;
	LED4_TOGGLE;
}
