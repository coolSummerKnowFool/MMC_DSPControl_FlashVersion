#include "exti.h"
#include "leds.h"
#include "ad760x.h"
#include "menu.h"

// ADBUSY1 初始化
void EXTI1_Init(void)
{
    EALLOW;

    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;

    // 配置GPIO引脚模式，
    // 0代表普通IO，由CPU直接控制引脚输出输入等
    // 1、2、3代表不同的外设功能，如PWM、ADC等等，由芯片内部特殊电路与CPU共同控制，
    // 具体功能情况见数据手册
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;
    // 配置GPIO方向为输入
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 0;
    // 不接上拉电阻
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 1;
    // 设置输入限定（只在配置为普通GPIO输入的时候有用）
    // 0(仅与SYSCLK同步)：输入信号经过一个简单的同步器（通常是2个触发器），与系统时钟同步。这可以过滤掉非常窄的毛刺（短于一个时钟周期），但响应最快。
    // 1和2(采样窗口限定): 输入信号必须在连续3(配置为1)个或6(配置为2)个特定的采样周期内保持稳定，才会被确认。这能有效过滤较宽的毛刺，但会引入延迟。
    // 3(异步路径): 输入信号不经过任何同步或滤波，直接进入CPU或外设。响应最快，但抗噪声能力最差，仅适用于非常干净的信号或需要立即响应的场景（如外部中断）。
    GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;
    // 该寄存器与上一句的采样窗口限定模式配套使用,用于设置每次采样的周期时间
    // QUALPRD0	GPIO0 - GPIO7	每个8位寄存器控制8个引脚的输入采样周期，这8个引脚的采样时间共享，不能单独设置。
    // QUALPRD1	GPIO8 - GPIO15
    // QUALPRD2	GPIO16 - GPIO23
    // QUALPRD3	GPIO24 - GPIO31
    // T = 2*n*T_sysclkout  这里的n就是下面的255，T_sysclkout是系统晶振时钟
    // GpioCtrlRegs.GPACTRL.bit.QUALPRD3 = 5;  // x表示0、1、2、3

    // XINT1、2还有XINT13只能由GPIO0-31触发，而XINT3-7则只能由GPIO31-63触发
    // 选择GPIO12来触发XINT1
    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 29;

    
    // 外部中断控制寄存器，从XINT1CR到XINT7CR，用于配置外部中断的触发方式以及中断使能
    // POLARITY: 0-下降沿触发中断，1-上升沿触发中断，2-下降沿触发中断，3-双边沿触发中断
    // ENABLE: 0-禁止外部中断，1-使能外部中断
    XIntruptRegs.XINT1CR.bit.POLARITY = 0; // 下降沿触发中断
    XIntruptRegs.XINT1CR.bit.ENABLE = 1;   // 使能XINT1

    // 不需要进入外部触发中断就注释掉下面三句
    // 设置外部中断1的中断函数地址
    // PieVectTable.XINT1 = &EXTI1_IRQn;

    // 使能PIE向量表中的PIEIER1的INT4，对应的就是外部中断1（XINT1）
    // CPU中断线由12个，对应的PIEIER寄存器也有PIEIER1-PIEIER12共12个
    // 每个PIEIERx可以控制8个小中断线，这里的INT4就是这8根线中的一根
    // 注：PIEIFR是PIE级的中断标志寄存器，一般由硬件自动使能，有的中断需要在中断函数结束后置1关闭中断使能
    // PieCtrlRegs.PIEIER1.bit.INTx4 = 1; // 使能PIE组1的INT4

    // IER：中断使能寄存器
    // 使能CPU中断1（INT1）（总共12支，代表CPU挂在可以相应的12个大中断线）
    // 注：IFR是中断标志寄存器，一般由硬件自动使能，而在中断函数或者由硬件自动清楚，所以在中断初始化配置中看不到
    // IER |= M_INT1;

    EDIS;
}

// 配置不进入中断
interrupt void EXTI1_IRQn(void)
{
    LED1_TOGGLE;
    // StartDMACH1();

    //  作用是将12根CPU级中断的对应PIEACKx位设置为0
    // 该位如果不为0，则无法响应，同组（INT1）后续的所有中断都会被阻塞。
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
    // 注：PIEACK 和 IFR 这两个寄存器的位有明确的因果关系和先后顺序
    // 1. 一个外设中断产生，使 PIEIFR1.bit.INTx 置位。
    // 2. 如果 PIEACK.bit.ACK1 == 0，PIE将该请求转发给CPU，导致 IFR.bit.INT1 == 1。
    // 3. CPU响应INT1中断，硬件自动将 IFR.bit.INT1 清零。
    // 4. 同时，硬件会自动将 PIEACK.bit.ACK1 置1，从而锁定INT1组，防止同组新中断打断当前服务。
    // 5. CPU开始执行INT1组的ISR。
    // 6. 在ISR末尾，程序员手动写 PIEACK.bit.ACK1 = 1 来将其清零，解锁INT1组，允许PIE再次向CPU转发该组的中断。
}

// ADBUSY2 初始化
void EXTI2_Init(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1; // GPIO input clock

    // ADBUSY2 端口配置
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO30 = 1;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO30 = 3;
    // GpioCtrlRegs.GPACTRL.bit.QUALPRD3 = 5;

    GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 30; // XINT2是GPIO30
    
    XIntruptRegs.XINT2CR.bit.POLARITY = 0; // 下降沿触发中断
    XIntruptRegs.XINT2CR.bit.ENABLE = 1;   // 使能XINT2

    // 不需要进入外部触发中断就注释掉下面三句
    // PieVectTable.XINT2 = &EXTI2_IRQn;

    // PieCtrlRegs.PIEIER1.bit.INTx5 = 1; // 使能PIE组1的INT5
    
    // IER |= M_INT1; // 使能CPU中断1（INT1）

    EDIS;
}

// 配置不进入中断
interrupt void EXTI2_IRQn(void)
{
    LED2_TOGGLE;
    // StartDMACH2();
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}

// ADBUSY3 初始化
void EXTI4_Init(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1; // GPIO input clock

    // ADBUSY3 端口配置
    GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO35 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO35 = 1;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO35 = 3;
    // GpioCtrlRegs.GPBCTRL.bit.QUALPRD0 = 5;

    GpioIntRegs.GPIOXINT4SEL.bit.GPIOSEL = 35; // XINT4是GPIO35
    
    XIntruptRegs.XINT4CR.bit.POLARITY = 0; // 下降沿触发中断
    XIntruptRegs.XINT4CR.bit.ENABLE = 1;   // 使能XINT4
    
    // 不需要进入外部触发中断就注释掉下面三句
    // PieVectTable.XINT4 = &EXTI4_IRQn;

    // PieCtrlRegs.PIEIER12.bit.INTx2 = 1; // 使能PIE组12的INT2

    // IER |= M_INT12; // 使能CPU中断1（INT1）

    EDIS;
}

// 配置不进入中断
interrupt void EXTI4_IRQn(void)
{
    LED3_TOGGLE;
    // StartDMACH3();
    PieCtrlRegs.PIEACK.bit.ACK12 = 1;
}

// 按键检测引脚A0~A2初始化
void KEY_Init(void)
{
    EALLOW;

    GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO49 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 0;

    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO50 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 0;

    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 0;

    EDIS;
}

// 按键中断引脚EXTI3初始化
void EXTI3_Init(void)
{
    EALLOW;

    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;

    // 配置GPIO引脚模式，
    // 0代表普通IO，由CPU直接控制引脚输出输入等
    // 1、2、3代表不同的外设功能，如PWM、ADC等等，由芯片内部特殊电路与CPU共同控制，
    // 具体功能情况见数据手册
    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;
    // 配置GPIO方向为输入
    GpioCtrlRegs.GPBDIR.bit.GPIO48 = 0;
    // 设置输入限定（只在配置为普通GPIO输入的时候有用）
    // 0(仅与SYSCLK同步)：输入信号经过一个简单的同步器（通常是2个触发器），与系统时钟同步。这可以过滤掉非常窄的毛刺（短于一个时钟周期），但响应最快。
    // 1和2(采样窗口限定): 输入信号必须在连续3(配置为1)个或6(配置为2)个特定的采样周期内保持稳定，才会被确认。这能有效过滤较宽的毛刺，但会引入延迟。
    // 3(异步路径): 输入信号不经过任何同步或滤波，直接进入CPU或外设。响应最快，但抗噪声能力最差，仅适用于非常干净的信号或需要立即响应的场景（如外部中断）。
    GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 1;
    // 该寄存器与上一句的采样窗口限定模式配套使用,用于设置每次采样的周期时间
    // QUALPRD0	GPIO0 - GPIO7	每个8位寄存器控制8个引脚的输入采样周期，这8个引脚的采样时间共享，不能单独设置。
    // QUALPRD1	GPIO8 - GPIO15
    // QUALPRD2	GPIO16 - GPIO23
    // QUALPRD3	GPIO24 - GPIO31
    // T = 2*n*T_sysclkout  这里的n就是下面的255，T_sysclkout是系统晶振时钟
    GpioCtrlRegs.GPBCTRL.bit.QUALPRD2 = 5; // x表示0、1、2、3

    // XINT1、2还有XINT13只能由GPIO0-31触发，而XINT3-7则只能由GPIO32-63触发
    // 选择GPIO48来触发XINT3
    GpioIntRegs.GPIOXINT3SEL.bit.GPIOSEL = 48;
    // 设置外部中断3的中断函数地址
    PieVectTable.XINT3 = &EXTI3_IRQn;
    // 使能PIE向量表中的PIEIER12的INT1，对应的就是外部中断3（XINT3）
    // CPU中断线由12个，对应的PIEIER寄存器也有PIEIER1-PIEIER12共12个
    // 每个PIEIERx可以控制8个小中断线，这里的INT1就是这8根线中的一根
    // 注：PIEIFR是PIE级的中断标志寄存器，一般由硬件自动使能，有的中断需要在中断函数结束后置1关闭中断使能
    PieCtrlRegs.PIEIER12.bit.INTx1 = 1; // 使能PIE组12的INT1

    // 外部中断控制寄存器，从XINT1CR到XINT7CR，用于配置外部中断的触发方式以及中断使能
    // POLARITY: 0-下降沿触发中断，1-上升沿触发中断，2-下降沿触发中断，3-双边沿触发中断
    // ENABLE: 0-禁止外部中断，1-使能外部中断
    XIntruptRegs.XINT3CR.bit.POLARITY = 0; // 下降沿触发中断
    XIntruptRegs.XINT3CR.bit.ENABLE = 1;   // 使能XINT3

    // IER：中断使能寄存器
    // 使能CPU中断12（INT12）（总共12支，代表CPU挂在可以相应的12个大中断线）
    // 注：IFR是中断标志寄存器，一般由硬件自动使能，而在中断函数或者由硬件自动清楚，所以在中断初始化配置中看不到
    IER |= M_INT12;

    EDIS;
}

// 按键中断服务函数
interrupt void EXTI3_IRQn(void)
{
    Uint32 i;
    for (i = 0; i < 10000; i++)
        ; // 键盘消抖动

    Uint16 code = (KEY_A2_STATUS << 2) | (KEY_A1_STATUS << 1) | (KEY_A0_STATUS << 0);
    switch (code)
    {
    case KEY_STATUS_DOWN: /* I7 被按下 */
        // LED1_TOGGLE;
        keyDownOption();
        break;
    case KEY_STATUS_CENTER: /* I6 */
        // LED1_TOGGLE;
        keyMidOption();
        break;
    case KEY_STATUS_RIGHT: /* I5 */
        // LED2_TOGGLE;
        keyRightOption();
        break;
    case KEY_STATUS_LEFT: /* I4 */
        // LED3_TOGGLE;
        keyLeftOption();
        break;
    case KEY_STATUS_UP: /* I3 */
        // LED4_TOGGLE;
        keyUpOption();
        break;
    default:
        break;
    }

    PieCtrlRegs.PIEACK.bit.ACK12 = 1;
}
