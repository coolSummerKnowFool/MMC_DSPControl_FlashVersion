#include "dma_xintf.h"
#include "epwm_combine.h"
#include "basic_control.h"
#include "leds.h"

#pragma CODE_SECTION(DMA_CH1_ISR, ".TI.ramfunc")
#pragma CODE_SECTION(DMA_CH2_ISR, ".TI.ramfunc")
#pragma CODE_SECTION(DMA_CH3_ISR, ".TI.ramfunc")

/* DMA 可访问 RAM 段（linker .cmd 中应定义 DMARAML4/5/6） */
#pragma DATA_SECTION(AD7606_1_Buf, "DMARAML4")
#pragma DATA_ALIGN(AD7606_1_Buf, 2)  // 强制偶数地址对齐(2 words = 32-bit aligned)
/* 把缓冲区定义放在这里（与 header 中的 pragma 配合） */
volatile Uint16 AD7606_1_Buf[AD7606_BUF_SIZE];

#pragma DATA_SECTION(AD7606_2_Buf, "DMARAML5")
#pragma DATA_ALIGN(AD7606_2_Buf, 2)
volatile Uint16 AD7606_2_Buf[AD7606_BUF_SIZE];

#pragma DATA_SECTION(AD7606_3_Buf, "DMARAML6")
#pragma DATA_ALIGN(AD7606_3_Buf, 2)
volatile Uint16 AD7606_3_Buf[AD7606_BUF_SIZE];

volatile Uint16 dma_done_count = 0;

/* ===========================
   XINTF：Zone0 / Zone6 / Zone7 初始化
   参照你给的 init_zone7() 样例并按 AD7606 时序要求设置
   AD7606: RD低 >=21ns, RD高 >=15ns
   假设 XTIMCLK = SYSCLKOUT（XTIMCLK 周期约 = 1/SYSCLK）
   下面用 ticks 值确保满足最小 ns 要求（可按实际 SYSCLK 精确计算）
   =========================== */
void Init_XINTF_Zones_AD7606(void)
{
    EALLOW;
    /* 确保 XINTF 时钟使能 */
    SysCtrlRegs.PCLKCR3.bit.XINTFENCLK = 1;
    EDIS;

    /* 配置 GPIO 为 XINTF（16-bit）——例程函数，在 DSP2833x_Xintf.c 中 */
    InitXintf16Gpio();

    EALLOW;
    /* 全局 XINTF 配置：XTIMCLK = SYSCLKOUT（使用芯片系统时钟作为 XTIM） */
    XintfRegs.XINTCNF2.bit.XTIMCLK = 0;  // XTIMCLK = SYSCLKOUT
    /* 写缓冲允许提高写性能，可根据需求调整 */
    XintfRegs.XINTCNF2.bit.WRBUFF = 3;   // 允许最多3个写操作缓冲
    /* 使能 XCLKOUT 输出（外设可选用该时钟） */
    XintfRegs.XINTCNF2.bit.CLKOFF = 0;   // 使能 XCLKOUT
    /* XCLKOUT = XTIMCLK */
    XintfRegs.XINTCNF2.bit.CLKMODE = 0;  // XCLKOUT = XTIMCLK

    /* ------------------ Zone0 (对应 XZCS0 -> AD7606 #1) ------------------ */
    /* 写时序（示例）：Lead/Active/Trail */
    XintfRegs.XTIMING0.bit.XWRLEAD   = 2; // 写引导周期：1 * XTIMCLK
    XintfRegs.XTIMING0.bit.XWRACTIVE = 4; // 写有效周期：2 * XTIMCLK
    XintfRegs.XTIMING0.bit.XWRTRAIL  = 2; // 写拖尾周期：1 * XTIMCLK

    /* 读时序（保证 RD 低电平持续 >=21ns） */
    XintfRegs.XTIMING0.bit.XRDLEAD   = 2; // 读引导周期：1 * XTIMCLK
    XintfRegs.XTIMING0.bit.XRDACTIVE = 4; // 读有效周期：3 * XTIMCLK （尽量 ≥21ns）
    XintfRegs.XTIMING0.bit.XRDTRAIL  = 2; // 读拖尾周期：0（可为0）

    XintfRegs.XTIMING7.bit.X2TIMING = 0;  // 不使用双倍时序
    /* 不使用 XREADY 信号（AD7606 响应时间由时序保证） */
    XintfRegs.XTIMING0.bit.USEREADY  = 0;
    XintfRegs.XTIMING0.bit.READYMODE = 0;
    /* x16 数据总线 */
    XintfRegs.XTIMING0.bit.XSIZE = 3; /* 3 表示 x16 */

    /* ------------------ Zone6 (对应 XZCS6 -> AD7606 #2) ------------------ */
    /* 直接复制 Zone0 的时序（如果需要对某块单独微调可单独设置） */
    XintfRegs.XTIMING6.all = XintfRegs.XTIMING0.all;

    /* ------------------ Zone7 (对应 XZCS7 -> AD7606 #3) ------------------ */
    XintfRegs.XTIMING7.all = XintfRegs.XTIMING0.all;

    EDIS;
    /* pipeline flush */
    asm(" RPT #7 || NOP");
}

/* ===========================
   Init_DMA_AD7606()
   - 配置 DMA 通道 1/2/3，触发源分别为 DMA_XINT1 / DMA_XINT2 / DMA_XINT4
   - 将 DMA 完成中断的向量映射到 PIE（在这里完成，不放 main）
   - 把 DMA 的目标缓冲区设置到 DMARAML4/L5/L6（在 .h 用 pragma 已声明）
   注意：
   - DMACHxAddrConfig 的参数按你例程的定义：DMACHxAddrConfig(DST, SRC)
   - Burst/Transfer 参数以 SDK/例程语义为准（这里使用常见例程风格）
   =========================== */
void Init_DMA_AD7606(void)
{
    EALLOW;
    /* 打开 DMA 时钟 */
    SysCtrlRegs.PCLKCR3.bit.DMAENCLK = 1;
    EDIS;

    /* 初始化 DMA 控制器（清寄存器） - 例程函数 */
    DMAInitialize();

    Init_XINTF_Zones_AD7606();

    /* ------------------- CH1 配置（AD7606 #1, 触发: XINT1） ------------------- */
    /* 地址配置：目的地(内部RAM缓冲) <- 源(XINTF zone0 地址)
       注意：这里使用 DMACH1AddrConfig(DST, SRC) 的示例 API（与你工程一致） */
    DMACH1AddrConfig(AD7606_1_Buf, AD7606_ZONE0_ADDR);

    /* 突发/传输配置：
       - 我们每次希望搬运 AD7606 的 AD7606_BUF_SIZE 个 16-bit 单元
       - 常见 TI 例程 DMACHxBurstConfig 参数格式： (burstCount-1, srcStep, dstStep)
         因此传入 AD7606_BUF_SIZE-1
       - srcStep/dstStep 以 16-bit 字为单位，如果希望地址不自增（固定地址），step 应为 0
         但常见做法是设置 step 为 1（按字递增），因为目标缓冲区要递增写入
       这里我们希望从 XINTF 固定地址读取连续 word（读取同一总线的连续位置）——
       对于外设并联情况，XINTF 读取每次对同样地址即可获得并联的 DB[15:0]（一次16位读取）。
       若使用库函数需要将 srcStep=0 保证每次对 same src 地址读取，可根据库语义设定。
    */
    /* 这里采用常见的：一次 burst 为 1 word，burstCount = AD7606_BUF_SIZE,
       也可以直接使用 DMACH1BurstConfig(AD7606_BUF_SIZE-1, 1, 1)
       具体依据你工程中 DMACH1BurstConfig 的实现语义 */
    DMACH1BurstConfig(15, 0, 1);  // 每个burst组大小 = AD7606_BUF_SIZE
    DMACH1TransferConfig(0, 0, 0);               // 单组 transfer
    DMACH1WrapConfig(0, 0, 0, 0);                // 禁用 wrap（明确写 0）
    /* 模式：外设触发 = DMA_XINT1；开启外设中断（PERINT_ENABLE）和 CH 中断（CHINT_ENABLE） */
    DMACH1ModeConfig(DMA_XINT1, PERINT_ENABLE, ONESHOT_DISABLE, CONT_ENABLE,
                     SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, SIXTEEN_BIT,
                     CHINT_END, CHINT_ENABLE);

    /* ------------------- CH2 配置（AD7606 #2, 触发: XINT2） ------------------- */
    DMACH2AddrConfig(AD7606_2_Buf, AD7606_ZONE6_ADDR);
    DMACH2BurstConfig(15, 0, 1);
    DMACH2TransferConfig(0, 0, 0);
    DMACH2WrapConfig(0, 0, 0, 0);
    DMACH2ModeConfig(DMA_XINT2, PERINT_ENABLE, ONESHOT_DISABLE, CONT_ENABLE,
                     SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, SIXTEEN_BIT,
                     CHINT_END, CHINT_ENABLE);

    /* ------------------- CH3 配置（AD7606 #3, 触发: XINT4） ------------------- */
    DMACH3AddrConfig(AD7606_3_Buf, AD7606_ZONE7_ADDR);
    DMACH3BurstConfig(15, 0, 1);
    DMACH3TransferConfig(0, 0, 0);
    DMACH3WrapConfig(0, 0, 0, 0);
    DMACH3ModeConfig(DMA_XINT4, PERINT_ENABLE, ONESHOT_DISABLE, CONT_ENABLE,
                     SYNC_DISABLE, SYNC_SRC, OVRFLOW_DISABLE, SIXTEEN_BIT,
                     CHINT_END, CHINT_ENABLE);

    /* ------------------- 中断向量映射与 PIE 使能 ------------------- */
    EALLOW;
    /* 将 DMA 完成中断挂到 PIE 向量（使用库中名字，不同 SDK 可能名略有差） */
    PieVectTable.DINTCH1 = &DMA_CH1_ISR; // DMA ch1 完成中断向量
    PieVectTable.DINTCH2 = &DMA_CH2_ISR; // DMA ch2 完成中断向量
    PieVectTable.DINTCH3 = &DMA_CH3_ISR; // DMA ch3 完成中断向量
    
    /* 使能 PIE 组7 的对应通道（DMA完成中断在组7） */
    // TODO
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;  // 使能 PIE 中组7 的 INT1 (DMA CH1)
    PieCtrlRegs.PIEIER7.bit.INTx2 = 0;  // DMA CH2
    PieCtrlRegs.PIEIER7.bit.INTx3 = 0;  // DMA CH3
    
    /* 允许 CPU 接收 INT7（DMA 对应） */
    IER |= M_INT7;

    EDIS;

    StartDMACH1();
    StartDMACH2();
    StartDMACH3();
}

/* ===========================
   DMA 中断处理：CH1 / CH2 / CH3
   每个中断：
   - 清外设中断标志（PERINTCLR）
   - dma_done_count++，若达到阈值则调用数据处理函数并清零
   - ACK PIE group7
   =========================== */
interrupt void DMA_CH1_ISR(void)
{
    /* 清 DMA CH1 的 PERINT 标志（例程寄存器） */
    DmaRegs.CH1.CONTROL.bit.PERINTCLR = 1;

    if(g_ADCIsrFlag)
        BridgeArmSort();
    
    // TODO
    // dma_done_count += g_ADCIsrFlag; // 预充电期间不处理数据，闭环期间才增加计数
    // if (dma_done_count >= DMA_DONE_COUNT_TARGET)
    // {
    //     BridgeArmSort();
    //     dma_done_count = 0;
    // }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
    /* ACK PIE group7 (DMA) */
}

interrupt void DMA_CH2_ISR(void)
{
    DmaRegs.CH2.CONTROL.bit.PERINTCLR = 1;
    dma_done_count += g_ADCIsrFlag; // 预充电期间不处理数据，闭环期间才增加计数
    if (dma_done_count >= DMA_DONE_COUNT_TARGET)
    {
        BridgeArmSort();
        dma_done_count = 0;
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

interrupt void DMA_CH3_ISR(void)
{
    DmaRegs.CH3.CONTROL.bit.PERINTCLR = 1;
    dma_done_count += g_ADCIsrFlag; // 预充电期间不处理数据，闭环期间才增加计数
    if (dma_done_count >= DMA_DONE_COUNT_TARGET)
    {
        BridgeArmSort();
        dma_done_count = 0;
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;
}

/* ===========================
   示例数据处理函数：当三路完成后调用（请按需实现）
   这里举例计算三路的平均值并占位处理
   =========================== */
// void Sort_AD7606_Data(void)
// {
//     // 函数框架有点多余,之后没有别的东西把这个函数直接加到中断里去!!!
//     BridgeArmSort();
//     asm(" NOP"); /* 占位，避免空函数被优化掉 */
// }
