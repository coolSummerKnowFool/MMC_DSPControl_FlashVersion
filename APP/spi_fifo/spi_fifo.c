#include "spi_fifo.h"
#include "printf.h"

#pragma CODE_SECTION(SPIA_RxISR, ".TI.ramfunc")
#pragma CODE_SECTION(SPIA_TxISR, ".TI.ramfunc")
#pragma CODE_SECTION(SPIA_FillTxFifo_Batched, ".TI.ramfunc")
#pragma CODE_SECTION(SPIA_SendNonBlocking, ".TI.ramfunc")
#pragma CODE_SECTION(SPIA_ReadNonBlocking, ".TI.ramfunc")

// 软件发送缓冲区及指针
volatile Uint16 spia_tx_buf[SPIA_TX_BUF_SIZE];
volatile Uint16 spia_tx_head = 0;
volatile Uint16 spia_tx_tail = 0;
// 待发送计数（软件维护，表示环形缓冲中未发送到硬件的字数）
volatile Uint16 spia_tx_pending = 0;

// 软件接收缓冲区及指针
volatile Uint16 spia_rx_buf[SPIA_RX_BUF_SIZE];
volatile Uint16 spia_rx_head = 0;
volatile Uint16 spia_rx_tail = 0;

// ============================================================
// 函数名称：SPIA_FillTxFifo
// 功能说明：将软件环形缓冲中的数据尽可能写入 SPI 发送 FIFO（深度16）
// ============================================================
static inline Uint16 SPIA_FillTxFifo_Batched(void)
{
    Uint16 availableSpace;
    Uint16 bytesToSend;
    Uint16 i;

    // 读硬件 FIFO 已占用字数，计算可写空位
    availableSpace = 16 - SpiaRegs.SPIFFTX.bit.TXFFST; // FIFO 深度16

    // 要发的字节数：取最小值（可写空位, 软件待发送量）
    bytesToSend = (availableSpace < spia_tx_pending) ? availableSpace : spia_tx_pending;

    for (i = 0; i < bytesToSend; i++)
    {
        // 从环形缓冲读数据写入 SPI TX FIFO
        SpiaRegs.SPITXBUF = (spia_tx_buf[spia_tx_tail] << 8);
        // SpiaRegs.SPITXBUF = spia_tx_buf[spia_tx_tail];
        spia_tx_tail = (spia_tx_tail + 1) & SPIA_TX_MASK;

        // 更新软件计数
        spia_tx_pending--;
    }

    return bytesToSend;
}

void SPIA_CS_Init(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1; // 开启GPIO时钟

    // OLED CS - GPIO57
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0; // 设置为通用GPIO功能
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;  // 设置GPIO方向为输出
    GpioCtrlRegs.GPBPUD.bit.GPIO57 = 1;  // 不使能GPIO上拉电阻(外部有接)

    // Flash CS - GPIO58
    GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0; // 设置为通用GPIO功能
    GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;  // 设置GPIO方向为输出
    GpioCtrlRegs.GPBPUD.bit.GPIO58 = 1;  // 不使能GPIO上拉电阻(外部有接)

    GpioDataRegs.GPBSET.bit.GPIO57 = 1; // OLED CS 拉高
    GpioDataRegs.GPBSET.bit.GPIO58 = 1; // Flash CS 拉高
    EDIS;
}

// ============================================================
// 函数名称：SPIA_Init_Advanced
// 功能说明：配置SPIA外设、FIFO和中断，支持软件缓冲和非阻塞通信
// ============================================================
void SPIA_Init_Advanced(void)
{
    EALLOW;
    // 使能SPIA模块时钟
    SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;
    EDIS;

    // 初始化SPIA引脚
    InitSpiaGpio();

    EALLOW;

    // ========== 发送FIFO配置 ==========
    SpiaRegs.SPIFFTX.bit.SPIRST = 1;     // 释放SPI复位
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;   // 使能FIFO增强功能
    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;     // 启用发送FIFO
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1; // 清除TX中断标志
    SpiaRegs.SPIFFTX.bit.TXFFIENA = 1;   // 允许TXFIFO中断
    SpiaRegs.SPIFFTX.bit.TXFFIL = 4;     // 触发TX中断的FIFO阈值（4字）

    // ========== 接收FIFO配置 ==========
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1; // 启用接收FIFO
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;  // 清除溢出标志
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;  // 清除接收中断标志
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;    // 允许接收FIFO中断
    // SpiaRegs.SPIFFRX.bit.RXFFIL = 4;      // 接收FIFO中断阈值（4字）
    SpiaRegs.SPIFFRX.bit.RXFFIL = 1;      // 接收FIFO中断阈值（1字）

    // FIFO传输时序
    SpiaRegs.SPIFFCT.bit.TXDLY = 0; // 不添加额外延时

    // ========== SPI工作模式配置 ==========
    SpiaRegs.SPICCR.bit.SPISWRESET = 0; // 暂时保持复位
    SpiaRegs.SPICCR.bit.SPICHAR = 0x7;  // 每帧8位
    // SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;   // 空闲时钟为低电平
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;  // 空闲时钟为高电平
    SpiaRegs.SPICCR.bit.SPILBK = 0;       // 禁止内部回环
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1; // 主机模式
    SpiaRegs.SPICTL.bit.TALK = 1;         // 允许发送
    // SpiaRegs.SPICTL.bit.CLK_PHASE = 0;     // 数据采样于第一个边沿
    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;     // 数据采样于第二个边沿
    SpiaRegs.SPICTL.bit.OVERRUNINTENA = 1; // 使能溢出中断（可用于调试）
    SpiaRegs.SPIBRR = 0x0003;              // 设置波特率分频（根据LSPCLK调整）

    SpiaRegs.SPIPRI.bit.FREE = 1;          // 允许在仿真暂停时继续运行

    // ========== 中断向量与使能 ==========
    PieVectTable.SPIRXINTA = &SPIA_RxISR; // 注册接收中断函数
    PieVectTable.SPITXINTA = &SPIA_TxISR; // 注册发送中断函数
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;    // 允许PIE组6的INTx1（接收）
    PieCtrlRegs.PIEIER6.bit.INTx2 = 1;    // 允许PIE组6的INTx2（发送）
    IER |= M_INT6;                        // 允许CPU响应组6中断

    // 释放SPI复位，正式运行
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;
    EDIS;

    SPIA_CS_Init();
}

// ============================================================
// 接收中断服务函数：从FIFO批量读取数据到软件环形缓冲
// ============================================================
interrupt void SPIA_RxISR(void)
{
    while (SpiaRegs.SPIFFRX.bit.RXFFST > 0)
    {
        // printf("raw=0x%04X low=0x%02X high=0x%02X\r\n", 1, 2, 3);
        // Uint16 raw = SpiaRegs.SPIRXBUF;    // 读取原始 16-bit 寄存器
        // Uint16 low = raw & 0x00FF;
        // Uint16 high = (raw >> 8) & 0x00FF;

        spia_rx_buf[spia_rx_head] = SpiaRegs.SPIRXBUF;    // 读取FIFO中的数据
        spia_rx_head = (spia_rx_head + 1) & SPIA_RX_MASK; // 更新接收指针
    }
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // 清除中断标志
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // 通知PIE可以接收下一中断
}

// ============================================================
// 发送中断服务函数：FIFO有空位时自动写入更多数据
// ============================================================
interrupt void SPIA_TxISR(void)
{
    SPIA_FillTxFifo_Batched(); // 填充发送FIFO

    // 若发送缓冲已空，则关闭发送中断
    if (spia_tx_tail == spia_tx_head)
    {
        SpiaRegs.SPIFFTX.bit.TXFFIENA = 0;
    }

    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1; // 清除发送中断标志
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

// ============================================================
// 非阻塞发送：将数据写入软件缓冲，并尝试立即写入FIFO
// ============================================================
// 非阻塞入队发送一个字（16位）
// 成功返回1，失败返回0（缓冲满）
int SPIA_SendNonBlocking(Uint16 data)
{
    Uint16 next;
    // 先计算下一个位置（不过这里要在临界区内写入并更新 head/pending）
    next = (spia_tx_head + 1) & SPIA_TX_MASK;

    // 如果缓冲已满，立即返回失败（避免在这里阻塞）
    if (next == spia_tx_tail)
    {
        printf("SPIA Send Buffer Full!\n");
        return 0;
    }

    // ---- 短临界区开始 ----
    // 为防止 ISR 并发修改 txTail/spia_tx_pending，我们先禁用 SPI 的 TX FIFO 中断
    EALLOW;
    SpiaRegs.SPIFFTX.bit.TXFFIENA = 0; // 关闭 TX 中断，保护下面对共享变量的修改
    EDIS;

    // 将数据写入软件缓冲并更新 head 与待发送计数
    spia_tx_buf[spia_tx_head] = data;
    // spia_tx_buf[spia_tx_head] = SPI_TX_8BIT(data); // 只发送低8位，符合OLED/Flash的8位SPI协议
    spia_tx_head = next;
    spia_tx_pending++; // 新增一个待发送字

    // 尝试立即把数据推入硬件 FIFO（减少后续中断次数）
    // 注意：SPIA_FillTxFifo_Batched 内部会读取 TXFFST 并写寄存器（EALLOW 不必包裹所有写）
    SPIA_FillTxFifo_Batched();

    // 如果仍有待发送数据，则开启 TX 中断；否则保持关闭
    EALLOW;
    if (spia_tx_pending > 0)
    {
        SpiaRegs.SPIFFTX.bit.TXFFIENA = 1; // 启用 TX 中断，由 ISR 继续填充
    }
    else
    {
        SpiaRegs.SPIFFTX.bit.TXFFIENA = 0; // 没数据就不启中断
    }
    EDIS;
    // ---- 短临界区结束 ----

    return 1;
}

// ============================================================
// 非阻塞接收：若有数据可读则输出到*pData并返回1
// ============================================================
int SPIA_ReadNonBlocking(Uint16 *pData)
{
    if (spia_rx_head == spia_rx_tail)
        return 0;
    *pData = spia_rx_buf[spia_rx_tail];
    spia_rx_tail = (spia_rx_tail + 1) & SPIA_RX_MASK;
    return 1;
}

// ============================================================
// 函数名称：SPIA_SetMode
// 功能说明：在SPIA已初始化后动态切换模式（0或3）
// 形参说明：mode = 0 表示模式0 (Flash)，mode = 3 表示模式3 (OLED)
// 注意事项：切换前会确保SPI进入安全状态，避免数据丢失
// ============================================================
void SPIA_SetMode(Uint16 mode)
{
    EALLOW;

    // 1. 确保 SPI 空闲（等待发送FIFO空）
    while (SpiaRegs.SPIFFTX.bit.TXFFST != 0) {}

    // 2. 暂时关闭 SPI
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;

    // 3. 根据模式设置极性与相位
    switch (mode)
    {
        case FLASH_MODE: // 模式0：空闲低，首沿采样（Flash）
            SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
            SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
            SpiaRegs.SPIFFRX.bit.RXFFIL = 1;      // 接收FIFO中断阈值（1字）
            break;

        case OLED_MODE: // 模式3：空闲高，次沿采样（OLED）
            SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;
            SpiaRegs.SPICTL.bit.CLK_PHASE = 1;
            SpiaRegs.SPIFFRX.bit.RXFFIL = 8;      // 接收FIFO中断阈值（8字）
            break;

        default:
            // 非法模式保护
            break;
    }

    // 4. 重新使能 SPI
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;

    EDIS;
}
