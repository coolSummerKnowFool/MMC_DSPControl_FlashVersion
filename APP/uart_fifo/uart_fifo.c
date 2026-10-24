#include "uart_fifo.h"
#include "printf.h"

#pragma CODE_SECTION(scibTxIsr_FIFO, ".TI.ramfunc")
#pragma CODE_SECTION(scibRxIsr_FIFO, ".TI.ramfunc")
#pragma CODE_SECTION(UARTb_FIFO_SendByte, ".TI.ramfunc")
#pragma CODE_SECTION(UARTb_FIFO_ReadByte, ".TI.ramfunc")

#define RX_BUFFER_SIZE 64U    // 环形缓冲区大小
#define RX_BUFFER_MASK (RX_BUFFER_SIZE - 1U)

#define RX_FIFO_TRIGGER_LEVEL 1   // 接收 FIFO 达到 1 字节触发中断
#define TX_FIFO_TRIGGER_LEVEL 8   // 发送 FIFO 空位达到 4 字节触发中断

volatile Uint16 rxBuffer[RX_BUFFER_SIZE];  // 接收环形缓冲区
volatile Uint16 rxHead = 0;              // 写入位置
volatile Uint16 rxTail = 0;              // 读取位置  
volatile Uint16 rxOverflow = 0;          // 溢出标志

volatile Uint16 txBuffer[RX_BUFFER_SIZE];  // 发送环形缓冲区
volatile Uint16 txHead = 0;              // 写入位置
volatile Uint16 txTail = 0;              // 读取位置
volatile Uint16 txPending = 0;           // 待发送字节数


//=================================================
// 初始化 SCI-B
//=================================================
void UARTb_FIFO_Init(Uint32 baud)
{
    unsigned char scihbaud = 0;
    unsigned char scilbaud = 0;
    Uint16 scibaud = 0;

    scibaud = 37500000 / (8 * baud) - 1;
    scihbaud = scibaud >> 8;
    scilbaud = scibaud & 0xff;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.SCIBENCLK = 1;   // 使能 SCI-B 时钟
    EDIS;

    InitScibGpio();

    // ========== 初始化 SCI FIFO ==========
    
    // TX FIFO 配置
    ScibRegs.SCIFFTX.bit.SCIRST       = 1;    // 释放 FIFO reset
    ScibRegs.SCIFFTX.bit.SCIFFENA     = 1;    // FIFO 增强模式
    ScibRegs.SCIFFTX.bit.TXFIFOXRESET = 1;    // TX FIFO 使能
    ScibRegs.SCIFFTX.bit.TXFFIENA     = 1;    // 使能 TX FIFO 中断
    ScibRegs.SCIFFTX.bit.TXFFIL       = TX_FIFO_TRIGGER_LEVEL; // 空位达到4字节触发中断

    // RX FIFO 配置  
    ScibRegs.SCIFFRX.bit.RXFIFORESET  = 1;    // RX FIFO 使能
    ScibRegs.SCIFFRX.bit.RXFFIENA     = 1;    // 使能 RX FIFO 中断
    ScibRegs.SCIFFRX.bit.RXFFIL       = RX_FIFO_TRIGGER_LEVEL; // 达到8字节触发中断

    ScibRegs.SCIFFCT.all = 0x0;               // 禁止自动波特率

    // 清除所有中断标志
    ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;

    // ========== SCI 控制寄存器配置 ==========
    ScibRegs.SCICCR.bit.STOPBITS      = 0;    // 1个停止位
    ScibRegs.SCICCR.bit.PARITYENA     = 0;    // 无校验
    ScibRegs.SCICCR.bit.SCICHAR       = 7;    // 8位数据
    ScibRegs.SCICCR.bit.LOOPBKENA     = 0;    // 禁止环回
    ScibRegs.SCICCR.bit.ADDRIDLE_MODE = 0;    // 空闲线模式

    ScibRegs.SCICTL1.bit.TXENA        = 1;    // 开启发送
    ScibRegs.SCICTL1.bit.RXENA        = 1;    // 开启接收
    
    ScibRegs.SCICTL2.bit.TXINTENA     = 1;    // 使能 TX 中断
    ScibRegs.SCICTL2.bit.RXBKINTENA   = 1;    // 使能 RX 中断
    
    ScibRegs.SCIHBAUD = scihbaud;
    ScibRegs.SCILBAUD = scilbaud;
    
    ScibRegs.SCICTL1.bit.SWRESET = 1;         // 解除 SCI reset

    // ========== 配置 PIE 中断 ==========
    EALLOW;
    PieVectTable.SCIRXINTB = &scibRxIsr_FIFO;      // RX ISR
    PieVectTable.SCITXINTB = &scibTxIsr_FIFO;      // TX ISR
    
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;        // Group9-3: SCIRXINTB
    PieCtrlRegs.PIEIER9.bit.INTx4 = 1;        // Group9-4: SCITXINTB
    
    IER |= M_INT9;                            // 允许 CPU 响应 Group 9
    EDIS;
}

//=================================================
// 发送单个字节（非阻塞，使用缓冲区）
//=================================================
void UARTb_FIFO_SendByte(char data)
{
    Uint16 nextHead;
    
    // 计算下一个写入位置
    nextHead = (txHead + 1) & RX_BUFFER_MASK;
    
    // 如果缓冲区满，等待
    while(nextHead == txTail)
    {
        // 缓冲区满，可以在这里处理或返回错误
        // 实际应用中可能需要超时机制
    }
    
    // 写入数据到发送缓冲区
    txBuffer[txHead] = data;
    txHead = nextHead;
    
    // 更新待发送字节计数
    txPending++;
    
    // 如果发送中断被禁用（表示当前没有在发送），则手动启动发送
    if(ScibRegs.SCIFFTX.bit.TXFFIENA == 0)
    {
        // 使能发送中断，触发第一次发送
        ScibRegs.SCIFFTX.bit.TXFFIENA = 1;
    }
}

//=================================================
// 发送字符串
//=================================================
void UARTb_FIFO_SendString(char *msg)
{
    while(*msg != '\0')
    {
        UARTb_FIFO_SendByte(*msg++);
    }
}

//=================================================
// 发送中断服务函数
//=================================================
interrupt void scibTxIsr_FIFO(void)
{
    Uint16 availableSpace;
    Uint16 bytesToSend;
    Uint16 i;
    
    // 检查发送 FIFO 中断
    if(ScibRegs.SCIFFTX.bit.TXFFINT == 1)
    {
        // 获取发送 FIFO 中的可用空间
        availableSpace = 16 - ScibRegs.SCIFFTX.bit.TXFFST;
        
        // 计算可以发送的字节数（取可用空间和待发送数的较小值）
        bytesToSend = (availableSpace < txPending) ? availableSpace : txPending;
        
        // 从发送缓冲区读取数据并发送
        for(i = 0; i < bytesToSend; i++)
        {
            // 修正：直接赋值给 SCITXBUF，不要使用 .all
            ScibRegs.SCITXBUF = txBuffer[txTail];
            txTail = (txTail + 1) & RX_BUFFER_MASK;
            txPending--;
        }
        
        // 如果所有数据都已发送完成，禁用发送中断
        if(txPending == 0)
        {
            ScibRegs.SCIFFTX.bit.TXFFIENA = 0;
        }
        
        // 清除发送中断标志
        ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    }
    
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

//=================================================
// 接收中断服务函数
//=================================================
interrupt void scibRxIsr_FIFO(void)
{
    char data;

    // FIFO 中有数据
    while (ScibRegs.SCIFFRX.bit.RXFFST > 0)
    {
        data = ScibRegs.SCIRXBUF.all;

        // 存入环形缓冲
        rxBuffer[rxHead] = data;
        rxHead = (rxHead + 1) & RX_BUFFER_MASK;

        // 简单转发回 PC
        // 后续接收中断动作在这里编写
        // UARTb_SendByte(data);

        // 若缓冲满则丢弃最旧数据，并标记溢出
        if (rxHead == rxTail)
        {
            rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
            rxOverflow = 1;   // 标记覆盖
        }
    }
    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    
    // 检查溢出标志
    if(ScibRegs.SCIFFRX.bit.RXFFOVF == 1)
    {
        ScibRegs.SCIFFRX.bit.RXFFOVRCLR = 1;
        rxOverflow = 1;  // 标记软件溢出标志
    }
    
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

//=================================================
// 从接收缓冲区读取一个字节（非阻塞）
// 返回：1-成功读取，0-缓冲区为空
//=================================================
int UARTb_FIFO_ReadByte(Uint16 *data)
{
    if(rxHead == rxTail)
    {
        return 0;  // 缓冲区为空
    }
    
    *data = rxBuffer[rxTail];
    rxTail = (rxTail + 1) & RX_BUFFER_MASK;
    return 1;
}

//=================================================
// 获取接收缓冲区中可读字节数
//=================================================
Uint16 UARTb_FIFO_GetRxCount(void)
{
    if(rxHead >= rxTail)
    {
        return rxHead - rxTail;
    }
    else
    {
        return RX_BUFFER_SIZE - rxTail + rxHead;
    }
}

//=================================================
// 检查是否发生溢出
//=================================================
int UARTb_FIFO_CheckOverflow(void)
{
    int overflow = rxOverflow;
    rxOverflow = 0;  // 读取后清除标志
    return overflow;
}
