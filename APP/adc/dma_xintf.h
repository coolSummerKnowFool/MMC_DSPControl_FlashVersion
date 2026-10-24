#ifndef __DMA_XINTF_H
#define __DMA_XINTF_H

#include "DSP2833x_Device.h"     // DSP2833x 头文件
#include "DSP2833x_Examples.h"   // DSP2833x 例子相关头文件

/* 每块 AD7606 一次采样要搬运的 16-bit 单元数（示例为 8 通道 * 2 若需要可改） */
#define AD7606_CHANNELS_PER_CONV 8
#define AD7606_SAMPLES_PER_TRANSFER (AD7606_CHANNELS_PER_CONV) // 每次搬运单组通道
#define AD7606_BUF_SIZE AD7606_SAMPLES_PER_TRANSFER

/* 计数阈值：三块 ADC 都完成一次就处理一次 */
#define DMA_DONE_COUNT_TARGET 3

#define AD7606_ZONE0_ADDR (volatile Uint16 *)0x00004000
#define AD7606_ZONE6_ADDR (volatile Uint16 *)0x00100000
#define AD7606_ZONE7_ADDR (volatile Uint16 *)0x00200000

/* DMA 目标缓冲区（放在 DMARAMLx，确保 DMA 可访问） */
extern volatile Uint16 AD7606_1_Buf[AD7606_BUF_SIZE];
extern volatile Uint16 AD7606_2_Buf[AD7606_BUF_SIZE];
extern volatile Uint16 AD7606_3_Buf[AD7606_BUF_SIZE];

/* DMA 完成计数器（中断处理时使用） */
extern volatile Uint16 dma_done_count;

/* 初始化与处理接口 */
void Init_XINTF_Zones_AD7606(void);  // XINTF zone0/6/7 时序配置
void Init_DMA_AD7606(void);          // DMA 通道配置（CH1/CH2/CH3）

/* DMA 中断服务函数声明（PIE向量中会挂接它们） */
interrupt void DMA_CH1_ISR(void);
interrupt void DMA_CH2_ISR(void);
interrupt void DMA_CH3_ISR(void);

#endif /* __DMA_XINTF_H */
