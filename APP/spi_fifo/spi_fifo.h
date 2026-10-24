#ifndef __SPI_H
#define __SPI_H

#include "DSP2833x_Device.h"   // DSP2833x 头文件
#include "DSP2833x_Examples.h" // DSP2833x 例子相关头文件

// ====================== 环形缓冲定义 ======================
#define SPIA_TX_BUF_SIZE 64U
#define SPIA_TX_MASK (SPIA_TX_BUF_SIZE - 1U)
#define SPIA_RX_BUF_SIZE 64U
#define SPIA_RX_MASK (SPIA_RX_BUF_SIZE - 1U)

void SPIA_Init_Advanced(void);
int SPIA_SendNonBlocking(Uint16 data);
int SPIA_ReadNonBlocking(Uint16 *pData);
void SPIA_CS_Init(void);
void SPIA_SetMode(Uint16 mode);

interrupt void SPIA_RxISR(void);
interrupt void SPIA_TxISR(void);

#define OLED_CS_LOW   (GpioDataRegs.GPBCLEAR.bit.GPIO57=1)
#define OLED_CS_HIGH  (GpioDataRegs.GPBSET.bit.GPIO57=1)

#define FLASH_CS_LOW  (GpioDataRegs.GPBCLEAR.bit.GPIO58=1)
#define FLASH_CS_HIGH (GpioDataRegs.GPBSET.bit.GPIO58=1)

#define OLED_BAUD 0x005D // OLED SPI 频率 400kHz
#define FLASH_BAUD 0x0001 // Flash SPI 频率 最大支持104MHz，直接跑满18.75MHz即可

#define OLED_MODE 3
#define FLASH_MODE 0

#define SPI_TX_8BIT(data) ((data & 0xFF) << 8)   // ★ 新增宏

// extern volatile Uint16 spia_tx_head;
// extern volatile Uint16 spia_tx_tail;
extern volatile Uint16 spia_tx_buf[];
extern volatile Uint16 spia_rx_buf[];
extern volatile Uint16 spia_rx_head;
extern volatile Uint16 spia_rx_tail;

#endif /* __SPI_H */
