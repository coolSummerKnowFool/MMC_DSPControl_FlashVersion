#ifndef __UART_FIFO_H
#define __UART_FIFO_H

#include "DSP2833x_Device.h"   // DSP2833x 头文件
#include "DSP2833x_Examples.h" // DSP2833x 例子相关头文件

void UARTb_FIFO_Init(Uint32 baud);
void UARTb_FIFO_SendByte(char data);
void UARTb_FIFO_SendString(char *msg);

interrupt void scibTxIsr_FIFO(void);
interrupt void scibRxIsr_FIFO(void);

int UARTb_FIFO_ReadByte(Uint16 *data);
Uint16 UARTb_FIFO_GetRxCount(void);
int UARTb_FIFO_CheckOverflow(void);

void uart_printf(char * fmt, ...); //自定义变参函数

#endif /* __UART_FIFO_H */
