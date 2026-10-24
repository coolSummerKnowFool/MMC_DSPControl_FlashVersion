#ifndef __FLASH_H
#define __FLASH_H

#include "DSP2833x_Device.h"   // DSP2833x 头文件
#include "DSP2833x_Examples.h" // DSP2833x 例子相关头文件


#define CMD_WREN        0x06
#define CMD_RDSR        0x05
#define CMD_READ        0x03
#define CMD_FAST_READ   0x0B
#define CMD_PP          0x02
// 扇区擦除  4KB
#define CMD_SE          0x20
// 块擦除  64KB
#define CMD_BE          0xD8
// 全片擦除  8MB
#define CMD_CE          0xC7

#define CMD_JEDEC_ID    0x9F
#define CMD_DUMMY       0xFF

#define PAGE_SIZE       256
#define SECTOR_SIZE     4096

// 读取设备ID EF 40 17
void W25Q64_ReadID(Uint16 *mid, Uint16 *memtype, Uint16 *capacity);
// 查询设备是否空闲，空闲返回0
void W25Q64_WaitBusy(void);
// 扇区擦除(4K)
void W25Q64_SectorErase(Uint32 addr);
// 整片擦除 (8M)
void W25Q64_BulkErase(void);
// 写入数据，后面可能还要改
void W25Q64_PageProgram(Uint32 addr, Uint16 *buf, Uint16 len);
// 读取数据，返回spia_rx_buff应该读取的首位序号
// Uint16 W25Q64_ReadData(Uint32 addr, Uint16 len);
void W25Q64_ReadData(Uint32 addr, Uint16 len, Uint16 *buf);

#endif /* __FLASH_H */
