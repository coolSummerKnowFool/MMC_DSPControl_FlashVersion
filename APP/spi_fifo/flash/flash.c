#include "flash.h"
#include "spi_fifo.h"
#include "basic.h"

// 1.空间分层架构
//      块(Block) :每个块64KB，W25Q64共128块(8MB÷64KB = 128)。
//      扇区(Sector) :每个块划分为16个扇区，每个扇区4KB (64KB ÷4KB= 16)，每个扇区划分为16页。
//      页(Page) :每页256字节(4KB÷256B= 16)。
// 2.地址规律
//      页内地址:最低8位(Ox00~OxFF)，决定页内字节位置。
//      扇区地址:中间8位(Ox00~OxFF)，配合块地址定位扇区。  确定扇区(高4位)和页地址(低4位)  每个块(64K)有16个扇区，每个扇区(4K)有16页，每页(256B)有256个Byte存储位
//      块地址:最高8位(Ox00~Ox7F，W25Q64仅用前128块)。 确定块地址 (64K*128 = 8M)

Uint16 W25Q64_WriteReadValue(Uint16 cmd)
{
    Uint16 position;
    SPIA_SendNonBlocking(cmd);
    while (SpiaRegs.SPIFFTX.bit.TXFFST)
        ;
    DELAY_US(1);
    position = (spia_rx_head == 0) ? SPIA_RX_MASK : (spia_rx_head - 1);
    return position;
}

void W25Q64_WriteEnable(void)
{
    FLASH_CS_LOW;
    W25Q64_WriteReadValue(CMD_WREN);
    FLASH_CS_HIGH;
}

Uint16 W25Q64_ReadStatus(void)
{
    return spia_rx_buf[W25Q64_WriteReadValue(CMD_RDSR)];
}

void W25Q64_WaitBusy(void)
{
    FLASH_CS_LOW;
    W25Q64_ReadStatus();
    while (spia_rx_buf[W25Q64_WriteReadValue(CMD_DUMMY)] & 0x01)
        ;
    FLASH_CS_HIGH;
}

void W25Q64_ReadID(Uint16 *mid, Uint16 *memtype, Uint16 *capacity)
{
    Uint16 position;
    FLASH_CS_LOW;
    position = spia_rx_head;
    SPIA_SendNonBlocking(CMD_JEDEC_ID);
    SPIA_SendNonBlocking(CMD_DUMMY);
    SPIA_SendNonBlocking(CMD_DUMMY);
    SPIA_SendNonBlocking(CMD_DUMMY);
    while (SpiaRegs.SPIFFTX.bit.TXFFST)
        ;
    DELAY_US(1);
    *mid = spia_rx_buf[(++position) & SPIA_TX_MASK];
    *memtype = spia_rx_buf[(++position) & SPIA_TX_MASK];
    *capacity = spia_rx_buf[(++position) & SPIA_TX_MASK];
    FLASH_CS_HIGH;
}

// 扇区擦除 (4K)
void W25Q64_SectorErase(Uint32 addr)
{
    W25Q64_WriteEnable();
    FLASH_CS_LOW;
    W25Q64_WriteReadValue(CMD_SE);
    W25Q64_WriteReadValue((addr >> 16) & 0xFF);
    W25Q64_WriteReadValue((addr >> 8) & 0xFF);
    W25Q64_WriteReadValue(addr & 0xFF);
    FLASH_CS_HIGH;
}

// 整片擦除 (8M)
void W25Q64_BulkErase(void)
{
    W25Q64_WriteEnable();
    FLASH_CS_LOW;
    DELAY_US(1);
    W25Q64_WriteReadValue(CMD_CE);
    FLASH_CS_HIGH;
}

void W25Q64_PageProgram(Uint32 addr, Uint16 *buf, Uint16 len)
{
    Uint16 i;
    W25Q64_WriteEnable();
    FLASH_CS_LOW;
    SPIA_SendNonBlocking(CMD_PP);
    SPIA_SendNonBlocking((addr >> 16) & 0xFF);
    SPIA_SendNonBlocking((addr >> 8) & 0xFF);
    SPIA_SendNonBlocking(addr & 0xFF);
    while (SpiaRegs.SPIFFTX.bit.TXFFST)
        ;
    DELAY_US(1);
    for (i = 0; i < len; i++)
        SPIA_SendNonBlocking(buf[i] & 0xFF);
    while (SpiaRegs.SPIFFTX.bit.TXFFST)
        ;
    DELAY_US(1);
    FLASH_CS_HIGH;
}

// Uint16 W25Q64_ReadData(Uint32 addr, Uint16 len, Uint16 *buf)
// {
//     Uint16 i, position;
//     FLASH_CS_LOW;
//     DELAY_US(1);

//     W25Q64_WriteReadValue(CMD_READ);
//     W25Q64_WriteReadValue((addr >> 16) & 0xFF);
//     W25Q64_WriteReadValue((addr >> 8) & 0xFF);
//     W25Q64_WriteReadValue(addr & 0xFF);
//     // DELAY_US(1);
//     // position = spia_rx_head;
//     for (i = 0; i < len; i++)
//     {
//         buf[i] = W25Q64_WriteReadValue(CMD_DUMMY);
//     }
//     // for (i = 0; i < len-1; i++)
//     // {
//     //     buf[i] = ((buf[i]<<1)&0xff) | (((buf[i+1]&0x80) == 0) ? 0 : 1);
//     // }
//     // buf[i] = (buf[i] << 1) & 0xff;

//     // SPIA_SendNonBlocking(CMD_READ);
//     // SPIA_SendNonBlocking((addr >> 16) & 0xFF);
//     // SPIA_SendNonBlocking((addr >> 8) & 0xFF);
//     // SPIA_SendNonBlocking(addr & 0xFF);
//     // while (SpiaRegs.SPIFFTX.bit.TXFFST);
//     // DELAY_US(1);
//     // position = spia_rx_head;
//     // for (i = 0; i < len; i++)
//     // {
//     //     SPIA_SendNonBlocking(CMD_DUMMY);
//     //     buf[i] = spia_rx_buf[position];
//     //     position = (position + 1) & SPIA_TX_MASK;
//     // }
//     // while (SpiaRegs.SPIFFTX.bit.TXFFST);
//     // DELAY_US(1);
//     FLASH_CS_HIGH;
//     return position;
// }

void W25Q64_ReadData(Uint32 addr, Uint16 len, Uint16 *buf)
{
    Uint16 i;
    FLASH_CS_LOW;
    // SPIA_SendNonBlocking(CMD_READ);
    SPIA_SendNonBlocking(CMD_FAST_READ);
    SPIA_SendNonBlocking((addr >> 16) & 0xFF);
    SPIA_SendNonBlocking((addr >> 8) & 0xFF);
    SPIA_SendNonBlocking(addr & 0xFF);
    SPIA_SendNonBlocking(CMD_DUMMY);
    while (SpiaRegs.SPIFFTX.bit.TXFFST)
        ;
    DELAY_US(10);

    for (i = 0; i < len; i++)
    {
        buf[i] = spia_rx_buf[W25Q64_WriteReadValue(CMD_DUMMY)];
    }
    
    for (i = 0; i < len-1; i++)
    {
        buf[i] = ((buf[i]<<1)&0xff) | (((buf[i+1]&0x80) == 0) ? 0 : 1);
    }
    buf[i] = (buf[i] << 1) & 0xff;

    while (SpiaRegs.SPIFFTX.bit.TXFFST)
        ;
    DELAY_US(1);
    FLASH_CS_HIGH;
}
