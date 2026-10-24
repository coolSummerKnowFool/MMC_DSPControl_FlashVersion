#include "oled.h"
#include "spi_fifo.h"
#include "basic.h"

void sendCMD(Uint16 cmd)
{
    OLED_DC_CMD;
    OLED_CS_LOW;
    SPIA_SendNonBlocking(cmd);
    // for (Uint16 i = 0; i < 8; i++)
    // {
    //     OLED_CLK_LOW;
    //     if (cmd & 0x80)
    //         OLED_MOSI_HIGH;
    //     else
    //         OLED_MOSI_LOW;
        
    //     OLED_CLK_HIGH;
    //     cmd <<= 1;
    // }
    DELAY_US(3);
    OLED_CS_HIGH;
}

void sendData(Uint16 dat)
{
    OLED_DC_DATA;
    OLED_CS_LOW;
    SPIA_SendNonBlocking(dat);
    // for (Uint16 i = 0; i < 8; i++)
    // {
    //     OLED_CLK_LOW;
    //     if (dat & 0x80)
    //         OLED_MOSI_HIGH;
    //     else
    //         OLED_MOSI_LOW;
    //     OLED_CLK_HIGH;
    //     dat <<= 1;
    // }
    DELAY_US(3);
    OLED_CS_HIGH;
}

void oledReset(void)
{
    const Uint16 rstCmd[] ={0X8D, 0X10, 0XAE, 0xAE, 0x02, 0x10, 0x40, 0x81,
                            0xCF, 0xA1, 0xC8, 0xA6, 0xA8, 0x3f, 0xD3, 0x00,
                            0xd5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB, 0x40,
                            0x20, 0x02, 0x8D, 0x14, 0xA4, 0xA6, 0xAF, 0X8D,
                            0X14, 0XAF};

    OLED_RES_HIGH;
    DELAY_US((long)100000);
    OLED_RES_LOW;
    DELAY_US((long)100000);
    OLED_RES_HIGH;

    for (Uint16 cnt = 0; cnt < 34; cnt++)
        sendCMD(rstCmd[cnt]);
}

void oledReset_test(void)
{
    const Uint16 rstCmd[] ={0X8D, 0X10, 0XAE, 0xAE, 0x02, 0x10, 0x40, 0x81,
                            0xCF, 0xA1, 0xC8, 0xA6, 0xA8, 0x3f, 0xD3, 0x00,
                            0xd5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB, 0x40,
                            0x20, 0x02, 0x8D, 0x14, 0xA4, 0xA6, 0xAF, 0X8D,
                            0X14, 0XAF};
    OLED_RES_HIGH;
    DELAY_US((long)100000);
    OLED_RES_LOW;
    DELAY_US((long)100000);
    OLED_RES_HIGH;

    OLED_DC_CMD;
    OLED_CS_LOW;
    for (Uint16 cnt = 0; cnt < 34; cnt++)
        SPIA_SendNonBlocking(rstCmd[cnt]);
    
    while (SpiaRegs.SPIFFTX.bit.TXFFST); // 等待发送完成
    DELAY_US(1);
    OLED_CS_HIGH;
}

// 填充整个屏幕
// 0x0000 全黑  0xFFFF 全白
void OLED_FillScreen(Uint16 data)
{
    Uint16 i, n;
    for (i = 0; i < 8; i++)
    {
        sendCMD(0xB0 | i); // 设置页地址（0~7）
        sendCMD(0x02);     // 设置显示位置—列低地址
        sendCMD(0x10);     // 设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            sendData(data); // 填充数据
    } // 更新显示
}

void OLED_Filltest(Uint16 data)
{
    Uint16 i, n;
    OLED_CS_LOW;
    for (i = 0; i < 8; i++)
    {
        OLED_DC_CMD;
        SPIA_SendNonBlocking(0xB0 | i);
        SPIA_SendNonBlocking(0x02);
        SPIA_SendNonBlocking(0x10);
        while (SpiaRegs.SPIFFTX.bit.TXFFST);
        DELAY_US(1);

        OLED_DC_DATA;
        for (n = 0; n < 128; n++)
            SPIA_SendNonBlocking(data); // 填充数据
        
        while (SpiaRegs.SPIFFTX.bit.TXFFST);
        DELAY_US(1);
    } // 更新显示
    OLED_CS_HIGH;
}

void setCol(Uint16 col)
{
    // 设置列地址的高四位指令： 0001 **** (前四位是固定值，指令范围:10~1F)
    // 设置列地址的低四位指令： 0000 **** (前四位是固定值，指令范围:00~0F)
    sendCMD(col & 0x0f);            // 设置显示位置—列低地址
    sendCMD(0x10 | (col >> 4));     // 设置显示位置—列高地址
}

void setPage(Uint16 page)
{
    // 设置行地址指令： 1011 0***
    // 屏幕共64行，每页8行，共8页
    sendCMD(0xB0 | page);
}

/**
 * @brief 在OLED上显示字模
 *
 * @param ptr_pic 要显示的字模首地址，有const属性，表示只读
 * @param colStart 要在屏幕上显示的首列地址
 * @param pageStart 要在屏幕上显示的首行地址
 * @param line 要显示的字模高度
 * @param col 要显示的字模宽度
 */
void Picture_display(const Uint16 *ptr_pic, Uint16 colStart, Uint16 pageStart, Uint16 line, Uint16 col)
{
    Uint16 page, column;
    OLED_CS_LOW;
    for (page = pageStart; page < pageStart + (line / 8); page++) // page loop
    {
        OLED_DC_CMD;
        SPIA_SendNonBlocking(0xB0 | page);
        SPIA_SendNonBlocking(0X10 | (colStart>>4));
        SPIA_SendNonBlocking(colStart & 0x0f);
        while (SpiaRegs.SPIFFTX.bit.TXFFST);
        DELAY_US(1);

        OLED_DC_DATA;
        for (column = 0; column < col; column++)
            SPIA_SendNonBlocking(*ptr_pic++); // 填充数据
        
        while (SpiaRegs.SPIFFTX.bit.TXFFST);
        DELAY_US(1);
    }
    OLED_CS_HIGH;
}

/**
 * @brief 将所选中的行取反(高亮显示)
 *
 * @param ptr_pic 图像首地址
 * @param colStart 开始列
 * @param pageStart 开始页
 * @param line 高度
 * @param col 宽度
 */
void PictureContrastDisplay(const Uint16 *ptr_pic, Uint16 colStart, Uint16 pageStart, Uint16 line, Uint16 col)
{
    Uint16 page, column;
    OLED_CS_LOW;
    for (page = pageStart; page < pageStart + (line / 8); page++) // page loop
    {
        OLED_DC_CMD;
        SPIA_SendNonBlocking(0xB0 | page);
        SPIA_SendNonBlocking(0X10 | (colStart>>4));
        SPIA_SendNonBlocking(colStart & 0x0f);
        while (SpiaRegs.SPIFFTX.bit.TXFFST);
        DELAY_US(1);

        OLED_DC_DATA;
        for (column = 0; column < col; column++)
            SPIA_SendNonBlocking(~*ptr_pic++); // 填充数据
        
        while (SpiaRegs.SPIFFTX.bit.TXFFST);
        DELAY_US(1);
    }
    OLED_CS_HIGH;
}

void OLED_Init(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1; // 开启GPIO时钟

    // OLED RES - GPIO57 （低电平复位）
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0; // 设置为通用GPIO功能
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;  // 设置GPIO方向为输出
    GpioCtrlRegs.GPBPUD.bit.GPIO52 = 1;  // 不使能GPIO上拉电阻

    // OLED DC - GPIO53 （低电平0：发送命令；高电平1：发送数据）
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0; // 设置为通用GPIO功能
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = 1;  // 设置GPIO方向为输出
    GpioCtrlRegs.GPBPUD.bit.GPIO53 = 1;  // 不使能GPIO上拉电阻

    GpioDataRegs.GPBSET.bit.GPIO52 = 1; // OLED RES 拉高
    GpioDataRegs.GPBSET.bit.GPIO53 = 1; // OLED DC 拉高

    // GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0; // 设置为通用GPIO功能
    // GpioCtrlRegs.GPBDIR.bit.GPIO54 = 1;  // 设置GPIO方向为输出
    // GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;  // 不使能GPIO上拉电阻
    // GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0; // 设置为通用GPIO功能
    // GpioCtrlRegs.GPBDIR.bit.GPIO56 = 1;  // 设置GPIO方向为输出
    // GpioCtrlRegs.GPBPUD.bit.GPIO56 = 1;  // 不使能GPIO上拉电阻
    EDIS;

    // SPIA_CS_Init();

    // oledReset();
    oledReset_test();

    // OLED_FillScreen(0xff); // 全屏白
    // OLED_FillScreen(0x00); // 全屏黑
    OLED_Filltest(0x00); // 全屏黑
}
