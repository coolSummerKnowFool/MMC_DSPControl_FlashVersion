#include "oledOutput.h"
#include "spi_fifo.h"

// oled上所指的行数
Uint16 line = 0;
// 当前行已经显示的字符数
Uint16 wordCount = 0;
// 高亮的位置在屏幕的第几行
int16 chooseLine = -1;
char oled_str[32] = {0};

void screenClear(void)
{
	OLED_Filltest(0x00);
	line = wordCount = 0;
}

// utf-8格式:
//  Byte1	  Byte2	   Byte3
// 1110xxxx	10xxxxxx 10xxxxxx

// Unicode格式:
//  Byte1	 Byte2
// xxxxxxxx	xxxxxxxx

/**
 * @brief 将utf-8编码的*中文*转换为Unicode格式
 *
 * @param c utf-8编码格式中文(三个字节表示)的首地址
 * @return uint16_t Unicode格式的中文
 */
Uint16 utf8ToUnicode(Uint16 *c)
{
	Uint16 result = 0;
	Uint16 tmp;
	// 0x 0011 1111
	result += c[2] & 0x3f;
	tmp = c[1] & 0x3f;
	result += (tmp << 6);
	tmp = c[0] & 0x0f;
	result += (tmp << 12);
	return result;
}

// 除了u4e00-u9fa5 (中文)之外，还有(0x3400, 0x4DB5)也是。
// GB18030在GB13000.1的20902个汉字的基础上增加了CJK扩展A的6582个汉字(Unicode码0x3400-0x4db5)
// 一共收录了27484个汉字。
#define ZhStart 0x3400
#define MidLen 0x19b5
// UTF-8 (Unicode)
// - u4e00-u9fa5 (中文)
#define Zh2Start 0x4e00

Uint16 zhBuf[3];
Uint16 zhCnt = 0;

#define MAX_LINE 8
#define MAX_WORD 16

#define ZHSIZE 32
Uint16 zhData[ZHSIZE] = {0};

// /**
//  * @brief 黑底白字在屏幕上打印字模
//  *
//  * @param ch 字符utf-8编码的每一个字节依次传入
//  */
void OLED_putchar(Uint16 ch)
{
	ch = ch & 0xff;
	if ((wordCount >= MAX_WORD) && ch != '\n')
	{
		wordCount = 0;
		line += asciiHigh / 8;
	}

	if (zhCnt)
	{
		zhBuf[zhCnt++] = ch;
		if (zhCnt == 3)
		{
			zhCnt = 0;
			Uint16 zh = utf8ToUnicode(zhBuf);
			// FLASH从0x000000地址开始存储汉字的字模数据
			// 但0x3400, 0x4DB5和u4e00-u9fa5之间的数据没有存入FLASH，
			// 所以根据Unicode算汉字在FLASH中的地址时需要进行换算
			if (zh - ZhStart > MidLen)
			{
				SPIA_SetMode(FLASH_MODE);
				W25Q64_ReadData((Uint32)(MidLen + zh - Zh2Start + 1) << 5, ZHSIZE, zhData);
				SPIA_SetMode(OLED_MODE);
			}
			else
			{
				SPIA_SetMode(FLASH_MODE);
				W25Q64_ReadData((Uint32)(zh - ZhStart) << 5, ZHSIZE, zhData);
				SPIA_SetMode(OLED_MODE);
			}
			Picture_display(zhData, 2 + wordCount * asciiWide, line, 16, 16);
			wordCount += 2;
		}
		return;
	}
	// 判断utf-8的中文的第一位，之后两个个字节将进入👆
	if ((ch & 0xf0) == 0xe0)
	{
		zhBuf[zhCnt++] = ch;
		return;
	}

	// 当 ch<=127 时进入下面的switch中
	switch (ch)
	{
	case '\n':
		line += asciiHigh / 8;
		wordCount = 0;
		return;
	case '\b':
		wordCount--;
		break;
	case 127:
		break;
	// ASCLL码中32-126为可视的图形,储存在 font.c 中
	default:
		if (ch >= 32)
			Picture_display(ASCII[ch - 32], 2 + wordCount * asciiWide, line, asciiHigh, asciiWide);
		break;
	}
	wordCount++;
}

/**
 * @brief 黑底白字高亮显示
 *
 * @param ch 字符utf-8编码的每一个字节依次传入
 */
void OLED_putContrastChar(Uint16 ch)
{
	ch = ch & 0xff;
	if (wordCount >= 16)
	{
		wordCount = 0;
		line += asciiHigh / 8;
	}

	if (zhCnt)
	{
		zhBuf[zhCnt++] = ch;
		if (zhCnt == 3)
		{
			zhCnt = 0;
			Uint16 zh = utf8ToUnicode(zhBuf);
			// FLASH从0x000000地址开始存储汉字的字模数据
			// 但0x3400, 0x4DB5和u4e00-u9fa5之间的数据没有存入FLASH，
			// 所以根据Unicode算汉字在FLASH中的地址时需要进行换算
			if (zh - ZhStart > MidLen)
			// zh - Zh2Start + MidLen + 1
			{
				SPIA_SetMode(FLASH_MODE);
				W25Q64_ReadData((Uint32)(MidLen + zh - Zh2Start + 1) << 5, ZHSIZE, zhData);
				SPIA_SetMode(OLED_MODE);
			}
			else
			{
				SPIA_SetMode(FLASH_MODE);
				W25Q64_ReadData((Uint32)(zh - ZhStart) << 5, ZHSIZE, zhData);
				SPIA_SetMode(OLED_MODE);
			}
			PictureContrastDisplay(zhData, 2 + wordCount * asciiWide, line, 16, 16);
			wordCount += 2;
		}
		return;
	}
	// 判断utf-8的中文的第一位，之后两个个字节将进入👆
	if ((ch & 0xf0) == 0xe0)
	{
		zhBuf[zhCnt++] = ch;
		return;
	}

	switch (ch)
	{
	case '\n':
		line += asciiHigh / 8;
		wordCount = 0;
		return;
	case '\b':
		wordCount--;
		break;
	case 127:
		break;
	default:
		if (ch >= 32)
			PictureContrastDisplay(ASCII[ch - 32], 2 + wordCount * asciiWide, line, asciiHigh, asciiWide);
		break;
	}
	wordCount++;
}

/**
 * @brief 将字符串打印在屏幕上
 *
 * @param str 字符串的首地址(注意:不要传入字符)
 */
void OLED_print(char *str)
{
	if (chooseLine == line)
		while (*str)
			OLED_putContrastChar(*str++);
	else
		while (*str)
			OLED_putchar(*str++);
}
