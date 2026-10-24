#ifndef __OLEDOUTPUT_H
#define __OLEDOUTPUT_H

#include "DSP2833x_Device.h"   // DSP2833x 头文件
#include "DSP2833x_Examples.h" // DSP2833x 例子相关头文件
#include "ascii_font.h"
#include "oled.h"
#include "stdlib.h"
#include "printf.h"
#include "flash.h"

extern int16 chooseLine;
extern char oled_str[];

// ...表示可变参数，__VA_ARGS__就是将...的值复制到这里
// ##__VA_ARGS__前面加上##的作用是：
// 当可变参数的个数为0时，这里的##可以把把前面多余的","去掉,否则会编译出错。
// 
// sprintf指的是字符串格式化命令，
// 函数声明为 int sprintf(char *string, char *format [,argument,…]);
// 主要功能是把格式化的数据写入某个字符串中，即发送格式化输出到 string 所指向的字符串。
// sprintf 是个变参函数。使用sprintf 对于写入buffer的字符数是没有限制的，这就存在了buffer溢出的可能性。
// 解决这个问题，可以考虑使用 snprintf函数，该函数可对写入字符数做出限制。
// ex: sprintf(str,"%6.2f",f);
#define OLED_printf(...)              \
    sprintf(oled_str, ##__VA_ARGS__); \
    OLED_print(oled_str)

void OLED_print(char *str);
void OLED_putchar(Uint16 ch);
void screenClear(void);

#endif /* __OLEDOUTPUT_H */
