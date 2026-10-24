#ifndef __MENU_H
#define __MENU_H

#include "DSP2833x_Device.h"     // DSP2833x 头文件
#include "DSP2833x_Examples.h"   // DSP2833x 例子相关头文件
#include "stdlib.h"

typedef struct
{
    char *caption;
    void (*mid)(void);
    void (*left)(void);
    void (*right)(void);
} MenuTypedef;

#define MAX_DISPLAY_LINE 4

typedef struct
{
    void (*append)(MenuTypedef *);
    MenuTypedef *(*getCurrentMenu)(void);
    MenuTypedef *(*getLastMenu)(void);
} MenuRecorderObject;

#define GO_BACK_MENU                                                                                       \
    {                                                                                                      \
        .caption = "< Back", .mid = goBackToLastMenu, .right = goBackToLastMenu, .left = goBackToLastMenu \
    }
#define END_OF_MENU                                               \
    {                                                             \
        .caption = NULL, .left = NULL, .right = NULL, .mid = NULL \
    }

extern MenuRecorderObject menuManager;
extern Uint16 currentChoose;
extern Uint16 updataMeunFlag;

extern MenuTypedef mainMenu[];
extern MenuTypedef UpperBridgeArmMenu[];
extern MenuTypedef PWMConfigMenu[];

extern Uint16 autoRefreshFlag;
extern Uint16 autoRefreshClass;

extern char menuCaptionStr1[20];
extern char menuCaptionStr2[20];
extern char menuCaptionStr3[20];
extern char menuCaptionStr4[20];

void keyUpOption(void);
void keyDownOption(void);
void keyLeftOption(void);
void keyMidOption(void);
void keyRightOption(void);
void MENU_Init(void);
void showMenu(MenuTypedef *menuNode);

typedef enum
{
    NoUpdata,
    CurrentUpdata,
    LastUpdata
} UpdataStatusChoose;

typedef enum
{
    NoChooseClass,
    UpperBridgeArmClass,
    LowerBridgeArmClass,
    PWMConfigClass
} RefreashClassChoose;

void goBackToLastMenu(void);
void gotoNextMenu(MenuTypedef *menu);

#endif /* __MENU_H */
