#include "menu.h"
#include "tim.h"
#include "ad760x.h"
#include "basic_control.h"

char menuCaptionStr1[20];
char menuCaptionStr2[20];
char menuCaptionStr3[20];
char menuCaptionStr4[20];

void goToUpperBridgeArmMenu(void)
{
    gotoNextMenu(UpperBridgeArmMenu);
    autoRefreshFlag = 1;
    autoRefreshClass = UpperBridgeArmClass;
    TIM0_Start();
}

void goToPWMConfigMenu(void)
{
    gotoNextMenu(PWMConfigMenu);
    autoRefreshFlag = 1;
    autoRefreshClass = PWMConfigClass;
    TIM0_Start();
}

void goBackFromAutoRefreshMenu(void)
{
    TIM0_Stop();
    autoRefreshFlag = 0;
    autoRefreshClass = NoChooseClass;
    goBackToLastMenu();
}

MenuTypedef mainMenu[] = {
    {.caption = "上桥臂ADC", .left = NULL, .mid = goToUpperBridgeArmMenu, .right = goToUpperBridgeArmMenu},
    {.caption = "下桥臂ADC", .left = NULL, .mid = NULL, .right = NULL},
    {.caption = "PWM配置", .left = NULL, .mid = goToPWMConfigMenu, .right = goToPWMConfigMenu},
    {.caption = "功能4", .left = NULL, .mid = NULL, .right = NULL},
    {.caption = "功能5", .left = NULL, .mid = NULL, .right = NULL},
    {.caption = "功能6", .left = NULL, .mid = NULL, .right = NULL},
    {.caption = "功能7", .left = NULL, .mid = NULL, .right = NULL},
    END_OF_MENU};


MenuTypedef UpperBridgeArmMenu[] = {
    {.caption = menuCaptionStr1, .left = goBackFromAutoRefreshMenu, .mid = NULL, .right = StartAD760x},
    {.caption = menuCaptionStr2, .left = goBackFromAutoRefreshMenu, .mid = NULL, .right = StartAD760x},
    {.caption = menuCaptionStr3, .left = goBackFromAutoRefreshMenu, .mid = NULL, .right = StartAD760x},
    {.caption = menuCaptionStr4, .left = goBackFromAutoRefreshMenu, .mid = NULL, .right = StartAD760x},
    END_OF_MENU};

MenuTypedef PWMConfigMenu[] = {
    {.caption = menuCaptionStr1, .left = MosWorkStatusChange, .mid = NULL, .right = MosWorkStatusChange},
    {.caption = menuCaptionStr2, .left = goBackFromAutoRefreshMenu, .mid = NULL, .right = NULL},
    {.caption = "返回", .left = goBackFromAutoRefreshMenu, .mid = NULL, .right = NULL},
    END_OF_MENU};


