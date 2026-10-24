#include "oledOutput.h"
#include "stdlib.h"
#include "menu.h"
#include "oled.h"

#define MAX_MENU_RECODER 10
Uint16 currentStackIndex = 0;
MenuTypedef *menuStackRecoder[MAX_MENU_RECODER];
Uint16 autoRefreshFlag = 0;
Uint16 autoRefreshClass = NoChooseClass;

// 现在高亮显示的是整个菜单(不是屏幕)的第几个(一行占两个chooseLine)
Uint16 currentChoose = 0;
Uint16 updataMeunFlag = 0;

// 将最新一级的菜单推入栈中
void pushMenu(MenuTypedef *menu)
{
    if (currentStackIndex < MAX_MENU_RECODER)
        menuStackRecoder[currentStackIndex++] = menu;
}

// 将最新一级菜单从栈中弹出，返回上一级的菜单
MenuTypedef *popMenu(void)
{
    if (currentStackIndex > 0)
        return menuStackRecoder[--currentStackIndex - 1];
    else
        return NULL;
}

// 获得当前栈顶的菜单，即当前显示的菜单
MenuTypedef *getCurrentStackValue(void)
{
    return menuStackRecoder[currentStackIndex - 1];
}

// 菜单的入栈、出栈 (即显示上一栈) 和显示当前栈
MenuRecorderObject menuManager = {
    .append = pushMenu,
    .getLastMenu = popMenu,
    .getCurrentMenu = getCurrentStackValue};

void MENU_Init(void)
{
    OLED_Init();
    chooseLine = 0;
    menuManager.append(mainMenu);
    updataMeunFlag = CurrentUpdata;
}

void showMenu(MenuTypedef *menuNode)
{
    Uint16 totalLine = 1;
    screenClear();
    // 先让高亮的行确定，此时会显示在屏幕的第一行
    menuNode += currentChoose;
    // 再将高亮的行挪到屏幕应该在的位置
    menuNode -= chooseLine / 2;
    totalLine = MAX_DISPLAY_LINE;
    while ((totalLine--) && ((menuNode->caption) != NULL))
    {
        OLED_printf("%s\n", menuNode->caption);
        menuNode++;
    }
}


#define currentMenu (menuManager.getCurrentMenu())


void keyUpOption(void)
{
    if (currentChoose > 0)
    {
        currentChoose--;
        if (chooseLine > 0)
            chooseLine -= 2;
        if(~autoRefreshFlag)
            updataMeunFlag = CurrentUpdata;
    }
}

void keyDownOption(void)
{
    if (currentMenu[currentChoose + 1].caption != NULL)
    {
        currentChoose++;
        if (chooseLine / 2 < MAX_DISPLAY_LINE - 1)
            chooseLine += 2;
        if(~autoRefreshFlag)
            updataMeunFlag = CurrentUpdata;
    }
}

void keyLeftOption(void)
{
    if (currentMenu[currentChoose].left != NULL)
        currentMenu[currentChoose].left();
}

void keyMidOption(void)
{
    if (currentMenu[currentChoose].mid != NULL)
        currentMenu[currentChoose].mid();
}

void keyRightOption(void)
{
    if (currentMenu[currentChoose].right != NULL)
        currentMenu[currentChoose].right();
}

// 进入选定的某个下级目录
void gotoNextMenu(MenuTypedef *menu)
{
    currentChoose = chooseLine = 0;
    menuManager.append(menu);
    if(~autoRefreshFlag)
        updataMeunFlag = CurrentUpdata;
}

// 返回上级目录
void goBackToLastMenu(void)
{
    chooseLine = currentChoose = 0;
    if(~autoRefreshFlag)
        updataMeunFlag = LastUpdata;
}
