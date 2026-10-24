#include "DSP2833x_Device.h"     // DSP2833x 头文件
#include "DSP2833x_Examples.h"   // DSP2833x 例子相关头文件
#include "leds.h"
#include "tim.h"
#include "uart_fifo.h"
#include "exti.h"
#include "printf.h"
#include "basic.h"
#include "spi_fifo.h"
#include "oled.h"
#include "ascii_font.h"
#include "oledOutput.h"
#include "flash.h"
#include "epwm_combine.h"
#include "ad760x.h"
#include "menu.h"
#include "basic_control.h"
#include "epwm_precharge.h"
#include "epwm_softstart.h"
#include "dma_xintf.h"

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);

// 需要烧写字库时打开这个宏，烧写完成后注释掉
// 执行python上位机发送字库之前需要在串口助手里等待接收到Hello World!的字符，因为擦写整个盘需要一定的时间，期间上位机发送的数据会丢失
// 没接收到Hello World!的话执行python代码会显示无法通讯
// #define FLASH_WRITE_MODE_OPEN

#ifdef FLASH_WRITE_MODE_OPEN
Uint16 writeBuf[256];
Uint32 writeAddr = 0x000000;
#endif

typedef enum {
    SYS_STATE_INIT = 0,     /* 外设就绪，PWM 未启动 */
    SYS_STATE_SOFTSTART,    /* 软启动进行中 */
    SYS_STATE_CLOSED_LOOP,  /* 正常闭环运行 */
    SYS_STATE_FAULT         /* 故障停机 */
} SystemState;

// 系统状态全局变量
volatile SystemState g_sysState = SYS_STATE_INIT;

static const char *SysStateName(SystemState s)
{
    switch (s)
    {
    case SYS_STATE_INIT:        return "INIT";
    case SYS_STATE_SOFTSTART:   return "SOFTSTART";
    case SYS_STATE_CLOSED_LOOP: return "CLOSED_LOOP";
    case SYS_STATE_FAULT:       return "FAULT";
    default:                    return "UNKNOWN";
    }
}

static void System_StartSoftStart(void)
{
    MOS_WORK_ENABLE;
    DELAY_US(1000);

    if (SoftStart_Start())
    {
        EPWM_ResetControlState();
        EPWM_SwitchToClosedLoop(1);
        g_sysState = SYS_STATE_SOFTSTART;
        printf("[SYS] Soft-Start launched\r\n");
    }
    else
    {
        EPWM_SafeStop();
        MOS_WORK_DISABLE;
        g_sysState = SYS_STATE_FAULT;
        printf("[SYS] Soft-Start init FAILED\r\n");
    }
}

static void System_EmergencyStop(void)
{
    EPWM_SafeStop();
    MOS_WORK_DISABLE;
    SoftStart_Reset();
    g_sysState = SYS_STATE_FAULT;
    printf("[SYS] EMERGENCY STOP\r\n");
}

static void System_Reset(void)
{
    EPWM_SafeStop();
    MOS_WORK_DISABLE;
    DELAY_US(10000);
    SoftStart_Reset();
    MMC_Init(P_MMC_array, N_MMC_array);
    K_value = 0;
    g_sysState = SYS_STATE_INIT;
    printf("[SYS] Reset to INIT\r\n");
}

/* ══════════════════════════════════════════════════════
 *                      串口命令处理
 * ══════════════════════════════════════════════════════ */

static void Cmd_PrintHelp(void)
{
    printf("\r\n===== COMMAND LIST =====\r\n");
    printf(" -- Control --\r\n");
    printf("  g : Go (start soft-start)\r\n");
    printf("  s : Stop (emergency)\r\n");
    printf("  r : Reset to INIT\r\n");
    printf("  t : Toggle MOS enable\r\n");
    printf(" -- Status --\r\n");
    printf("  i : System info\r\n");
    printf("  a : Soft-start status\r\n");
    printf("  v : Voltage samples\r\n");
    printf("  o : Sort order (position_id)\r\n");
    printf("  d : Partial status & stats\r\n");
    printf("  f : Frequency & PID status\r\n");
    printf(" -- Debug --\r\n");
    printf("  x : Clear statistics\r\n");
    printf("  h : This help\r\n");
    printf("========================\r\n");
}

static void Cmd_PrintSysInfo(void)
{
    printf("\r\n[SYS] state = %s\r\n", SysStateName(g_sysState));
    printf("[SYS] MOS    = %s\r\n",
           GpioDataRegs.GPADAT.bit.GPIO12 ? "DISABLED" : "ENABLED");
    printf("[SYS] K=%d  D=%.4f  U_in=%.1fV  U_load=%.1fV\r\n",
           K_value, g_partialDuty, U_in, U_load);
    printf("[SYS] TBPRD=%d (f_sub=%.0fHz f_full=%.0fHz)\r\n",
           EPwm1Regs.TBPRD,
           (float32)SYSCLK_HZ / (float32)EPwm1Regs.TBPRD,
           (float32)SYSCLK_HZ / (float32)EPwm1Regs.TBPRD / 2.0f);
}

static void Cmd_PrintVoltages(void)
{
    Uint16 i;

    /* ★ INIT 状态下先触发一次采样 ★
     * 闭环状态下 DMA 会持续更新缓冲区，不需要手动触发 */
    if (g_sysState == SYS_STATE_INIT)
    {
        /* 连续触发几次让数据稳定 */
        for (i = 0; i < 2; i++)
        {
            StartAD760x();
            DELAY_US(30);
        }
        printf("[V] (triggered fresh sample)\r\n");
    }

    printf("\r\n[V] P raw:");
    for (i = 0; i < MMC_NUMBER; i++)
        printf(" %5d", (int16)AD7606_1_Buf[i]);
    printf("\r\n[V] P mem:");
    for (i = 0; i < MMC_NUMBER; i++)
        printf(" %5d", (int16)P_MMC_array[i].recent_voltage);

    printf("\r\n[V] N raw:");
    for (i = 0; i < MMC_NUMBER; i++)
        printf(" %5d", (int16)AD7606_2_Buf[i]);
    printf("\r\n[V] N mem:");
    for (i = 0; i < MMC_NUMBER; i++)
        printf(" %5d", (int16)N_MMC_array[i].recent_voltage);

    printf("\r\n[V] U_in raw=%d  U_load raw=%d\r\n",
           (int16)AD7606_1_Buf[6], (int16)AD7606_2_Buf[6]);
}

static void Cmd_PrintSortOrder(void)
{
    Uint16 i;

    printf("\r\n[SORT] K=%d  D=%.4f\r\n", K_value, g_partialDuty);

    printf("[SORT] P pos :");
    for (i = 0; i < MMC_NUMBER; i++) printf(" %d", P_MMC_array[i].position_id);
    printf("\r\n[SORT] P last:");
    for (i = 0; i < MMC_NUMBER; i++) printf(" %d", P_MMC_array[i].last_position_id);
    printf("\r\n[SORT] P diff:");
    for (i = 0; i < MMC_NUMBER; i++) printf(" %5d", P_MMC_array[i].voltage_diff);

    printf("\r\n[SORT] N pos :");
    for (i = 0; i < MMC_NUMBER; i++) printf(" %d", N_MMC_array[i].position_id);
    printf("\r\n[SORT] N last:");
    for (i = 0; i < MMC_NUMBER; i++) printf(" %d", N_MMC_array[i].last_position_id);
    printf("\r\n[SORT] N diff:");
    for (i = 0; i < MMC_NUMBER; i++) printf(" %5d", N_MMC_array[i].voltage_diff);
    printf("\r\n");

    /* 校验：Partial 模块不应落在受限通道 */
    for (i = 0; i < MMC_NUMBER; i++)
    {
        if ((g_partialDuty > 0.0f) && (P_MMC_array[i].position_id == K_value))
        {
            if ((i == 0) || (i == 1))
                printf("[SORT] !! P Partial on RESTRICTED sm%d !!\r\n", i);
        }
        if ((g_partialDuty > 0.0f) && (N_MMC_array[i].position_id == K_value))
        {
            if ((i == 0) || (i == 1))
                printf("[SORT] !! N Partial on RESTRICTED sm%d !!\r\n", i);
        }
    }
}

static void Cmd_PrintPartial(void)
{
    printf("\r\n[PTL] D=%.4f  K=%d\r\n", g_partialDuty, K_value);
    printf("[PTL] active=%d (arm=%d gm=%d isA=%d sec=%d)\r\n",
           g_partialActive.active, g_partialActive.arm,
           g_partialActive.globalModule, g_partialActive.channelIsA,
           g_partialActive.secondCmpVal);
    printf("[PTL] prep  =%d (arm=%d gm=%d isA=%d f=%d s=%d)\r\n",
           g_partialPrepared.valid, g_partialPrepared.arm,
           g_partialPrepared.globalModule, g_partialPrepared.channelIsA,
           g_partialPrepared.firstCmp, g_partialPrepared.secondCmp);
    printf("[PTL] Tbase=%d  Dcrit=%.4f\r\n",
           g_partialTimeBase, g_partialDcrit);
    printf("[PTL] skip=%lu  miss=%lu\r\n",
           g_partialInjectSkipCnt, g_partialRestoreMissCnt);
}

static void Cmd_PrintFreqPID(void)
{
    printf("\r\n[PID] freq : out=%.0fHz  err=%.2f  U_load=%.1f\r\n",
           fre_PID.output, fre_PID.prev_error, U_load);
    printf("[PID] arm  : out=%.1f  Pavg=%.1f  Navg=%.1f\r\n",
           err_PID.output, arm_per_voltage_P, arm_per_voltage_N);
    printf("[PID] duty : out=%.5f  U_in=%.1f\r\n",
           duty_PID.output, U_in);
    printf("[PID] CMP  : base[0]=%d  err[0]=%d\r\n",
           CMP_Values[0], CMP_addError_Values[0]);
}

static void Cmd_ClearStats(void)
{
    g_partialInjectSkipCnt  = 0;
    g_partialRestoreMissCnt = 0;
    printf("[SYS] Statistics cleared\r\n");
}

void main()
{
    int16 i = 0;
    Uint16 rxData;
    // Uint16 rx_spi_data;
    // Uint16 id1, id2, id3;

    /*================================= 中断初始化配置 =================================*/
    InitSysCtrl();
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	InitPieVectTable();

    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash();

    LED_Init();
    TIM0_Init(150, 500000); // 0.5s(2Hz刷新率)
    // TIM1_Init(150, 500);  // 0.5s
    // TIM2_Init(150, 250000);  // 0.25s
    UARTb_FIFO_Init(115200);
    KEY_Init();
    EXTI3_Init();
    SPIA_Init_Advanced();

    // 已完成AD7606的初始化、DMA+XINTF、AD7606启动功能，具体在哪里启动、读取完成后的数据排序等需要在中断中完善
    // AD760x_GPIO_Protect(0); // 启动时先关断信号二极管，保护GPIO口，等DMA配置完成后再打开
    #ifndef FLASH_WRITE_MODE_OPEN
    AD760x_Init();
    
    // // 已完成6通道MMC的ePWM初始化，但只有基础模式，没有考虑带K的情况，也没有与ADC进行联动
    // // 计划完成ADC采样后继续完善

    // EPWM_ModuleInit();
    MosEnableGPIO_Init();
    MMC_Init(P_MMC_array, N_MMC_array);

    // TODO 这两个不注释掉的话直接执行EPWM_SwitchToClosedLoop(1);无法正常工作，到时候再找问题
    SoftStart_Init();
    GPIO_PrechargeInit();                             // 初始化为GPIO模式，等待预充电完成后再切换回EPWM模式

    #endif

    // EPWM_SwitchToClosedLoop(1);
    
	EINT;
    ERTM;
    

    /*================================= 外设初始化配置 =================================*/
    #ifdef FLASH_WRITE_MODE_OPEN
    // W25Q64必须在 SPI 模式0 才能通讯
    {
        // 写入字库前擦除整个盘
        SPIA_SetMode(FLASH_MODE);
        W25Q64_BulkErase();
        W25Q64_WaitBusy();
    }
    #endif


    #ifndef FLASH_WRITE_MODE_OPEN
    // OLED必须在 SPI 模式3 才能通讯
    MENU_Init();

    AD760x_Regester_Init(CHOOSE_ZONE0_ADDR);
    AD760x_Regester_Init(CHOOSE_ZONE6_ADDR);
    AD760x_Regester_Init(CHOOSE_ZONE7_ADDR);
    #endif

    /*================================= 外设功能测试 =================================*/
    // printf("Hello World!\r\n");
    // printf("You will enter a character, and the DSP will echo it back!\r\n");

    printf("\r\n\r\n");
    printf("========================================\r\n");
    printf("   MMC Controller (F28335) Ready\r\n");
    printf("   Press 'h' for command list\r\n");
    printf("========================================\r\n");

    #ifndef FLASH_WRITE_MODE_OPEN
    g_sysState = SYS_STATE_INIT;
    #endif

    while (1)
    {
        while (UARTb_FIFO_ReadByte(&rxData))
        {
            #ifndef FLASH_WRITE_MODE_OPEN
            // 处理接收到的数据,这里暂时简单地回显回去
            // 后续会根据指令进行不同的处理
            // printf("Echoed: %c\r\n", rxData);

            switch (rxData)
            {
            /* ===== 控制类 ===== */
            case 'g': case 'G':
                if (g_sysState == SYS_STATE_INIT)
                {
                    printf("[CMD] Start soft-start...\r\n");
                    System_StartSoftStart();
                }
                else
                {
                    printf("[CMD] Rejected: state=%s (need INIT)\r\n",
                           SysStateName(g_sysState));
                }
                break;

            case 's': case 'S':
                printf("[CMD] Emergency stop\r\n");
                System_EmergencyStop();
                break;

            case 'r': case 'R':
                printf("[CMD] Reset\r\n");
                System_Reset();
                break;

            case 't': case 'T':
                MOS_WORK_STATUS_TOGGLE;
                printf("[CMD] MOS -> %s\r\n",
                       GpioDataRegs.GPADAT.bit.GPIO12 ? "DISABLED" : "ENABLED");
                break;

            /* ===== 状态查询 ===== */
            case 'i': case 'I':  Cmd_PrintSysInfo();     break;
            case 'a': case 'A':  SoftStart_PrintStatus();break;
            case 'v': case 'V':  Cmd_PrintVoltages();    break;
            case 'o': case 'O':  Cmd_PrintSortOrder();   break;
            case 'd': case 'D':  Cmd_PrintPartial();     break;
            case 'f': case 'F':  Cmd_PrintFreqPID();     break;

            /* ===== 调试 ===== */
            case 'x': case 'X':  Cmd_ClearStats();       break;
            case 'h': case 'H':
            case '?':            Cmd_PrintHelp();        break;

            default:
                break;
            }
            #endif

            #ifdef FLASH_WRITE_MODE_OPEN
            // 烧写中文字库的程序
            // 烧写字库要把上面👆的printf注释掉，不然影响上位机接收“OK”
            writeBuf[i++] = rxData;
            if(i == 256)
            {
                W25Q64_PageProgram(writeAddr, writeBuf, PAGE_SIZE);
                W25Q64_WaitBusy();
                writeAddr += 0x0100;
                printf("OK");
                i = 0;
                // if(writeAddr == 0x000100)
                //     W25Q64_ReadData(0x000000, PAGE_SIZE, readBuf);
            }
            #endif
        }
        if(UARTb_FIFO_CheckOverflow())
        {
            printf("\r\n[Warning] RX Buffer Overflow! Data Lost!\r\n");
        }

        #ifndef FLASH_WRITE_MODE_OPEN
        /* ---------- 软启动异步消息打印 ---------- */
        SoftStart_ProcessMessages();

        /* ---------- 系统状态机 ---------- */
        switch (g_sysState)
        {
        case SYS_STATE_INIT:
            break;

        case SYS_STATE_SOFTSTART:
            if (SoftStart_IsComplete())
            {
                g_sysState = SYS_STATE_CLOSED_LOOP;
                printf("[SYS] -> CLOSED_LOOP\r\n");
            }
            /* 软启动期间的保护：restore 中断丢失 */
            // if (g_partialRestoreMissCnt > 10)
            // {
            //     printf("[SYS] FAULT: restore miss=%lu\r\n",
            //            g_partialRestoreMissCnt);
            //     System_EmergencyStop();
            // }
            break;

        case SYS_STATE_CLOSED_LOOP:
            /* 运行期保护 */
            // if (g_partialRestoreMissCnt > 100)
            // {
            //     printf("[SYS] FAULT: restore miss=%lu\r\n",
            //            g_partialRestoreMissCnt);
            //     System_EmergencyStop();
            // }
            break;

        case SYS_STATE_FAULT:
            LED1_ON;
            break;

        default:
            g_sysState = SYS_STATE_FAULT;
            break;
        }
        #endif

        switch (updataMeunFlag)
        {
        case NoUpdata:
            break;
        case CurrentUpdata:
            showMenu(menuManager.getCurrentMenu());
            updataMeunFlag = NoUpdata;
            break;
        case LastUpdata:
            showMenu(menuManager.getLastMenu());
            updataMeunFlag = NoUpdata;
            break;
        default:
            break;
        }   
    }
}
