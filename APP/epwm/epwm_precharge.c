#include "epwm_precharge.h"
#include "epwm_combine.h"
#include "dma_xintf.h"
#include "ad760x.h"
#include "printf.h"
#include "leds.h"

// ==================== 全局变量 ====================
volatile PrechargeManager g_precharge = {
    .state = PRECHARGE_IDLE,
    .targetN = 6,
    .K_value = 0,
    .currentBalanceLevel = TOTAL_SM_COUNT,
    .iterationCount = 0,
    .phase = BALANCE_PHASE_CHARGE,
    .maxVoltage = 0,
    .minVoltage = 0,
    .voltageSpread = 0
};

// ePWM寄存器指针
static volatile struct EPWM_REGS *const ePWM_regs[6] = {
    &EPwm1Regs, &EPwm2Regs, &EPwm3Regs,
    &EPwm4Regs, &EPwm5Regs, &EPwm6Regs
};

/*
 * GPIO与ePWM引脚映射：
 * ePWM1A -> GPIO0,  ePWM1B -> GPIO1   (上桥臂 SM0, SM1)
 * ePWM2A -> GPIO2,  ePWM2B -> GPIO3   (上桥臂 SM2, SM3)
 * ePWM3A -> GPIO4,  ePWM3B -> GPIO5   (上桥臂 SM4, SM5)
 * ePWM4A -> GPIO6,  ePWM4B -> GPIO7   (下桥臂 SM0, SM1)
 * ePWM5A -> GPIO8,  ePWM5B -> GPIO9   (下桥臂 SM2, SM3)
 * ePWM6A -> GPIO10, ePWM6B -> GPIO11  (下桥臂 SM4, SM5)
 */

// ==================== 私有函数声明 ====================
static void Precharge_ApplyBalanceStrategy(void);
static Uint16 Precharge_CheckBalanceComplete(void);
static void Precharge_SetFinalState(void);
static void Precharge_InsertionSort(volatile PrechargeSmInfo *arr, Uint16 n);

// ==================== GPIO初始化（预充电模式） ====================
void GPIO_PrechargeInit(void)
{
    EALLOW;
    
    // GPIO0-11 配置为通用GPIO输出模式
    // 上桥臂: GPIO0-5
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;     // GPIO模式
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;
    
    // 下桥臂: GPIO6-11
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;
    
    // 设置为输出
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;
    
    // 禁用上拉
    GpioCtrlRegs.GPAPUD.all |= 0x00000FFF;  // GPIO0-11禁用上拉
    
    // 初始状态：全部置高（所有子模块接入）
    GpioDataRegs.GPASET.all = 0x00000FFF;   // GPIO0-11置高
    
    EDIS;
    
    // 初始化管理器
    // g_precharge.state = PRECHARGE_IDLE;
    // g_precharge.iterationCount = 0;
    // g_precharge.K_value = 0;
    // g_precharge.targetN = 6;
    // g_precharge.currentBalanceLevel = TOTAL_SM_COUNT;

    g_ADCIsrFlag = 0; // 预充电期间不执行ADC的中断内的桥臂控制函数
    
    // 初始化子模块列表
    // for (Uint16 i = 0; i < UPPER_ARM_COUNT; ++i)
    // {
    //     g_precharge.smList[i].smIndex = i;
    //     g_precharge.smList[i].isUpperArm = 1;
    //     g_precharge.smList[i].voltage = 0;
    //     g_precharge.upperVoltages[i] = 0;
    //     g_precharge.upperSortedIdx[i] = i;
    // }
    // for (Uint16 i = 0; i < LOWER_ARM_COUNT; ++i)
    // {
    //     g_precharge.smList[UPPER_ARM_COUNT + i].smIndex = i;
    //     g_precharge.smList[UPPER_ARM_COUNT + i].isUpperArm = 0;
    //     g_precharge.smList[UPPER_ARM_COUNT + i].voltage = 0;
    //     g_precharge.lowerVoltages[i] = 0;
    // }
}

// ==================== 设置上桥臂子模块状态 ====================
void GPIO_SetUpperSM(Uint16 smIndex, SubmoduleState state)
{
    if (smIndex >= UPPER_ARM_COUNT) return;
    
    // GPIO0-5 对应上桥臂 SM0-5
    Uint32 mask = 1UL << smIndex;
    
    if (state == SM_INSERTED)
        GpioDataRegs.GPASET.all = mask;     // 置高
    else
        GpioDataRegs.GPACLEAR.all = mask;   // 置低
}

// ==================== 设置下桥臂子模块状态 ====================
void GPIO_SetLowerSM(Uint16 smIndex, SubmoduleState state)
{
    if (smIndex >= LOWER_ARM_COUNT) return;
    
    // GPIO6-11 对应下桥臂 SM0-5
    Uint32 mask = 1UL << (smIndex + 6);
    
    if (state == SM_INSERTED)
        GpioDataRegs.GPASET.all = mask;
    else
        GpioDataRegs.GPACLEAR.all = mask;
}

// ==================== 设置所有上桥臂 ====================
void GPIO_SetAllUpperArm(SubmoduleState state)
{
    // GPIO0-5
    if (state == SM_INSERTED)
        GpioDataRegs.GPASET.all = 0x0000003F;
    else
        GpioDataRegs.GPACLEAR.all = 0x0000003F;
}

// ==================== 设置所有下桥臂 ====================
void GPIO_SetAllLowerArm(SubmoduleState state)
{
    // GPIO6-11
    if (state == SM_INSERTED)
        GpioDataRegs.GPASET.all = 0x00000FC0;
    else
        GpioDataRegs.GPACLEAR.all = 0x00000FC0;
}

// ==================== 采样所有子模块电压 ====================
void Precharge_SampleAllVoltages(void)
{
    StartAD760x();
    DELAY_US(AD7606_CONVERSION_TIME_US);
    
    for (Uint16 i = 0; i < UPPER_ARM_COUNT; ++i)
    {
        g_precharge.upperVoltages[i] = (int16)AD7606_1_Buf[i];
        g_precharge.smList[i].voltage = g_precharge.upperVoltages[i];
    }
    for (Uint16 i = 0; i < LOWER_ARM_COUNT; ++i)
    {
        g_precharge.lowerVoltages[i] = (int16)AD7606_2_Buf[i];
        g_precharge.smList[UPPER_ARM_COUNT + i].voltage = g_precharge.lowerVoltages[i];
    }
}

// ==================== 计算目标N值 ====================
Uint16 Precharge_CalculateN(void)
{
    double u_in = AD7606_1_Buf[6] / 32768.0f * 1505.0f;
    
    int k = Get_K_FloorMethod(u_in);
    if (k < 0) k = 0;
    if (k > UPPER_ARM_COUNT) k = UPPER_ARM_COUNT;
    
    g_precharge.K_value = (Uint16)k;
    g_precharge.targetN = LOWER_ARM_COUNT + k;
    
    K_value = g_precharge.K_value;
    U_in = u_in;
    
    printf("[Precharge] U_in=%.1fV, K=%d, targetN=%d\r\n", u_in, k, g_precharge.targetN);
    
    return g_precharge.targetN;
}

// ==================== 插入排序（升序） ====================
static void Precharge_InsertionSort(volatile PrechargeSmInfo *arr, Uint16 n)
{
    for (Uint16 i = 1; i < n; ++i)
    {
        PrechargeSmInfo key = arr[i];
        int16 j = (int16)i - 1;
        
        while (j >= 0 && arr[j].voltage > key.voltage)
        {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

// ==================== 对全部12个子模块按电压升序排序 ====================
void Precharge_SortAllByVoltage(void)
{
    Precharge_InsertionSort(g_precharge.smList, TOTAL_SM_COUNT);
    
    g_precharge.minVoltage = g_precharge.smList[0].voltage;
    g_precharge.maxVoltage = g_precharge.smList[TOTAL_SM_COUNT - 1].voltage;
    g_precharge.voltageSpread = g_precharge.maxVoltage - g_precharge.minVoltage;
}

// ==================== 对上桥臂按电压升序排序 ====================
static void Precharge_SortUpperArm(void)
{
    // 直接对upperSortedIdx排序，无需初始化（保持上次结果作为初态更优）
    for (Uint16 i = 1; i < UPPER_ARM_COUNT; ++i)
    {
        Uint16 keyIdx = g_precharge.upperSortedIdx[i];
        int16 keyVoltage = g_precharge.upperVoltages[keyIdx];
        int16 j = (int16)i - 1;
        
        while (j >= 0)
        {
            Uint16 cmpIdx = g_precharge.upperSortedIdx[j];
            if (g_precharge.upperVoltages[cmpIdx] > keyVoltage)
            {
                g_precharge.upperSortedIdx[j + 1] = g_precharge.upperSortedIdx[j];
                j--;
            }
            else
            {
                break;
            }
        }
        g_precharge.upperSortedIdx[j + 1] = keyIdx;
    }
}

// ==================== 检查均压是否完成 ====================
static Uint16 Precharge_CheckBalanceComplete(void)
{
    return (g_precharge.voltageSpread <= PRECHARGE_VOLTAGE_TOLERANCE) ? 1 : 0;
}

// ==================== 应用均压策略（优化：交替开通/关断） ====================
static void Precharge_ApplyBalanceStrategy(void)
{
    Uint16 balanceLevel = g_precharge.currentBalanceLevel;
    Uint16 bypassCount = TOTAL_SM_COUNT - balanceLevel;
    
    // smList已按电压升序排列：[0]最低 ... [11]最高
    
    // 先确定每个子模块的目标状态
    SubmoduleState targetStates[TOTAL_SM_COUNT];
    
    if (g_precharge.phase == BALANCE_PHASE_CHARGE)
    {
        // 充电阶段：接入电压低的，旁路电压高的
        for (Uint16 i = 0; i < TOTAL_SM_COUNT; ++i)
        {
            targetStates[i] = (i < balanceLevel) ? SM_INSERTED : SM_BYPASSED;
        }
    }
    else
    {
        // 放电阶段：旁路电压低的，接入电压高的
        for (Uint16 i = 0; i < TOTAL_SM_COUNT; ++i)
        {
            targetStates[i] = (i < bypassCount) ? SM_BYPASSED : SM_INSERTED;
        }
    }
    
    // 交替执行：先关断一个需要旁路的，再开通一个需要接入的
    // 这样可以保持接入数量的稳定，避免瞬态不平衡
    
    // 找到需要接入和需要旁路的子模块
    // TODO：这个数组的大小是不是可以设置为6啊？
    Uint16 toInsert[UPPER_ARM_COUNT];
    Uint16 toBypass[UPPER_ARM_COUNT];
    Uint16 insertCount = 0;
    Uint16 bypassCountActual = 0;
    
    for (Uint16 i = 0; i < TOTAL_SM_COUNT; ++i)
    {
        volatile PrechargeSmInfo *sm = &g_precharge.smList[i];
        
        // 获取当前GPIO状态
        Uint32 currentState;
        if (sm->isUpperArm)
        {
            currentState = (GpioDataRegs.GPADAT.all >> sm->smIndex) & 0x01;
        }
        else
        {
            currentState = (GpioDataRegs.GPADAT.all >> (sm->smIndex + 6)) & 0x01;
        }
        
        SubmoduleState currentSM = currentState ? SM_INSERTED : SM_BYPASSED;
        
        if (targetStates[i] == SM_INSERTED && currentSM == SM_BYPASSED)
        {
            toInsert[insertCount++] = i;
        }
        else if (targetStates[i] == SM_BYPASSED && currentSM == SM_INSERTED)
        {
            toBypass[bypassCountActual++] = i;
        }
    }
    
    // 确定循环次数
    Uint16 maxOps = (insertCount > bypassCountActual) ? insertCount : bypassCountActual;
    
    // 交替执行：旁路一个，接入一个
    for (Uint16 op = 0; op < maxOps; ++op)
    {
        // 先旁路一个（减少接入数量）
        if (op < bypassCountActual)
        {
            Uint16 idx = toBypass[op];
            volatile PrechargeSmInfo *sm = &g_precharge.smList[idx];
            if (sm->isUpperArm)
                GPIO_SetUpperSM(sm->smIndex, SM_BYPASSED);
            else
                GPIO_SetLowerSM(sm->smIndex, SM_BYPASSED);
        }
        
        // 再接入一个（增加接入数量）
        if (op < insertCount)
        {
            Uint16 idx = toInsert[op];
            volatile PrechargeSmInfo *sm = &g_precharge.smList[idx];
            if (sm->isUpperArm)
                GPIO_SetUpperSM(sm->smIndex, SM_INSERTED);
            else
                GPIO_SetLowerSM(sm->smIndex, SM_INSERTED);
        }
    }
}

// ==================== 设置最终状态（优化：K=0处理） ====================
static void Precharge_SetFinalState(void)
{
    // 最终状态：
    // - 上桥臂：电压最低的K个接入（GPIO高），其余旁路（GPIO低）
    // - 下桥臂：全部6个接入（GPIO高）

    // 下桥臂全部接入
    GPIO_SetAllLowerArm(SM_INSERTED);
    
    if (g_precharge.K_value == 0)
    {
        // K=0：上桥臂全部旁路
        GPIO_SetAllUpperArm(SM_BYPASSED);
        printf("[Precharge] Final: Lower=all, Upper=none (K=0)\r\n");
    }
    else
    {
        // K>0：需要排序并选择K个最低电压的接入
        Precharge_SortUpperArm();
        
        for (Uint16 i = 0; i < UPPER_ARM_COUNT; ++i)
        {
            Uint16 upperIdx = g_precharge.upperSortedIdx[i];
            if (i < g_precharge.K_value)
                GPIO_SetUpperSM(upperIdx, SM_INSERTED);
            else
                GPIO_SetUpperSM(upperIdx, SM_BYPASSED);
        }
        
        printf("[Precharge] Final: Lower=all, Upper=%d lowest inserted\r\n", 
               g_precharge.K_value);
    }
}

// ==================== 启动预充电 ====================
void Precharge_Start(void)
{
    g_precharge.state = PRECHARGE_WAIT_VOLTAGE;
    g_precharge.iterationCount = 0;
    g_precharge.phase = BALANCE_PHASE_CHARGE;
    g_precharge.currentBalanceLevel = TOTAL_SM_COUNT;
    
    // 初始：全部接入
    GPIO_SetAllUpperArm(SM_INSERTED);
    GPIO_SetAllLowerArm(SM_INSERTED);
    
    printf("[Precharge] Started - all 12 SM inserted\r\n");
}

// ==================== 预充电状态机（优化：处理level=12的情况） ====================
PrechargeState Precharge_Process(void)
{
    switch (g_precharge.state)
    {
        case PRECHARGE_IDLE:
            break;

        case PRECHARGE_WAIT_VOLTAGE:
            g_precharge.iterationCount++;
            Precharge_SampleAllVoltages();
            
            if (g_precharge.iterationCount >= 10)
            {
                g_precharge.iterationCount = 0;
                g_precharge.state = PRECHARGE_CALC_N;
            }
            DELAY_US(PRECHARGE_STABLE_DELAY_US);
            break;

        case PRECHARGE_CALC_N:
            Precharge_CalculateN();
            g_precharge.state = PRECHARGE_INITIAL_CHARGE;
            break;

        case PRECHARGE_INITIAL_CHARGE:
            g_precharge.iterationCount++;
            Precharge_SampleAllVoltages();
            
            if (g_precharge.iterationCount >= 20)
            {
                g_precharge.iterationCount = 0;
                // 从11开始（12个时无法均压，因为没有旁路的子模块）
                g_precharge.currentBalanceLevel = TOTAL_SM_COUNT - 1;
                g_precharge.state = PRECHARGE_BALANCING;
                printf("[Precharge] Initial charge done, balancing (level=%d, target=%d)\r\n",
                       g_precharge.currentBalanceLevel, g_precharge.targetN);
            }
            DELAY_US(PRECHARGE_STABLE_DELAY_US);
            break;

        case PRECHARGE_BALANCING:
            g_precharge.iterationCount++;
            
            if (g_precharge.iterationCount >= PRECHARGE_MAX_ITERATIONS)
            {
                printf("[Precharge] Timeout!\r\n");
                g_precharge.state = PRECHARGE_TIMEOUT;
                break;
            }
            
            Precharge_SampleAllVoltages();
            Precharge_SortAllByVoltage();
            
            #ifdef PRECHARGE_DEBUG
            if ((g_precharge.iterationCount % 100) == 0)
            {
                printf("[Balance] iter=%d, level=%d, spread=%d\r\n",
                       g_precharge.iterationCount,
                       g_precharge.currentBalanceLevel,
                       g_precharge.voltageSpread);
            }
            #endif
            
            // 检查均压是否完成
            if (Precharge_CheckBalanceComplete())
            {
                if (g_precharge.currentBalanceLevel > g_precharge.targetN)
                {
                    // 降低级别，继续均压
                    g_precharge.currentBalanceLevel--;
                    g_precharge.phase = BALANCE_PHASE_CHARGE;
                    printf("[Precharge] Level -> %d\r\n", g_precharge.currentBalanceLevel);
                }
                else
                {
                    // 达到目标N，进入最终稳定
                    g_precharge.state = PRECHARGE_FINAL_SETTLE;
                    printf("[Precharge] Reached targetN=%d, final settle\r\n", g_precharge.targetN);
                    break;
                }
            }
            
            Precharge_ApplyBalanceStrategy();
            
            g_precharge.phase = (g_precharge.phase == BALANCE_PHASE_CHARGE) ?
                                 BALANCE_PHASE_DISCHARGE : BALANCE_PHASE_CHARGE;
            
            DELAY_US(PRECHARGE_STABLE_DELAY_US);
            break;

        case PRECHARGE_FINAL_SETTLE:
            Precharge_SetFinalState();
            DELAY_US(PRECHARGE_STABLE_DELAY_US * 10);
            
            Precharge_SampleAllVoltages();
            Precharge_SortAllByVoltage();
            
            printf("[Precharge] Final check: spread=%d (tolerance=%d)\r\n",
                   g_precharge.voltageSpread, PRECHARGE_VOLTAGE_TOLERANCE * 2);
            
            if (g_precharge.voltageSpread <= PRECHARGE_VOLTAGE_TOLERANCE * 2)
            {
                g_precharge.state = PRECHARGE_COMPLETE;
                // printf("[Precharge] Complete!\r\n");
            }
            else
            {
                g_precharge.state = PRECHARGE_BALANCING;
                g_precharge.currentBalanceLevel = TOTAL_SM_COUNT - 1;
                printf("[Precharge] Retry balance\r\n");
            }
            break;

        case PRECHARGE_COMPLETE:
            break;

        case PRECHARGE_TIMEOUT:
        case PRECHARGE_ERROR:
            // TODO:这里好像什么都不用做
            // GPIO_SetAllUpperArm(SM_INSERTED);
            // GPIO_SetAllLowerArm(SM_INSERTED);
            break;

        default:
            g_precharge.state = PRECHARGE_ERROR;
            break;
    }

    return g_precharge.state;
}

// ==================== 检查完成 ====================
Uint16 Precharge_IsComplete(void)
{
    return (g_precharge.state == PRECHARGE_COMPLETE) ? 1 : 0;
}

// ==================== 准备MMC数组数据 ====================
void Precharge_PrepareMMCArrays(void)
{
    Precharge_SampleAllVoltages();
    Precharge_SortUpperArm();
    
    // 更新P_MMC_array（上桥臂）
    for (Uint16 i = 0; i < UPPER_ARM_COUNT; ++i)
    {
        P_MMC_array[i].recent_voltage = g_precharge.upperVoltages[i];
        P_MMC_array[i].voltage_diff = 0;
        
        // 找位置
        Uint16 sortedPos = 0;
        for (Uint16 j = 0; j < UPPER_ARM_COUNT; ++j)
        {
            if (g_precharge.upperSortedIdx[j] == i)
            {
                sortedPos = j;
                break;
            }
        }
        P_MMC_array[i].position_id = sortedPos;
        P_MMC_array[i].last_position_id = sortedPos;
    }
    
    // 更新N_MMC_array（下桥臂）
    // 先排序下桥臂
    Uint16 lowerSortedIdx[LOWER_ARM_COUNT] = {0, 1, 2, 3, 4, 5};
    for (Uint16 i = 1; i < LOWER_ARM_COUNT; ++i)
    {
        Uint16 keyIdx = lowerSortedIdx[i];
        int16 keyV = g_precharge.lowerVoltages[keyIdx];
        int16 j = (int16)i - 1;
        
        while (j >= 0 && g_precharge.lowerVoltages[lowerSortedIdx[j]] > keyV)
        {
            lowerSortedIdx[j + 1] = lowerSortedIdx[j];
            j--;
        }
        lowerSortedIdx[j + 1] = keyIdx;
    }
    
    for (Uint16 i = 0; i < LOWER_ARM_COUNT; ++i)
    {
        N_MMC_array[i].recent_voltage = g_precharge.lowerVoltages[i];
        N_MMC_array[i].voltage_diff = 0;
        
        Uint16 sortedPos = 0;
        for (Uint16 j = 0; j < LOWER_ARM_COUNT; ++j)
        {
            if (lowerSortedIdx[j] == i)
            {
                sortedPos = j;
                break;
            }
        }
        N_MMC_array[i].position_id = sortedPos;
        N_MMC_array[i].last_position_id = sortedPos;
    }
    
    // 更新全局K值
    // TODO：待确认正确性
    K_value = g_precharge.K_value;
    K_last_value_P = K_value;
    K_last_value_N = K_value;
}

// ==================== 禁用所有中断 ====================
void EPWM_DisableAllInterrupts(void)
{
    EALLOW;
    for (Uint16 i = 0; i < 6; ++i)
    {
        ePWM_regs[i]->ETSEL.bit.INTEN = 0;
        ePWM_regs[i]->ETCLR.bit.INT = 1;
    }
    
    PieCtrlRegs.PIEIER3.bit.INTx1 = 0;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 0;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 0;
    PieCtrlRegs.PIEIER3.bit.INTx4 = 0;
    PieCtrlRegs.PIEIER3.bit.INTx5 = 0;
    PieCtrlRegs.PIEIER3.bit.INTx6 = 0;
    EDIS;
}

// ==================== 使能闭环中断 ====================
void EPWM_EnableClosedLoopInterrupts(void)
{
    EALLOW;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
    IER |= M_INT3;
    EDIS;
    
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
