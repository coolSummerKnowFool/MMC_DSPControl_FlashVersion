#include "epwm_combine.h"
#include "printf.h"
#include "leds.h"
#include "ad760x.h"
#include "basic_control.h"
#include "dma_xintf.h"
#include "epwm_softstart.h"
// #include <string.h> // for memcpy if needed

/* ========== 必须写入 RAM 的中断服务函数 ========== */
#pragma CODE_SECTION(EPWM_GlobalISR,                    ".TI.ramfunc")
// #pragma CODE_SECTION(EPWM_RestoreISR,                   ".TI.ramfunc")

/* ========== 由 DMA/主中断调用的核心排序与更新函数 ========== */
#pragma CODE_SECTION(BridgeArmSort,                     ".TI.ramfunc")

/* ========== Partial 注入/恢复关键路径 ========== */
#pragma CODE_SECTION(Partial_TryInjectNow,              ".TI.ramfunc")
#pragma CODE_SECTION(Partial_RestoreActive,             ".TI.ramfunc")
#pragma CODE_SECTION(Partial_ForceAbort,                ".TI.ramfunc")
#pragma CODE_SECTION(Partial_BuildPrepared,             ".TI.ramfunc")
#pragma CODE_SECTION(Partial_IsSafeToInject,            ".TI.ramfunc")
#pragma CODE_SECTION(Partial_WriteAllCmp,               ".TI.ramfunc")
#pragma CODE_SECTION(Partial_WriteCmpA,                 ".TI.ramfunc")
#pragma CODE_SECTION(Partial_WriteCmpB,                 ".TI.ramfunc")
// #pragma CODE_SECTION(Partial_WriteAQ,                   ".TI.ramfunc")
// #pragma CODE_SECTION(Partial_GetMinCmpNext,             ".TI.ramfunc")
#pragma CODE_SECTION(Partial_ClearPrepared,             ".TI.ramfunc")
#pragma CODE_SECTION(Partial_ClearActive,               ".TI.ramfunc")

/* ========== K 模块 PWM 动作切换 ========== */
#pragma CODE_SECTION(Change_K_Model_PWMWorkingStatus,   ".TI.ramfunc")

/* ========== 频率/误差/占空比更新及 PID 计算 ========== */
#pragma CODE_SECTION(EPWM_FrequencyUpdate,              ".TI.ramfunc")
#pragma CODE_SECTION(EPWM_ErrorUpdate,                  ".TI.ramfunc")
#pragma CODE_SECTION(Frequence_PID_Incremental_Calculate, ".TI.ramfunc")
#pragma CODE_SECTION(Error_PID_Incremental_Calculate,   ".TI.ramfunc")
#pragma CODE_SECTION(Duty_PID_Incremental_Calculate,    ".TI.ramfunc")
#pragma CODE_SECTION(Get_K_D_Method,                    ".TI.ramfunc")
#pragma CODE_SECTION(DecodeKDValue,                     ".TI.ramfunc")

/* ========== 实时性要求较高的辅助计算 ========== */
#pragma CODE_SECTION(Calc_TBPRD_forFreq,                ".TI.ramfunc")
#pragma CODE_SECTION(get_average_recent_voltage,        ".TI.ramfunc")
#pragma CODE_SECTION(change_CMP_Values_BasedOnError,    ".TI.ramfunc")
#pragma CODE_SECTION(Partial_CalcTimeBase, ".TI.ramfunc")


// ---------------- 全局变量定义 ----------------
// 索引含义： 0=CMPA0, 1=CMPB0, 2=CMPA1, 3=CMPB1, 4=CMPA2, 5=CMPB2
Uint16 CMP_Values[2 * PWM_PAIR_COUNT] = {450, 510, 570, 630, 690, 750};          // 存放所有比较值，便于排序
Uint16 CMP_addError_Values[2 * PWM_PAIR_COUNT] = {450, 510, 570, 630, 690, 750}; // 存放所有比较值，便于排序

PID fre_PID = {fre_PID_P, fre_PID_I, fre_PID_D, 0.0, 0.0, INIT_FREQUENCE};
PID err_PID = {err_PID_P, err_PID_I, err_PID_D, 0.0, 0.0, INIT_ERROR};
PID duty_PID = {duty_PID_P, duty_PID_I, duty_PID_D, 0.0, 0.0, INIT_DUTY_CYCLE};
float32 arm_per_voltage_P = 0.0;
float32 arm_per_voltage_N = 0.0;

Uint16 updat_voltage_index_record[MMC_NUMBER] = {0};
Uint16 *updat_voltage_index_record_ptr = updat_voltage_index_record; // 指向上面数组的指针，便于传参
int32 predict_voltage_diff = 0;                                      // 存放预测的电压差值（单位是AD7606的码值）
#define CMPA_IDX(i) (2 * (i))                                                 // 给 ePWM i 的 A 通道的 slot 索引（0/2/4）
#define CMPB_IDX(i) (2 * (i) + 1)                                             // 给 ePWM i 的 B 通道的 slot 索引（1/3/5）

static Uint32 g_isrToggleCnt = 0; // 中断切换计数器，计数为奇数时为下桥臂更新，计数为偶数为上桥臂更新
static Uint16 g_firstIsrDone = 0; // 标记是否已执行第一次中断（用于禁用ZRO动作）
Uint16 g_ADCIsrFlag = 0; // 标记是否进入ADC的中断内的桥臂控制函数(预充电期间不执行，闭环期间执行)

// volatile Uint16 SortOverFlag = 0;

/* ========== Partial 模块全局变量定义 ========== */
volatile float32 g_partialDuty = 0.0f;
PartialPreparedCtx  g_partialPrepared = {0};
volatile PartialActiveCtx    g_partialActive   = {0};

/* 调试统计：记录因安全检查失败而跳过注入的次数 */
volatile Uint32 g_partialInjectSkipCnt = 0;
volatile Uint32 g_partialRestoreMissCnt = 0;

/* 调试观测：记录最近一次使用的时间基准与临界 D */
volatile Uint16  g_partialTimeBase = 0;
volatile float32 g_partialDcrit    = 0.0f;

Uint16 changeState_flag_position = 6;
ChangeKStatus changeState_flag_State = Set_New_Channel_N;

// ----- 便利 ePWM 指针数组（便于循环访问） -----
// ==== 修改/优化 ==== 把常用 ePWM 指针放在文件作用域，避免反复 switch 调用
static volatile struct EPWM_REGS *const ePWM_all[6] = {&EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs};

// 计算 TBPRD（上计数模式），公式 TBPRD = TBCLK / freq
static Uint16 Calc_TBPRD_forFreq(Uint32 freqHz)
{
    if (freqHz < PWM_FREQ_MIN_HZ)
        freqHz = PWM_FREQ_MIN_HZ;
    if (freqHz > PWM_FREQ_MAX_HZ)
        freqHz = PWM_FREQ_MAX_HZ;

    Uint64 tbclk = (Uint64)SYSCLK_HZ;
    Uint64 tbprd = tbclk / freqHz;
    if (tbprd < 1)
        tbprd = 1;
    if (tbprd > 0xFFFF)
        tbprd = 0xFFFF;
    return (Uint16)tbprd;
}

/* ==================== 清空函数 ==================== */
static void Partial_ClearPrepared(PartialPreparedCtx *ctx)
{
    ctx->valid        = 0;
    ctx->arm          = 0;
    ctx->smIndex      = 0;
    ctx->localModule  = 0;
    ctx->globalModule = 0;
    ctx->channelIsA   = 1;
    ctx->ownBaseCmp   = 0;
    ctx->firstCmp     = 0;
    ctx->secondCmp    = 0;
}

static void Partial_ClearActive(volatile PartialActiveCtx *ctx)
{
    ctx->active                 = 0;
    ctx->arm                    = PARTIAL_ARM_P;
    ctx->smIndex                = 0;
    ctx->localModule            = 0;
    ctx->globalModule           = 0;
    ctx->channelIsA             = 1;
    ctx->secondCmpVal           = 0;
    ctx->savedBorrowCmp         = 0;
    ctx->savedPairedAQ          = 0;
    ctx->deferredBorrowCmp      = 0;
    ctx->deferredBorrowCmpValid = 0;
    ctx->deferredPairedAQ       = 0;
    ctx->deferredPairedAQValid  = 0;
}

// ---------------- 统一的闭环ePWM初始化函数 ----------------
void EPWM_SwitchToClosedLoop(Uint16 fromPrecharge)
{
    Uint16 i;
    Uint16 tbprd;

    EALLOW;
    // 先禁止 TBCLK 同步以便批量配置
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    // 冷启动时需要初始化GPIO为ePWM功能
    if (!fromPrecharge)
    {
        InitEPwm1Gpio();
        InitEPwm2Gpio();
        InitEPwm3Gpio();
        InitEPwm4Gpio();
        InitEPwm5Gpio();
        InitEPwm6Gpio();
    }

    EALLOW;
    // 计算 TBPRD 并写入所有模块
    tbprd = Calc_TBPRD_forFreq(INIT_FREQUENCE);

    for (i = 0; i < 6; ++i)
    {
        volatile struct EPWM_REGS *E = ePWM_all[i];

        // 清除所有配置
        E->TBCTL.all = 0;
        E->AQCTLA.all = 0;
        E->AQCTLB.all = 0;
        E->AQSFRC.all = 0;
        E->AQCSFRC.all = 0;

        // TBCTL
        // E->TBCTL.all = 0;
        E->TBCTL.bit.PHSEN = (i == 0) ? TB_DISABLE : TB_ENABLE; // ePWM1为主
        E->TBCTL.bit.PRDLD = TB_SHADOW;
        E->TBCTL.bit.HSPCLKDIV = TB_DIV1; // HSPCLKDIV = 1
        E->TBCTL.bit.CLKDIV = TB_DIV1;    // CLKDIV = 1
        E->TBPRD = tbprd;
        E->TBPHS.half.TBPHS = 0;
        E->TBCTR = 0;
        E->TBCTL.bit.CTRMODE = TB_COUNT_UP;

        // 同步选择： ePWM1 发 SYNCOUT，其他为 SYNC_IN
        E->TBCTL.bit.SYNCOSEL = (i == 0) ? TB_CTR_ZERO : TB_SYNC_IN;

        // 关闭影子寄存器，修改CMP的值直接生效（也可以选择在 CTR==0 时载入，见后面设置）
        // E->CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
        // E->CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;

        // CMP 影子寄存器设置（在 CTR==0 时载入）
        E->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        E->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
        E->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
        E->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

        // 写入初始 CMP 影子值（通过 CMP_order 映射）
        Uint16 idxA = (i < 3) ? P_MMC_array[CMPA_IDX(i)].position_id : N_MMC_array[CMPA_IDX(i - 3)].position_id;
        Uint16 idxB = (i < 3) ? P_MMC_array[CMPB_IDX(i)].position_id : N_MMC_array[CMPB_IDX(i - 3)].position_id;
        E->CMPA.half.CMPA = CMP_Values[idxA];
        E->CMPB = CMP_Values[idxB];

        // 使用AQCSFRC设置初始电平
        if (i < 3)
        {
            // 上桥臂
            if (P_MMC_array[CMPA_IDX(i)].position_id < K_value)
            {
                // K模块：强制高，保持高
                E->AQCSFRC.bit.CSFA = 2;
                E->AQCTLA.bit.CAU = AQ_SET;
            }
            else
            {
                // 非K模块：强制低，正常翻转
                E->AQCSFRC.bit.CSFA = 1;
                E->AQCTLA.bit.CAU = AQ_CLEAR;
            }

            if (P_MMC_array[CMPB_IDX(i)].position_id < K_value)
            {
                E->AQCSFRC.bit.CSFB = 2;
                E->AQCTLB.bit.CBU = AQ_SET;
            }
            else
            {
                E->AQCSFRC.bit.CSFB = 1;
                E->AQCTLB.bit.CBU = AQ_CLEAR;
            }
        }
        else
        {
            // 下桥臂
            Uint16 lowerIdx = i - 3;

            if (N_MMC_array[CMPA_IDX(lowerIdx)].position_id < K_value)
            {
                // K模块：强制高，保持高
                E->AQCSFRC.bit.CSFA = 2;
                E->AQCTLA.bit.CAU = AQ_SET;
            }
            else
            {
                // 非K模块：强制高，正常翻转
                E->AQCSFRC.bit.CSFA = 2;
                E->AQCTLA.bit.CAU = AQ_SET;
            }

            if (N_MMC_array[CMPB_IDX(lowerIdx)].position_id < K_value)
            {
                E->AQCSFRC.bit.CSFB = 2;
                E->AQCTLB.bit.CBU = AQ_SET;
            }
            else
            {
                E->AQCSFRC.bit.CSFB = 2;
                E->AQCTLB.bit.CBU = AQ_SET;
            }
        }

        // ==== 修改/优化 ==== 设为每次事件都触发中断（ET_1ST），注释已修正
        E->ETPS.bit.INTPRD = ET_1ST; // 每次匹配事件都会触发中断（ET_1ST）
        // 清除中断标志并禁止中断（后面会按最大值选举启用）
        E->ETCLR.bit.INT = 1;
        E->ETSEL.all = 0;
    }

    /* ---------- PIE 向量映射（新架构：只有两个 ISR） ---------- */
    PieVectTable.EPWM1_INT = &EPWM_GlobalISR;    /* 主中断，固定 ePWM1 CMPA */
    // PieVectTable.EPWM4_INT = &EPWM_RestoreISR;   /* Partial 恢复中断，按需开启 */

    // 如果从预充电切换，先切换MUX
    if (fromPrecharge)
    {
        GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;
        GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;
    }

    // 只使能 ePWM1 的 CMPA 中断
    EPwm1Regs.ETCLR.bit.INT = 1;                    // 清中断标志
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // 触发源：CTR向上匹配CMPA
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;             // 每次事件都触发
    EPwm1Regs.ETSEL.bit.INTEN = 1;                  // 使能ePWM1中断

    /* ---------- ePWM4 restore 中断：初始关闭 ---------- */
    // EPwm4Regs.ETCLR.bit.INT    = 1;
    // EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;     /* 固定为 PRD 触发 */
    // EPwm4Regs.ETPS.bit.INTPRD  = ET_1ST;
    // EPwm4Regs.ETSEL.bit.INTEN  = 1;              /* ★ 永久使能 */
    /* ★ 禁用 ePWM4 中断（不再使用） ★ */
    EPwm4Regs.ETCLR.bit.INT    = 1;
    EPwm4Regs.ETSEL.bit.INTEN  = 0;

    /* ---------- PIE 使能位 ---------- */
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;   /* ePWM1 主中断，常开 */
    PieCtrlRegs.PIEIER3.bit.INTx2 = 0;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 0;
    PieCtrlRegs.PIEIER3.bit.INTx4 = 0;   /* ePWM4 restore，动态开启 */
    PieCtrlRegs.PIEIER3.bit.INTx5 = 0;
    PieCtrlRegs.PIEIER3.bit.INTx6 = 0;

    // 重置ISR标志和Partial 状态
    EPWM_ResetISRFlags();  
    
    // 恢复 TBCLK 同步，这样所有 ePWM 一起开始计数
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    EDIS;

    IER |= M_INT3;

    printf("[ePWM] ClosedLoop Init (fromPre=%d, K=%d, TBPRD=%d)\r\n", fromPrecharge, K_value, tbprd);
}

// PID增量计算函数
void Frequence_PID_Incremental_Calculate(PID *pid, float setpoint, float feedback)
{
    // 当前误差
    float error = feedback - setpoint;

    // 增量计算公式： Δu = kp * (e[n] - e[n-1]) + ki * e[n] + kd * (e[n] - 2*e[n-1] + e[n-2])
    float delta_output = pid->kp * (error - pid->prev_error) + pid->ki * error + pid->kd * (error - 2 * pid->prev_error + pid->prev2_error);

    // 更新输出值
    pid->output += delta_output;
    // 限幅
    if (pid->output > FREQ_MAX)
    {
        pid->output = FREQ_MAX;
    }
    if (pid->output < FREQ_MIN)
    {
        pid->output = FREQ_MIN;
    }

    // 更新历史误差
    pid->prev2_error = pid->prev_error;
    pid->prev_error = error;
}

void Error_PID_Incremental_Calculate(PID *pid, float setpoint, float feedback)
{
    // 当前误差
    float error = feedback - setpoint;

    // 增量计算公式： Δu = kp * (e[n] - e[n-1]) + ki * e[n] + kd * (e[n] - 2*e[n-1] + e[n-2])
    float delta_output = pid->kp * (error - pid->prev_error) + pid->ki * error + pid->kd * (error - 2 * pid->prev_error + pid->prev2_error);

    // 更新输出值
    pid->output += delta_output;
    // 限幅
    if (pid->output > ERROR_MAX)
    {
        pid->output = ERROR_MAX;
    }
    if (pid->output < ERROR_MIN)
    {
        pid->output = ERROR_MIN;
    }

    // 更新历史误差
    pid->prev2_error = pid->prev_error;
    pid->prev_error = error;
}

void Duty_PID_Incremental_Calculate(PID *pid, float setpoint, float feedback)
{
    // 当前误差
    float delta_output;
    float error = feedback - setpoint;

    if(fabsf(error) > 80.0f)
    {
        delta_output = pid->kp * (error - pid->prev_error) + pid->ki * error + pid->kd * (error - 2 * pid->prev_error + pid->prev2_error);
    }
    else
    {
        delta_output = pid->kp * (error - pid->prev_error) + pid->kd * (error - 2 * pid->prev_error + pid->prev2_error);
    }

    // 更新输出值
    pid->output += delta_output;
    // 限幅
    if (pid->output > DUTY_MAX)
    {
        pid->output = DUTY_MAX;
    }
    if (pid->output < DUTY_MIN)
    {
        pid->output = DUTY_MIN;
    }

    // 更新历史误差
    pid->prev2_error = pid->prev_error;
    pid->prev_error = error;
}

// 主动触发一次写入硬件（将 CMP... 写进影子寄存器 / TBPRD）
void EPWM_FrequencyUpdate(void)
{
    // U_load = AD7606_2_Buf[6] * 5 / 32768.0 * 100.8; // 100.8 = 2016kΩ / 20kΩ
    // TODO
    // U_load = AD7606_2_Buf[6] / 32768.0f * 504.0f;
    float32 U_load_test = AD7606_2_Buf[6] / 32768.0f * 504.0f;

    // SOFT_PWM3_TOGGLE;
    Frequence_PID_Incremental_Calculate(&fre_PID, TARGET_VOLTAGE, U_load);
    // SOFT_PWM3_TOGGLE;

    // Uint16 tbprd = Calc_TBPRD_forFreq((Uint32)24000);
    Uint16 tbprd = Calc_TBPRD_forFreq((Uint32)fre_PID.output);

    for (Uint16 i = 0; i < 6; ++i)
    {
        ePWM_all[i]->TBPRD = tbprd;
    }
}

float get_average_recent_voltage(mmc_info *mmc_array)
{
    int16 min = mmc_array->recent_voltage;
    int16 max = mmc_array->recent_voltage;

    for (int i = 1; i < MMC_NUMBER; ++i)
    {
        int16 val = mmc_array[i].recent_voltage;
        if (val < min)
            min = val;
        if (val > max)
            max = val;
    }

    return (min + max) / 2.0f;
}

void change_CMP_Values_BasedOnError(Uint16 *CMP_addError_Values)
{
    // TODO
    // int32 error = (int32)err_PID.output;
    int32 error = 0;
    Uint16 i;
    /* 误差为零，直接复制（编译器会优化为 RPT || MOV） */
    if (error == 0)
    {
        for (i = 0; i < MMC_NUMBER; i++)
            CMP_addError_Values[i] = CMP_Values[i];
    }
    else
    {
        for (i = 0; i < MMC_NUMBER; i++)
            CMP_addError_Values[i] = CMP_Values[i] + error;
    }
}

void EPWM_ErrorUpdate(ArmUpdateStatus status)
{
    switch (status)
    {
    case Update_P_K_Arm:
        arm_per_voltage_P = get_average_recent_voltage(P_MMC_array);
        break;

    case Update_N_K_Arm:
        arm_per_voltage_N = get_average_recent_voltage(N_MMC_array);
        Error_PID_Incremental_Calculate(&err_PID, TARGET_ERROR, arm_per_voltage_P - arm_per_voltage_N);
        change_CMP_Values_BasedOnError(CMP_addError_Values);
        break;

    default:
        break;
    }
}

void Change_K_Model_PWMWorkingStatus(Uint16 position, ChangeKStatus order)
{
    if (position < MMC_NUMBER)
    {
        if (position & 0x01) // 奇数个(1、3、5)
        {
            switch (order)
            {
            case Set_New_Channel_N:
                ePWM_all[(position >> 1) + 3]->AQCTLB.bit.CBU = AQ_SET;
                break;
            case Reset_Old_Channel_N:
                ePWM_all[(position >> 1) + 3]->AQCTLB.bit.CBU = AQ_TOGGLE;
                break;
            case Set_New_Channel_P:
                ePWM_all[position >> 1]->AQCTLB.bit.CBU = AQ_SET;
                break;
            case Reset_Old_Channel_P:
                ePWM_all[position >> 1]->AQCTLB.bit.CBU = AQ_TOGGLE;
                break;
            default:
                break;
            }
        }
        else // 偶数个(0、2、4)
        {
            switch (order)
            {
            case Set_New_Channel_N:
                ePWM_all[(position >> 1) + 3]->AQCTLA.bit.CAU = AQ_SET;
                break;
            case Reset_Old_Channel_N:
                ePWM_all[(position >> 1) + 3]->AQCTLA.bit.CAU = AQ_TOGGLE;
                break;
            case Set_New_Channel_P:
                ePWM_all[position >> 1]->AQCTLA.bit.CAU = AQ_SET;
                break;
            case Reset_Old_Channel_P:
                ePWM_all[position >> 1]->AQCTLA.bit.CAU = AQ_TOGGLE;
                break;
            default:
                break;
            }
        }
    }
}

/*
 * Partial_CalcTimeBase()
 *
 * 计算 Partial 两次翻转所使用的时间基准 T_base。
 *
 * ── 设计原理 ──
 *
 * 理论上 Partial 的时间基准应当是一个完整子周期 tbprd：
 *     firstCmp  = ownBase + tbprd * D
 *     secondCmp = ownBase + tbprd * (1 - D)
 *
 * 但当 D 较小时，secondCmp 会超出本子周期末尾，
 * 落到下一子周期中（当前架构不支持跨周期）。
 *
 * 越界条件：
 *     ownBase + tbprd*(1-D) > tbprd - guard
 *  ⇔ D < (ownBase + guard) / tbprd  =  D_crit
 *
 * ── 解决方案 ──
 *
 * 在"secondCmp 不越界"的约束下，取 T_base 的最大可行值：
 *
 *     ownBase + T_base*(1-D) ≤ tbprd - guard
 *  ⇒  T_base ≤ (tbprd - guard - ownBase) / (1 - D)
 *
 * 再与物理上限 tbprd 取较小者：
 *
 *     T_base = min( tbprd,  (tbprd - guard - ownBase) / (1 - D) )
 *
 * ── 行为特性 ──
 *
 *   D ≥ D_crit  →  T_base = tbprd            （理论正确，无损）
 *   D → 0       →  T_base → tbprd - ownBase  （退化为原实现）
 *   中间区域    →  连续单调过渡，无需调参
 *
 * 参数：
 *   tbprd   : 当前子周期长度
 *   ownBase : Partial 模块原始 compare 值
 *   dutyD   : Partial 占空比 D (0 ~ 0.5)
 *
 * 返回：
 *   T_base（计数值），0 表示参数异常无法构造 Partial
 */
static Uint16 Partial_CalcTimeBase(Uint16 tbprd, Uint16 ownBase, float32 dutyD)
{
    float32 headroom;    /* tbprd - guard - ownBase，即 secondCmp 可用空间 */
    float32 tBaseMax;    /* 约束允许的最大基准 */
    Uint16  tBase;

    /* ---- 计算可用空间 ---- */
    headroom = (float32)tbprd - (float32)PARTIAL_SECOND_GUARD_CNT - (float32)ownBase;

    /* 异常：ownBase 过大（接近或超过 tbprd），无法安排 Partial */
    if (headroom <= 0.0f)
    {
        g_partialTimeBase = 0;
        return 0;
    }

    /* ---- 约束允许的最大基准 ----
     * 分母 (1-D) 恒 > 0.5（因为 D ≤ PARTIAL_D_MAX = 0.497），
     * 不会出现除零 */
    tBaseMax = headroom / (1.0f - dutyD);

    /* ---- 与物理上限取较小者 ---- */
    if (tBaseMax >= (float32)tbprd)
        tBase = tbprd;                    /* D 较大：使用完整子周期，理论正确 */
    else
        tBase = (Uint16)tBaseMax;         /* D 较小：压缩基准，保证不越界 */

    /* ---- 调试记录 ---- */
    g_partialTimeBase = tBase;
    g_partialDcrit    = ((float32)ownBase + (float32)PARTIAL_SECOND_GUARD_CNT) / (float32)tbprd;

    return tBase;
}

/* ==================== Build Prepared ==================== */
/*
 * Partial_BuildPrepared()
 *
 * 在 BridgeArmSort() 后段调用，为【下一子周期】构造 Partial 计划。
 *
 * secondCmp 计算（不跨周期）：
 *   T_partial = TBPRD - ownBase
 *   firstCmp  = ownBase + T_partial * D
 *   secondCmp = ownBase + T_partial * (1 - D)
 *
 * secondCmp 的上限约束：
 *   必须 < TBPRD（因为 restore 在 PRD 时才触发，
 *   secondCmp 必须在 PRD 之前发生，否则第二次翻转丢失）
 *
 * 返回：1=构造成功，0=退化为普通半开通
 */
static Uint16 Partial_BuildPrepared(mmc_info *armArray,
                                    Uint16   *cmpArray,
                                    Uint16             moduleBase,
                                    float32            dutyD,
                                    Uint16             tbprd,
                                    PartialPreparedCtx *ctx)
{
    Uint16 smIndex = 0xFFFFU;
    Uint16 i;
    Uint16 ownBase, tPartial;
    Uint16 firstCmp, secondCmp;

    Partial_ClearPrepared(ctx);

    /* D 边界检查 */
    if (dutyD <= PARTIAL_D_EPS)
        return 0;
    if (dutyD > PARTIAL_D_MAX)
        dutyD = PARTIAL_D_MAX;

    if (K_value >= MMC_NUMBER)
        return 0;

    /* 找到 position_id == K_value 的物理子模块（Partial 候选） */
    for (i = 0; i < MMC_NUMBER; i++)
    {
        if (armArray[i].position_id == K_value)
        {
            smIndex = i;
            break;
        }
    }
    if (smIndex == 0xFFFFU)
        return 0;

    /* 双保险：确认不在受限通道（sm0/sm1） */
    // if ((smIndex == 0U) || (smIndex == 1U))
    //     return 0;

    ownBase = cmpArray[K_value];

    if (tbprd <= ownBase)
        return 0;

    /* ══════════════════════════════════════════════════════
     * 计算时间基准 T_base（自适应：D 大用 tbprd，D 小自动压缩）
     * ══════════════════════════════════════════════════════ */
    tPartial = Partial_CalcTimeBase(tbprd, ownBase, dutyD);
    if (tPartial == 0U)
        return 0;                         /* 参数异常 */

    /* ── 两次翻转时刻 ──
     *   firstCmp  : HIGH→LOW，比原 compare 延后 T_base*D
     *   secondCmp : LOW→HIGH，比周期末尾提前 T_base*D
     * 两者关于 (ownBase + T_base/2) 对称 */
    firstCmp  = ownBase + (Uint16)(dutyD * (float32)tPartial + 0.5f);
    secondCmp = ownBase + (Uint16)((1.0f - dutyD) * (float32)tPartial + 0.5f);

    /* ---- 保护1：两次翻转间隔过小 ----
     * 间隔 = T_base * (1 - 2D)
     * D → 0.5 时间隔趋于 0，硬件来不及完成两次 AQ 动作 */
    if (secondCmp <= (firstCmp + PARTIAL_MIN_GAP_CNT))
        return 0;

    /* ---- 保护2：secondCmp 越界兜底 ----
     * Partial_CalcTimeBase 已在数学上保证不越界，
     * 此处仅防浮点舍入误差导致的 ±1 偏差 */
    if (secondCmp > (tbprd - PARTIAL_SECOND_GUARD_CNT))
        secondCmp = tbprd - PARTIAL_SECOND_GUARD_CNT;

    /* ---- 保护3：钳位后再次检查间隔 ---- */
    if (secondCmp <= (firstCmp + PARTIAL_MIN_GAP_CNT))
        return 0;

    /* 填充上下文 */
    ctx->valid        = 1;
    ctx->arm          = (moduleBase == 0U) ? PARTIAL_ARM_P : PARTIAL_ARM_N;
    ctx->smIndex      = smIndex;
    ctx->localModule  = smIndex >> 1;
    ctx->globalModule = moduleBase + ctx->localModule;
    ctx->channelIsA   = ((smIndex & 0x01U) == 0U) ? 1U : 0U;
    ctx->ownBaseCmp   = ownBase;
    ctx->firstCmp     = firstCmp;
    ctx->secondCmp    = secondCmp;

    return 1;
}

/* ==================== 安全检查 ==================== */
/*
 * Partial_IsSafeToInject()
 *
 * 确认 TBCTR 已越过本子周期所有半开通 compare 点。
 * 取两张 CMP 表中最大值 + 裕量。
 */
static Uint16 Partial_IsSafeToInject(void)
{
    Uint16 ctr, maxCmp;

    maxCmp = CMP_Values[MMC_NUMBER - 1];
    if (CMP_addError_Values[MMC_NUMBER - 1] > maxCmp)
        maxCmp = CMP_addError_Values[MMC_NUMBER - 1];

    ctr = EPwm1Regs.TBCTR;

    if (ctr <= (maxCmp + PARTIAL_INJECT_GUARD_CNT))
        return 0;

    return 1;
}

/* ==================== 注入 Second Edge ==================== */
/*
 * Partial_TryInjectNow()
 *
 * 调用位置：BridgeArmSort() 开头
 *
 * 消费上一子周期构造的 Prepared，注入 secondCmp。
 *
 * 新方案只涉及一个借用操作（不碰 ePWM4）：
 *
 *   Partial 在 ePWM2A 为例：
 *   ① 保存 ePWM2 CMPB 原值 + AQCTLB.CBU
 *   ② AQCTLB.CBU = NO_ACTION（屏蔽 B 对 CMPB）
 *   ③ AQCTLA.CBU = TOGGLE（A 在 CMPB 处第二次翻转）
 *   ④ CMPB 切 Immediate → 写入 secondCmp
 *
 * restore 触发：ePWM4 的 PRD 事件（在本函数中使能）
 *
 * 返回：1=注入成功，0=未注入
 */
Uint16 Partial_TryInjectNow(void)
{
    volatile struct EPWM_REGS *E_partial;
    PartialPreparedCtx *prep = &g_partialPrepared;
    volatile PartialActiveCtx   *act  = &g_partialActive;

    if (!prep->valid)
        return 0;

    // if (act->active)
    // {
    //     g_partialRestoreMissCnt++;
    //     Partial_ClearPrepared(prep);
    //     return 0;
    // }

    if (!Partial_IsSafeToInject())
    {
        g_partialInjectSkipCnt++;
        Partial_ClearPrepared(prep);
        return 0;
    }

    E_partial = ePWM_all[prep->globalModule];

    Partial_ClearActive(act);
    act->active       = 1;
    act->arm          = prep->arm;
    act->smIndex      = prep->smIndex;
    act->localModule  = prep->localModule;
    act->globalModule = prep->globalModule;
    act->channelIsA   = prep->channelIsA;
    act->secondCmpVal = prep->secondCmp;

    EALLOW;

    if (prep->channelIsA)
    {
        /* Partial 在 A → 借 CMPB */
        act->savedBorrowCmp = E_partial->CMPB;
        act->savedPairedAQ  = E_partial->AQCTLB.bit.CBU;

        E_partial->AQCTLB.bit.CBU = AQ_NO_ACTION;
        E_partial->AQCTLA.bit.CBU = AQ_TOGGLE;

        E_partial->CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;
        E_partial->CMPB = prep->secondCmp;
    }
    else
    {
        /* Partial 在 B → 借 CMPA */
        act->savedBorrowCmp = E_partial->CMPA.half.CMPA;
        act->savedPairedAQ  = E_partial->AQCTLA.bit.CAU;

        E_partial->AQCTLA.bit.CAU = AQ_NO_ACTION;
        E_partial->AQCTLB.bit.CAU = AQ_TOGGLE;

        E_partial->CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
        E_partial->CMPA.half.CMPA = prep->secondCmp;
    }

    EDIS;

    Partial_ClearPrepared(prep);
    return 1;
}

/* ==================== 恢复 Active ==================== */
/*
 * Partial_RestoreActive()
 *
 * 调用位置：EPWM_RestoreISR()（ePWM4 PRD 中断）
 *
 * 此刻 CTR = TBPRD，secondCmp 的翻转已完成。
 * 距下一子周期 CTR=0 还有 1 个 TBCLK（6.67ns），
 * 距最早的 compare（~450）还有 ~3μs。
 *
 * 只需恢复 Partial 所在模块的 borrowed compare：
 * ① 取消 Partial 通道对 borrowed compare 的翻转
 * ② 恢复 paired 通道的 AQ 动作
 * ③ 三步写入恢复 borrowed compare 值
 * ④ 关闭 restore 中断
 *
 * 不碰 ePWM4 的 CMPA / AQ 等寄存器。
 */
static void Partial_RestoreActive(volatile PartialActiveCtx *act)
{
    volatile struct EPWM_REGS *E_partial;
    Uint16 borrowCmp, pairedAQ;

    if (!act->active)
        return;

    E_partial = ePWM_all[act->globalModule];

    borrowCmp = act->deferredBorrowCmpValid
                    ? act->deferredBorrowCmp
                    : act->savedBorrowCmp;

    pairedAQ = act->deferredPairedAQValid
                   ? act->deferredPairedAQ
                   : act->savedPairedAQ;

    EALLOW;

    if (act->channelIsA)
    {
        /* ★ 新增：Partial A 通道下一子周期保持高
         * 相当于把该模块变成"临时 K 模块"
         * 下下子周期由 BridgeArmSort 的 K 切换逻辑恢复为 TOGGLE */
        E_partial->AQCTLA.bit.CAU = AQ_SET;

        /* Partial 在 A，借的是 CMPB */
        E_partial->AQCTLA.bit.CBU = AQ_NO_ACTION;
        E_partial->AQCTLB.bit.CBU = pairedAQ;

        /* 三步写入 */
        E_partial->CMPB = borrowCmp;
        E_partial->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
        E_partial->CMPB = borrowCmp;

    }
    else
    {
        /* ★ 新增：Partial B 通道下一子周期保持高 */
        E_partial->AQCTLB.bit.CBU = AQ_SET;

        /* Partial 在 B，借的是 CMPA */
        E_partial->AQCTLB.bit.CAU = AQ_NO_ACTION;
        E_partial->AQCTLA.bit.CAU = pairedAQ;

        /* 三步写入 */
        E_partial->CMPA.half.CMPA = borrowCmp;
        E_partial->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
        E_partial->CMPA.half.CMPA = borrowCmp;

    }
    EDIS;

    Partial_ClearActive(act);
}

/*
 * Partial_ForceAbort()
 *
 * 异常/急停/兜底时强制恢复。
 */
void Partial_ForceAbort(void)
{
    if (g_partialActive.active)
        Partial_RestoreActive(&g_partialActive);

    Partial_ClearPrepared(&g_partialPrepared);
}

/* ==================== Deferred 写入保护 ==================== */
/*
 * 新方案中只有 Partial 所在模块的 borrowed compare 需要保护。
 * ePWM4 不被借用，不需要 defer。
 */

/* 写 CMPA（带 defer 保护） */
static void Partial_WriteCmpA(Uint16 globalModule, Uint16 cmpA)
{
    /* Partial 在 B 通道时借用的是本模块 CMPA */
    if (g_partialActive.active &&
        (g_partialActive.globalModule == globalModule) &&
        (g_partialActive.channelIsA == 0U))
    {
        g_partialActive.deferredBorrowCmp      = cmpA;
        g_partialActive.deferredBorrowCmpValid = 1;
        return;
    }

    ePWM_all[globalModule]->CMPA.half.CMPA = cmpA;
}

/* 写 CMPB（带 defer 保护） */
static void Partial_WriteCmpB(Uint16 globalModule, Uint16 cmpB)
{
    /* Partial 在 A 通道时借用的是本模块 CMPB */
    if (g_partialActive.active &&
        (g_partialActive.globalModule == globalModule) &&
        (g_partialActive.channelIsA == 1U))
    {
        g_partialActive.deferredBorrowCmp      = cmpB;
        g_partialActive.deferredBorrowCmpValid = 1;
        return;
    }

    ePWM_all[globalModule]->CMPB = cmpB;
}

/* ==================== 12 路 compare 批量写入 ==================== */

static void Partial_WriteAllCmp(Uint16 *cmpArray,
                                PartialPreparedCtx *prep)
{
    Uint16 i;
    /* 预计算 Partial 替换目标（若有） */
    Uint16 pReplaceModule = 0xFF;   /* P 桥臂中需替换的 localModule */
    Uint16 pReplaceIsA    = 0;
    Uint16 nReplaceModule = 0xFF;   /* N 桥臂 */
    Uint16 nReplaceIsA    = 0;

    if (prep->valid)
    {
        if (prep->arm == PARTIAL_ARM_P)
        {
            pReplaceModule = prep->localModule;
            pReplaceIsA    = prep->channelIsA;
        }
        else
        {
            nReplaceModule = prep->localModule;
            nReplaceIsA    = prep->channelIsA;
        }
    }

    /* P 桥臂 */
    for (i = 0; i < 3; ++i)
    {
        Uint16 cmpA = cmpArray[P_MMC_array[CMPA_IDX(i)].position_id];
        Uint16 cmpB = cmpArray[P_MMC_array[CMPB_IDX(i)].position_id];

        if (i == pReplaceModule)
        {
            if (pReplaceIsA) cmpA = prep->firstCmp;
            else             cmpB = prep->firstCmp;
        }

        Partial_WriteCmpA(i, cmpA);
        Partial_WriteCmpB(i, cmpB);
    }

    /* N 桥臂 */
    for (i = 0; i < 3; ++i)
    {
        Uint16 cmpA = cmpArray[N_MMC_array[CMPA_IDX(i)].position_id];
        Uint16 cmpB = cmpArray[N_MMC_array[CMPB_IDX(i)].position_id];

        if (i == nReplaceModule)
        {
            if (nReplaceIsA) cmpA = prep->firstCmp;
            else             cmpB = prep->firstCmp;
        }

        Partial_WriteCmpA(i + 3, cmpA);
        Partial_WriteCmpB(i + 3, cmpB);
    }
}

float32 Get_K_D_Method(float U_in)
{
    static Uint16 updata_duty_flag = 0;
    float32 K_ff = Get_K_ff_Method(U_in);
    if (++updata_duty_flag >= 10)
    {
        updata_duty_flag = 0;
        Duty_PID_Incremental_Calculate(&duty_PID, 10000, U_in);
    }
    return K_ff + duty_PID.output;
}

static void DecodeKDValue(float32 kdValue, Uint16 *kOut, float32 *dOut)
{
    int16 kInt;
    float32 frac;

    kInt = (int16)kdValue;
    frac = kdValue - (float32)kInt;

    if (frac < 0.0f) frac = 0.0f;
    else if (frac > 0.995f) 
    {
        frac = 0.0f;
        kInt += 1;
    }

    if (kInt < 0)   kInt = 0;
    if (kInt > MMC_NUMBER) kInt = MMC_NUMBER;

    *kOut = (Uint16)kInt;
    *dOut = 0.5f * frac;

    if (*dOut < PARTIAL_D_EPS) *dOut = 0.0f;
    // if (*dOut > 0.499f) *dOut = 0.499f;
}

/* ==================== Restore ISR ==================== */
/*
 * EPWM_RestoreISR
 *
 * PIE 向量：PieVectTable.EPWM4_INT
 * 触发源  ：ePWM4 PRD（计数器到达 TBPRD）
 *
 * 该中断由 Partial_TryInjectNow() 使能，
 * 每次只触发一次（ET_1ST），在本函数中关闭。
 */
// interrupt void EPWM_RestoreISR(void)
// {
//     SOFT_PWM3_TOGGLE;
//     /* 清中断标志 */
//     EPwm4Regs.ETCLR.bit.INT = 1;

//     if(changeState_flag_position != 6)
//     {
//         Change_K_Model_PWMWorkingStatus(changeState_flag_position, changeState_flag_State);
//         changeState_flag_position = 6;
//     }

//     /* ★ 快速路径：无 Partial 需要恢复，立即返回 ★
//      *
//      * 大部分子周期都会走这个路径（Partial 并非每个子周期都存在，
//      * 而且 P/N 交替时其中一个桥臂的子周期没有 Partial）。
//      */
//     if (g_partialActive.active)
//     {
//         Partial_RestoreActive(&g_partialActive);
//     }

//     /* ACK PIE */
//     PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
//     SOFT_PWM3_TOGGLE;
// }

interrupt void EPWM_GlobalISR(void)
{
    SOFT_PWM3_TOGGLE;
    Uint16 i;
    // 固定清 ePWM1 的中断标志
    EPwm1Regs.ETCLR.bit.INT = 1;

    // ======================= 第一次中断处理 =======================
    if (!g_firstIsrDone)
    {
        EALLOW;

        for (i = 0; i < 6; ++i)
        {
            volatile struct EPWM_REGS *E = ePWM_all[i];
            E->AQCSFRC.bit.CSFA = 0; // 禁用软件强制
            E->AQCSFRC.bit.CSFB = 0;
        }
        // 此时已经过了第一个CMP点，电平已经确认，后续动作时开始正常翻转
        for (i = 0; i < 6; ++i)
        {
            volatile struct EPWM_REGS *E = ePWM_all[i];
            // 使用AQCSFRC设置初始电平
            if (i < 3)
            {
                // 上桥臂
                if (P_MMC_array[CMPA_IDX(i)].position_id < K_value)
                {
                    // K模块：强制高，保持高
                    E->AQCTLA.bit.CAU = AQ_SET;
                }
                else
                {
                    // 非K模块：强制低，正常翻转
                    E->AQCTLA.bit.CAU = AQ_TOGGLE;
                }

                if (P_MMC_array[CMPB_IDX(i)].position_id < K_value)
                {
                    E->AQCTLB.bit.CBU = AQ_SET;
                }
                else
                {
                    E->AQCTLB.bit.CBU = AQ_TOGGLE;
                }
            }
            else
            {
                // 下桥臂
                Uint16 lowerIdx = i - 3;

                if (N_MMC_array[CMPA_IDX(lowerIdx)].position_id < K_value)
                {
                    // K模块：强制高，保持高
                    E->AQCTLA.bit.CAU = AQ_SET;
                }
                else
                {
                    // 非K模块：强制高，正常翻转
                    E->AQCTLA.bit.CAU = AQ_TOGGLE;
                }

                if (N_MMC_array[CMPB_IDX(lowerIdx)].position_id < K_value)
                {
                    E->AQCTLB.bit.CBU = AQ_SET;
                }
                else
                {
                    E->AQCTLB.bit.CBU = AQ_TOGGLE;
                }
            }
        }
        EDIS;

        g_firstIsrDone = 1;
    }

    if (g_partialActive.active)
    {
        Partial_RestoreActive(&g_partialActive);
    }

    if (g_ADCIsrFlag)
    {
        g_isrToggleCnt++;
        // 触发后延迟2us再启动,等待开关管完全关断
        StartAD760x();
    }
    else
    {
        g_ADCIsrFlag = 1;
    }

    PieCtrlRegs.PIEACK.all = EPWM_PIE_GROUP_ACK;
    SOFT_PWM3_TOGGLE;
}

// 加入排序，然后在dma的中断函数中调用
void BridgeArmSort(void)
{
    Uint16 tbprd;
    // 步骤 0：消费上一子周期构造的 Prepared，注入 secondCmp
    Partial_TryInjectNow();
    tbprd = EPwm1Regs.TBPRD;  /* 取当前 TBPRD，供 BuildPrepared 使用 */

    switch (g_isrToggleCnt & 0x01)
    {
    case Update_P_K_Arm:
    SOFT_PWM2_TOGGLE;
        /* --- 步骤 1：更新 P 桥臂电压与排序 --- */
        K_last_value_P = K_value;
        g_partialDuty_P = g_partialDuty;
        // 更新上桥臂的电压信息与预计的开通顺序,需要使用K_last_value,因为上周期的K模块数决定了要把几个模块放到最后面开通
        updata_voltage_information(UpdataUpperArm);
        
        /* ═══ 更新 K 与 D ═══ */
        if (SoftStart_IsActive())
        {
            SoftStart_OnFullCycleTick();
            /* 软启动期间：K/D 由状态机给出，不跟随输入电压 */
            SoftStart_GetCommand(&K_value, &g_partialDuty);
        }
        else
        {
            float32 kdValue;
            Uint16  newK;
            float32 newD;

            /* --- 步骤 2：更新 K 与 D --- */
            // U_in = AD7606_1_Buf[6] * 5 / 32768.0 * 400; // 400 = 8000kΩ / 20kΩ
            // TODO
            // U_in = (AD7606_1_Buf[6] / 32768.0f) * 2000.0f;
            float32 U_in_test = (AD7606_1_Buf[6] / 32768.0f) * 2000.0f;

            if (U_in < TARGET_LLC_VOLTAGE) U_in = TARGET_LLC_VOLTAGE;
            if (U_in > 800.0f) U_in = 800.0f;

            kdValue = Get_K_D_Method(U_in);
            DecodeKDValue(kdValue, &newK, &newD);
            K_value       = newK;
            g_partialDuty = newD;
        }

        // 更新对家K模块导通动作并且预测上周期的K模块上周期结束时的电压情况(切换K模块时先关断以前的再开启新的，顺序不能反)
        if (K_last_value_N != 0)
        {
            Uint16 divisor, average_sum, i;
            for (i = 0; i < MMC_NUMBER; i++)
            {
                if (N_MMC_array[i].last_position_id < K_last_value_N)
                {
                    Change_K_Model_PWMWorkingStatus(i, Reset_Old_Channel_N);
                    *updat_voltage_index_record_ptr = i;
                    updat_voltage_index_record_ptr++;
                }
                else if ((N_MMC_array[i].position_id == K_last_value_N) && (g_partialDuty_N > 0.0f))
                {
                    *updat_voltage_index_record_ptr = i;
                    updat_voltage_index_record_ptr++;
                }
                else
                {
                    predict_voltage_diff += (AD7606_2_Buf[i] - N_MMC_array[i].recent_voltage);
                }
            }
            /* ★ 除零保护：K_last_value_N == 6 时分母为 0 ★ */
            average_sum = K_last_value_N + ((g_partialDuty_N > 0.0f) ? 1U : 0U);
            divisor = MMC_NUMBER - average_sum;
            if (divisor > 0U)
            {
                predict_voltage_diff /= (int32)divisor;
                for (i = 0; i < average_sum; i++)
                {
                    N_MMC_array[updat_voltage_index_record[i]].recent_voltage = AD7606_2_Buf[updat_voltage_index_record[i]] - predict_voltage_diff;
                }
            }
            else
            {
                for (i = 0; i < average_sum; i++)
                {
                    N_MMC_array[updat_voltage_index_record[i]].recent_voltage = AD7606_2_Buf[updat_voltage_index_record[i]];
                }
            }
            predict_voltage_diff = 0;
            updat_voltage_index_record_ptr = updat_voltage_index_record; // 重置指针，准备下一次记录
        }
        if (K_value != 0)
        {
            // 更新对家桥臂的K模块PWM变换方式
            for (Uint16 i = 0; i < MMC_NUMBER; i++)
            {
                if (N_MMC_array[i].position_id < K_value)
                {
                    Change_K_Model_PWMWorkingStatus(i, Set_New_Channel_N);
                }
            }
        }

        // 更新方波频率
        if (SoftStart_FreqLoopEnabled())
        {
            EPWM_FrequencyUpdate();
        }

        // 更新上下桥臂电压误差
        // EPWM_ErrorUpdate(Update_P_K_Arm);

        /*═════════════════════════════════════════════════
         * 【B】步骤 6：为下一子周期（奇数，Update_N_K_Arm）
         *              构造 N 桥臂的 Partial 计划
         *
         * 下一子周期 N 桥臂会用 CMP_Values 表
         *   （因为 P 桥臂用 CMP_addError_Values，N 用 CMP_Values，
         *    奇偶交替，具体见你原有的逻辑）
         *
         * moduleBase = 3（ePWM4~6，全局索引 3~5）
         *═════════════════════════════════════════════════*/
        Partial_BuildPrepared(N_MMC_array, CMP_Values, 3, g_partialDuty, tbprd, &g_partialPrepared);

        /*═════════════════════════════════════════════════
         * 【B】步骤 7：写入下一子周期的全部 12 路 CMP
         *
         * 使用带 defer 保护的批量写入。
         * Partial 模块的 own CMP 会被 firstCmp 替换。
         * 被 Active 占用的寄存器走 defer 通道。
         *
         * 【重要】模块 B 阶段：CMP 表选择
         *   下一子周期 N 桥臂用 CMP_Values（本子周期你原代码里 CMPB 部分）
         *   下一子周期 P 桥臂用 CMP_Values（也用 CMP_Values，因为 err PID 影响的是 addError）
         *   实际上你原代码在 Update_P 里全都用 CMP_Values，在 Update_N 里全都用 CMP_addError_Values
         *   这个策略保持不变
         *═════════════════════════════════════════════════*/
        Partial_WriteAllCmp(CMP_Values, &g_partialPrepared);

        // 对上周期partial已经设置高电平、不反转的模块恢复电平翻转
        // 需要对本子周期partial的模块设置其下子周期为高电平、不反转
        if(g_partialDuty_N != 0)
        {
            for (Uint16 i = 0; i < MMC_NUMBER; i++)
            {
                if (N_MMC_array[i].last_position_id == K_last_value_N)
                {
                    Change_K_Model_PWMWorkingStatus(i, Reset_Old_Channel_N);
                    break;
                }
            }
        }
        // if (g_partialDuty != 0)
        // {
        //     for (Uint16 i = 0; i < MMC_NUMBER; i++)
        //     {
        //         if (P_MMC_array[i].last_position_id == K_value)
        //         {
        //             changeState_flag_position = i;
        //             changeState_flag_State = Set_New_Channel_P;
        //             // Change_K_Model_PWMWorkingStatus(i, Set_New_Channel_P);
        //             break;
        //         }
        //     }
        // }

    SOFT_PWM2_TOGGLE;
        break;

    // &0x01后为1
    case Update_N_K_Arm:
    SOFT_PWM1_TOGGLE;
        /* --- 步骤 1：更新 N 桥臂电压与排序 --- */
        K_last_value_N  = K_value;
        g_partialDuty_N = g_partialDuty;
        updata_voltage_information(UpdataLowerArm);

        // 更新上下桥臂电压误差
        // EPWM_ErrorUpdate(Update_N_K_Arm);

        // 更新对家K模块导通动作并且预测上周期的K模块上周期结束时的电压情况(切换K模块时先关断以前的再开启新的，顺序不能反)
        if (K_last_value_P != 0)
        {
            Uint16 divisor, average_sum;
            for (Uint16 i = 0; i < MMC_NUMBER; i++)
            {
                if (P_MMC_array[i].last_position_id < K_last_value_P)
                {
                    Change_K_Model_PWMWorkingStatus(i, Reset_Old_Channel_P);
                    *updat_voltage_index_record_ptr = i;
                    updat_voltage_index_record_ptr++;
                }
                else if ((P_MMC_array[i].position_id == K_last_value_P) && (g_partialDuty_P > 0.0f))
                {
                    *updat_voltage_index_record_ptr = i;
                    updat_voltage_index_record_ptr++;
                }
                else
                {
                    predict_voltage_diff += (AD7606_1_Buf[i] - P_MMC_array[i].recent_voltage);
                }
            }
            /* ★ 除零保护：K_last_value_N == 6 时分母为 0 ★ */
            average_sum = K_last_value_P + ((g_partialDuty_P > 0.0f) ? 1U : 0U);
            divisor = MMC_NUMBER - average_sum;

            if (divisor > 0U)
            {
                predict_voltage_diff /= (int32)divisor;
                for (Uint16 i = 0; i < average_sum; i++)
                {
                    P_MMC_array[updat_voltage_index_record[i]].recent_voltage = AD7606_1_Buf[updat_voltage_index_record[i]] - predict_voltage_diff;
                }
            }
            else
            {
                for (Uint16 i = 0; i < average_sum; i++)
                {
                    P_MMC_array[updat_voltage_index_record[i]].recent_voltage = AD7606_1_Buf[updat_voltage_index_record[i]];
                }
            }
            predict_voltage_diff = 0;
            updat_voltage_index_record_ptr = updat_voltage_index_record; // 重置指针，准备下一次记录
        }
        if (K_value != 0)
        {
            // 更新对家桥臂的K模块PWM变换方式
            for (Uint16 i = 0; i < MMC_NUMBER; i++)
            {
                if (P_MMC_array[i].position_id < K_value)
                {
                    Change_K_Model_PWMWorkingStatus(i, Set_New_Channel_P);
                }
            }
        }

        /*═════════════════════════════════════════════════
         * 【B】步骤 4：为下一子周期（偶数，Update_P_K_Arm）
         *              构造 P 桥臂的 Partial 计划
         *
         * moduleBase = 0（ePWM1~3，全局索引 0~2）
         *═════════════════════════════════════════════════*/
        Partial_BuildPrepared(P_MMC_array, CMP_addError_Values, 0, g_partialDuty, tbprd, &g_partialPrepared);

        // Partial_BuildPrepared(N_MMC_array, CMP_addError_Values, 3, g_partialDuty, tbprd, &g_partialPrepared);

        /*═════════════════════════════════════════════════
         * 【B】步骤 5：写入下一子周期的 12 路 CMP
         *═════════════════════════════════════════════════*/
        Partial_WriteAllCmp(CMP_addError_Values, &g_partialPrepared);

        if(g_partialDuty_P != 0)
        {
            for (Uint16 i = 0; i < MMC_NUMBER; i++)
            {
                if (P_MMC_array[i].last_position_id == K_last_value_P)
                {
                    Change_K_Model_PWMWorkingStatus(i, Reset_Old_Channel_P);
                    break;
                }
            }
        }
        // 对这个做特殊处理
        // if (g_partialDuty != 0)
        // {
        //     for (Uint16 i = 0; i < MMC_NUMBER; i++)
        //     {
        //         if (N_MMC_array[i].last_position_id == K_value)
        //         {
        //             changeState_flag_position = i;
        //             changeState_flag_State = Set_New_Channel_N;
        //             // Change_K_Model_PWMWorkingStatus(i, Set_New_Channel_N);
        //             break;
        //         }
        //     }
        // }
        

    SOFT_PWM1_TOGGLE;
        break;

    default:
        break;
    }
}

void EPWM_ResetISRFlags(void)
{
    /* --- ISR 标志 --- */
    g_firstIsrDone = 0;
    g_isrToggleCnt = 0;
    g_ADCIsrFlag   = 0;

    /* --- Partial 状态 --- */
    g_partialDuty   = 0.0f;
    g_partialDuty_P = 0.0f;
    g_partialDuty_N = 0.0f;

    Partial_ClearPrepared(&g_partialPrepared);
    Partial_ClearActive(&g_partialActive);

    /* --- 调试统计 --- */
    g_partialInjectSkipCnt  = 0;
    g_partialRestoreMissCnt = 0;
}

// void EnableEPWM4Interrupt(void)
// {
//     EALLOW;
//     EPwm4Regs.ETCLR.bit.INT = 1;       // 再次清标志位，防止上次残留
//     EPwm4Regs.ETSEL.bit.INTEN = 1;     // 启用中断
//     PieCtrlRegs.PIEIER3.bit.INTx4 = 1; // PIE组3使能ePWM4
//     IER |= M_INT3;                     // 启用CPU组3中断
//     EDIS;
// }

// void DisableEPWM4Interrupt(void)
// {
//     EALLOW;
//     EPwm4Regs.ETSEL.bit.INTEN = 0;     // 禁止ePWM4中断
//     PieCtrlRegs.PIEIER3.bit.INTx4 = 0; // 禁止PIE通道中断
//     EDIS;
// }

// // 在这里进行K模块的ADC读取触发等操作
// interrupt void EPWM4_ISR(void)
// {
//     EPwm4Regs.ETCLR.bit.INT = 1;
//     // g_isrToggleCnt++;
//     // StartAD760x();
//     PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
// }

void EPWM_ResetControlState(void)
{
    /* --- 频率 PID --- */
    fre_PID.prev_error  = 0.0f;
    fre_PID.prev2_error = 0.0f;
    fre_PID.output      = INIT_FREQUENCE;

    /* --- 误差 PID --- */
    err_PID.prev_error  = 0.0f;
    err_PID.prev2_error = 0.0f;
    err_PID.output      = INIT_ERROR;

    /* --- 占空比 PID --- */
    duty_PID.prev_error  = 0.0f;
    duty_PID.prev2_error = 0.0f;
    duty_PID.output      = INIT_DUTY_CYCLE;

    /* --- 桥臂平均电压 --- */
    arm_per_voltage_P = 0.0f;
    arm_per_voltage_N = 0.0f;

    /* --- K 值历史 --- */
    K_last_value_P = 0;
    K_last_value_N = 0;

    /* --- 电压预测辅助 --- */
    predict_voltage_diff = 0;
    updat_voltage_index_record_ptr = updat_voltage_index_record;
}

void EPWM_SafeStop(void)
{
    EALLOW;

    /* 强制清理 Partial 状态 */
    Partial_ForceAbort();

    /* 关闭所有 ePWM 中断 */
    EPwm1Regs.ETSEL.bit.INTEN = 0;
    EPwm4Regs.ETSEL.bit.INTEN = 0;
    PieCtrlRegs.PIEIER3.all = 0;

    /* 切换回 GPIO 模式 */
    GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO8  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO9  = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;

    /* 全部置高 */
    GpioDataRegs.GPASET.all = 0x00000FFF;

    EDIS;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    g_ADCIsrFlag = 0;

    /* 重置控制状态 */
    EPWM_ResetISRFlags();
    EPWM_ResetControlState();

    printf("[EPWM] SafeStop\r\n");
}
