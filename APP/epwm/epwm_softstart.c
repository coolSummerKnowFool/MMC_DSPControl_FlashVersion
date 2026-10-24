#include "epwm_softstart.h"
#include "epwm_combine.h"
#include "basic_control.h"
#include "ad760x.h"
#include "dma_xintf.h"
#include "printf.h"

/* 关键路径函数放 RAM 执行 */
#pragma CODE_SECTION(SoftStart_OnFullCycleTick, ".TI.ramfunc")
#pragma CODE_SECTION(SoftStart_GetCommand,      ".TI.ramfunc")
#pragma CODE_SECTION(SoftStart_IsActive,        ".TI.ramfunc")
#pragma CODE_SECTION(SoftStart_FreqLoopEnabled, ".TI.ramfunc")
#pragma CODE_SECTION(SS_ShouldAdvanceK,   ".TI.ramfunc")
#pragma CODE_SECTION(SS_PostMsg,          ".TI.ramfunc")
#if SOFTSTART_USE_VOLT_CRITERION
#pragma CODE_SECTION(SS_IsVoltageStable,  ".TI.ramfunc")
#endif

volatile SoftStartManager g_softStart = {0};
volatile SoftStartMsg     g_ssMsg     = {0};

/* ══════════════════════════════════════════════════════
 *                     内部辅助函数
 * ══════════════════════════════════════════════════════ */

/*
 * 投递异步消息（中断安全，不做 printf）
 * 若上一条还没被主循环取走，直接覆盖（只保留最新状态）
 */
static void SS_PostMsg(Uint16 type, Uint16 k, float32 d, Uint32 cycles)
{
    g_ssMsg.type    = type;
    g_ssMsg.k       = k;
    g_ssMsg.d       = d;
    g_ssMsg.cycles  = cycles;
    g_ssMsg.pending = 1;
}

/*
 * SS_DecodeKD()
 *
 * 拆解 Get_K_D_Method() 返回值：
 *   整数部分 = K
 *   小数部分 / 2 = D
 *
 * 与 epwm_combine.c 中的 DecodeKDValue 保持一致的边界处理
 */
static void SS_DecodeKD(float32 kdValue, Uint16 *kOut, float32 *dOut)
{
    int16   kInt;
    float32 frac;

    kInt = (int16)kdValue;
    frac = kdValue - (float32)kInt;

    if (frac < 0.0f)
    {
        frac = 0.0f;
    }
    else if (frac > 0.99f)
    {
        frac = 0.0f;
        kInt += 1;
    }

    if (kInt < 0)          kInt = 0;
    if (kInt > MMC_NUMBER) kInt = MMC_NUMBER;

    *kOut = (Uint16)kInt;
    *dOut = 0.5f * frac;

    /* D 边界死区（与 Test2 实测一致） */
    if (*dOut < PARTIAL_D_EPS) *dOut = 0.0f;
    if (*dOut > PARTIAL_D_MAX) *dOut = PARTIAL_D_MAX;
}

/*
 * SS_TriggerOneSample()
 *
 * 手动触发一次 AD7606 转换并等待 DMA 完成。
 *
 * 注意：此时 g_ADCIsrFlag == 0，DMA 中断中的
 *       dma_done_count 不会累加，所以不会误触发 BridgeArmSort()。
 *       数据仍会被正常搬运到 AD7606_x_Buf。
 */
static void SS_TriggerOneSample(void)
{
    StartAD760x();
    DELAY_US(SOFTSTART_ADC_WAIT_US);
}

/*
 * SS_PreSample()
 *
 * 软启动前的预采样：连续触发若干次，让数据稳定。
 */
static void SS_PreSample(void)
{
    Uint16 i;
    for (i = 0; i < SOFTSTART_PRESAMPLE_COUNT; i++)
    {
        SS_TriggerOneSample();
    }
}

/*
 * SS_SortIndexByVoltage()
 *
 * 对电压数组做索引升序排序（插入排序，n=6 足够快）。
 * 输出 sortedIdx[j] = 第 j 小的元素在原数组中的下标。
 */
static void SS_SortIndexByVoltage(const volatile Uint16 *src,
                                  Uint16 *sortedIdx,
                                  Uint16 n)
{
    Uint16 i;
    int16  j;

    for (i = 0; i < n; i++)
        sortedIdx[i] = i;

    for (i = 1; i < n; i++)
    {
        Uint16 keyIdx = sortedIdx[i];
        Uint16 keyVal = src[keyIdx];
        j = (int16)i - 1;

        while ((j >= 0) && (src[sortedIdx[j]] > keyVal))
        {
            sortedIdx[j + 1] = sortedIdx[j];
            j--;
        }
        sortedIdx[j + 1] = keyIdx;
    }
}

/*
 * SS_InitMMCArrays()
 *
 * 用预采样结果初始化 P/N 两个桥臂的 mmc_info 数组：
 *   - recent_voltage  ← 实测值
 *   - voltage_diff    ← 0
 *   - position_id     ← 按电压升序（电压低的先开通）
 *
 * 同时初始化指针数组的顺序。
 */
static void SS_InitMMCArrays(void)
{
    Uint16 upperSorted[MMC_NUMBER];
    Uint16 lowerSorted[MMC_NUMBER];
    Uint16 i, j;

    SS_SortIndexByVoltage(AD7606_1_Buf, upperSorted, MMC_NUMBER);
    SS_SortIndexByVoltage(AD7606_2_Buf, lowerSorted, MMC_NUMBER);

    MMC_Init(P_MMC_array, N_MMC_array);

    for (i = 0; i < MMC_NUMBER; i++)
    {
        P_MMC_array[i].recent_voltage = AD7606_1_Buf[i];
        N_MMC_array[i].recent_voltage = AD7606_2_Buf[i];

        /* 按电压升序设置 position_id（电压低 → position_id 小 → 先开通） */
        for (j = 0; j < MMC_NUMBER; j++)
        {
            if (upperSorted[j] == i)
            {
                P_MMC_array[i].position_id      = j;
                P_MMC_array[i].last_position_id = j;
            }
            if (lowerSorted[j] == i)
            {
                N_MMC_array[i].position_id      = j;
                N_MMC_array[i].last_position_id = j;
            }
        }
    }
}

#if SOFTSTART_USE_VOLT_CRITERION
/*
 * SS_IsVoltageStable()
 *
 * 电压稳定判据：两个桥臂内部的电压极差都小于阈值。
 *
 * 注意：K 值较大时（尤其 K=6），大部分模块的 recent_voltage
 *       不会被更新，此判据会一直返回上次的结果。
 *       因此仅作为"最小停留时间之后的加速手段"，
 *       不能替代最大停留时间的兜底。
 */
static Uint16 SS_IsVoltageStable(void)
{
    Uint16 i;
    Uint16 pMin, pMax, nMin, nMax;

    pMin = pMax = P_MMC_array[0].recent_voltage;
    nMin = nMax = N_MMC_array[0].recent_voltage;

    for (i = 1; i < MMC_NUMBER; i++)
    {
        Uint16 pv = P_MMC_array[i].recent_voltage;
        Uint16 nv = N_MMC_array[i].recent_voltage;

        if (pv < pMin) pMin = pv;
        if (pv > pMax) pMax = pv;
        if (nv < nMin) nMin = nv;
        if (nv > nMax) nMax = nv;
    }

    if (((pMax - pMin) < SOFTSTART_VOLT_SPREAD_TH) &&
        ((nMax - nMin) < SOFTSTART_VOLT_SPREAD_TH))
    {
        return 1;
    }
    return 0;
}
#endif

/*
 * SS_ShouldAdvanceK()
 *
 * 判断 K 递减阶段是否可以推进到下一步。
 *
 * 逻辑：
 *   ① 未达最小停留时间 → 不推进
 *   ② 达到最大停留时间 → 强制推进（兜底）
 *   ③ 中间区间 → 若电压连续稳定足够周期数则推进
 */
static Uint16 SS_ShouldAdvanceK(void)
{
#if SOFTSTART_USE_VOLT_CRITERION
    /* ① 最小停留时间未满足 */
    if (g_softStart.stageCycles < g_softStart.kDwellMinCycles)
        return 0;
#endif

    /* ② 超时强制推进 */
    if (g_softStart.stageCycles >= g_softStart.kDwellMaxCycles)
        return 1;

#if SOFTSTART_USE_VOLT_CRITERION
    /* ③ 电压稳定判据 */
    if (SS_IsVoltageStable())
    {
        g_softStart.voltStableCycles++;
        if (g_softStart.voltStableCycles >= SOFTSTART_VOLT_STABLE_CYCLES)
            return 1;
    }
    else
    {
        g_softStart.voltStableCycles = 0;
    }
#endif

    return 0;
}

/* ══════════════════════════════════════════════════════
 *                       对外接口
 * ══════════════════════════════════════════════════════ */

void SoftStart_Reset(void)
{
    g_softStart.active         = 0;
    g_softStart.complete       = 0;
    g_softStart.freqLoopEnable = 0;
    g_softStart.stage          = SS_STAGE_IDLE;

    g_softStart.targetKD = 0.0f;
    g_softStart.targetK  = 0;
    g_softStart.targetD  = 0.0f;

    g_softStart.currentK = 0;
    g_softStart.currentD = 0.0f;

    g_softStart.stageCycles      = 0;
    g_softStart.voltStableCycles = 0;
    g_softStart.totalCycles      = 0;
    g_softStart.kStepCount       = 0;

    /* 预计算周期阈值 */
    g_softStart.kDwellMinCycles   = SOFTSTART_MS_TO_CYCLES(SOFTSTART_K_DWELL_MIN_MS);
    g_softStart.kDwellMaxCycles   = SOFTSTART_MS_TO_CYCLES(SOFTSTART_K_DWELL_MAX_MS);
    g_softStart.targetKHoldCycles = SOFTSTART_MS_TO_CYCLES(SOFTSTART_TARGET_K_HOLD_MS);
    g_softStart.partialPrimeCycles= SOFTSTART_MS_TO_CYCLES(SOFTSTART_PARTIAL_PRIME_MS);
    g_softStart.finalSettleCycles = SOFTSTART_MS_TO_CYCLES(SOFTSTART_FINAL_SETTLE_MS);

    g_ssMsg.pending = 0;
}

void SoftStart_Init(void)
{
    SoftStart_Reset();
}

Uint16 SoftStart_Start(void)
{
    float32 kdValue;
    Uint16  tK;
    float32 tD;

    SoftStart_Reset();

    /* ═══ 步骤 1：ADC 预采样 ═══ */  
    // 采样四次，每次间隔30us
    SS_PreSample();

    /* ═══ 步骤 2：读取输入电压 ═══ */
    // TODO
    U_in = 260;
    // U_in = (float)AD7606_1_Buf[6] / 32768.0f * 2000.0f;

    if (U_in < TARGET_LLC_VOLTAGE)
    {
        // printf("[SoftStart] ERROR: U_in=%.1fV too low (min=%.1fV)\r\n",
        //        U_in, TARGET_LLC_VOLTAGE);
        // g_softStart.stage = SS_STAGE_FAULT;
        // return 0;
        U_in = 260.0f;
    }
    if (U_in > 800.0)
    {
        printf("[SoftStart] ERROR: U_in=%.1fV too high\r\n", U_in);
        g_softStart.stage = SS_STAGE_FAULT;
        return 0;
    }

    /* ═══ 步骤 3：计算目标 K / D ═══
     *
     * 使用前馈公式 Get_K_ff_Method（不含 duty PID 积分项），
     * 因为软启动阶段 PID 还没有历史数据。
     */
    kdValue = Get_K_ff_Method(U_in);
    SS_DecodeKD(kdValue, &tK, &tD);

    g_softStart.targetKD = kdValue;
    g_softStart.targetK  = tK;
    g_softStart.targetD  = tD;

    /* ═══ 步骤 4：初始化 MMC 数组 ═══ */
    SS_InitMMCArrays();

    /* ═══ 步骤 5：设置初始状态 K=6, D=0 ═══ */
    g_softStart.currentK = MMC_NUMBER;
    g_softStart.currentD = 0.0f;

    /* 同步到全局变量，供 EPWM_SwitchToClosedLoop 初始化 AQ 使用 */
    K_value         = MMC_NUMBER;
    K_last_value_P  = MMC_NUMBER;
    K_last_value_N  = MMC_NUMBER;
    g_partialDuty   = 0.0f;
    g_partialDuty_P = 0.0f;
    g_partialDuty_N = 0.0f;

    /* ═══ 步骤 6：确定起始阶段 ═══ */
    if (g_softStart.currentK > g_softStart.targetK)
    {
        g_softStart.stage = SS_STAGE_RAMP_K;
    }
    else if (g_softStart.targetD > 0.0f)
    {
        /* K 已经等于目标（targetK=6 的极端情况），直接进保持阶段 */
        g_softStart.stage = SS_STAGE_TARGET_K_HOLD;
    }
    else
    {
        g_softStart.stage = SS_STAGE_FINAL_SETTLE;
    }

    g_softStart.active = 1;

    printf("[SoftStart] ===== START =====\r\n");
    printf("  U_in     = %.1f V\r\n", U_in);
    printf("  targetKD = %.4f\r\n", kdValue);
    printf("  targetK  = %d\r\n", g_softStart.targetK);
    printf("  targetD  = %.4f\r\n", g_softStart.targetD);
    printf("  K ramp   : 6 -> %d\r\n", g_softStart.targetK);
    printf("  dwell    : %lu~%lu cycles/step\r\n",
           g_softStart.kDwellMinCycles, g_softStart.kDwellMaxCycles);
    printf("=========================\r\n");

    return 1;
}

Uint16 SoftStart_IsActive(void)      { return g_softStart.active; }
Uint16 SoftStart_IsComplete(void)    { return g_softStart.complete; }
Uint16 SoftStart_FreqLoopEnabled(void) { return g_softStart.freqLoopEnable; }

void SoftStart_GetCommand(volatile Uint16 *k, volatile float32 *d)
{
    if (k) *k = g_softStart.currentK;
    if (d) *d = g_softStart.currentD;
}

/*
 * SoftStart_OnFullCycleTick()
 *
 * 【调用位置】BridgeArmSort() 的 Update_P_K_Arm 分支，K/D 赋值之前
 * 【调用频率】每个完整周期一次（= 2 个子周期）
 *
 * 状态机流转：
 *
 *   RAMP_K ──(K 降到 targetK)──┬─(D>0)─→ TARGET_K_HOLD ─→ PARTIAL_PRIME ─┐
 *                              │                                          │
 *                              └─(D=0)──────────────────────────────────→ FINAL_SETTLE
 *                                                                          │
 *                                                                          ↓
 *                                                                        DONE
 */
void SoftStart_OnFullCycleTick(void)
{
    if (!g_softStart.active)
        return;

    g_softStart.stageCycles++;
    g_softStart.totalCycles++;

    switch (g_softStart.stage)
    {
    /* ══════════ 阶段 1：K 递减 ══════════ */
    case SS_STAGE_RAMP_K:
        if (SS_ShouldAdvanceK())
        {
            Uint16 elapsed = (Uint16)g_softStart.stageCycles;

            g_softStart.stageCycles      = 0;
            // g_softStart.voltStableCycles = 0;

            if (g_softStart.currentK > g_softStart.targetK)
            {
                /* 递减一步 */
                g_softStart.currentK--;
                g_softStart.currentD = 0.0f;
                g_softStart.kStepCount++;

                SS_PostMsg(SS_MSG_K_STEP, g_softStart.currentK, 0.0f, elapsed);

                /* 递减后若已到目标，下一拍会走下面的分支 */
            }

            /* 检查是否已到达目标 K */
            if (g_softStart.currentK <= g_softStart.targetK)
            {
                g_softStart.currentK = g_softStart.targetK;

                if (g_softStart.targetD > 0.0f)
                {
                    g_softStart.stage = SS_STAGE_TARGET_K_HOLD;
                    SS_PostMsg(SS_MSG_ENTER_TARGET_HOLD,
                               g_softStart.currentK, 0.0f, 0);
                }
                else
                {
                    g_softStart.stage = SS_STAGE_FINAL_SETTLE;
                    SS_PostMsg(SS_MSG_ENTER_SETTLE,
                               g_softStart.currentK, 0.0f, 0);
                }
            }
        }
        break;

    /* ══════════ 阶段 2：targetK 保持（D=0）══════════ */
    case SS_STAGE_TARGET_K_HOLD:
        if (g_softStart.stageCycles >= g_softStart.targetKHoldCycles)
        {
            g_softStart.stageCycles = 0;

            /* 启用 Partial */
            g_softStart.currentD = g_softStart.targetD;
            g_softStart.stage    = SS_STAGE_PARTIAL_PRIME;

            SS_PostMsg(SS_MSG_ENTER_PARTIAL,
                       g_softStart.currentK, g_softStart.currentD, 0);
        }
        break;

    /* ══════════ 阶段 3：Partial 预热 ══════════ */
    case SS_STAGE_PARTIAL_PRIME:
        if (g_softStart.stageCycles >= g_softStart.partialPrimeCycles)
        {
            g_softStart.stageCycles = 0;
            g_softStart.stage       = SS_STAGE_FINAL_SETTLE;

            SS_PostMsg(SS_MSG_ENTER_SETTLE,
                       g_softStart.currentK, g_softStart.currentD, 0);
        }
        break;

    /* ══════════ 阶段 4：最终稳定 ══════════ */
    case SS_STAGE_FINAL_SETTLE:
        if (g_softStart.stageCycles >= g_softStart.finalSettleCycles)
        {
            g_softStart.stageCycles    = 0;
            g_softStart.stage          = SS_STAGE_DONE;
            g_softStart.active         = 0;
            g_softStart.complete       = 1;
            g_softStart.freqLoopEnable = 1;   /* ★ 开启频率闭环 */

            SS_PostMsg(SS_MSG_COMPLETE,
                       g_softStart.currentK, g_softStart.currentD,
                       g_softStart.totalCycles);
        }
        break;

    default:
        break;
    }
}

/*
 * SoftStart_ProcessMessages()
 *
 * 【调用位置】main() 主循环
 *
 * 把中断中投递的状态变化消息打印出来，避免在中断里 printf。
 */
void SoftStart_ProcessMessages(void)
{
    Uint16  type;
    Uint16  k;
    float32 d;
    Uint32  cycles;

    if (!g_ssMsg.pending)
        return;

    /* 快照后立即清标志，减少与中断的竞争窗口 */
    type   = g_ssMsg.type;
    k      = g_ssMsg.k;
    d      = g_ssMsg.d;
    cycles = g_ssMsg.cycles;
    g_ssMsg.pending = 0;

    switch (type)
    {
    case SS_MSG_K_STEP:
        printf("[SS] K -> %d  (prev stage %lu cyc)\r\n", k, cycles);
        break;

    case SS_MSG_ENTER_TARGET_HOLD:
        printf("[SS] Target K=%d reached, holding (D=0)...\r\n", k);
        break;

    case SS_MSG_ENTER_PARTIAL:
        printf("[SS] Partial ON: K=%d D=%.4f\r\n", k, d);
        break;

    case SS_MSG_ENTER_SETTLE:
        printf("[SS] Final settle: K=%d D=%.4f\r\n", k, d);
        break;

    case SS_MSG_COMPLETE:
        printf("[SS] ===== COMPLETE =====\r\n");
        printf("  final K = %d, D = %.4f\r\n", k, d);
        printf("  total   = %lu cycles (%lu ms)\r\n",
               cycles, cycles * 1000UL / SOFTSTART_FULLCYCLE_HZ);
        printf("  freq closed-loop ENABLED\r\n");
        printf("=======================\r\n");
        break;

    default:
        break;
    }
}

void SoftStart_PrintStatus(void)
{
    static const char *stageName[] = {
        "IDLE", "RAMP_K", "TARGET_HOLD", "PARTIAL_PRIME",
        "FINAL_SETTLE", "DONE", "FAULT"
    };
    Uint16 s = (Uint16)g_softStart.stage;
    if (s > SS_STAGE_FAULT) s = SS_STAGE_FAULT;

    printf("[SS] stage=%s act=%d cmpl=%d freqLoop=%d\r\n",
           stageName[s], g_softStart.active,
           g_softStart.complete, g_softStart.freqLoopEnable);
    printf("[SS] cur K=%d D=%.4f | target K=%d D=%.4f (KD=%.4f)\r\n",
           g_softStart.currentK, g_softStart.currentD,
           g_softStart.targetK, g_softStart.targetD, g_softStart.targetKD);
    printf("[SS] stageCyc=%lu totalCyc=%lu kSteps=%d voltStable=%lu\r\n",
           g_softStart.stageCycles, g_softStart.totalCycles,
           g_softStart.kStepCount, g_softStart.voltStableCycles);
}
