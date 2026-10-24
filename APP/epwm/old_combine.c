// #include "epwm_combine.h"
// #include "printf.h"
// #include "leds.h"
// #include "ad760x.h"
// #include "basic_control.h"
// #include "dma_xintf.h"
// #include "epwm_softstart.h"
// // #include <string.h> // for memcpy if needed

// /* ==================== ramfunc 声明 ==================== */
// /* 必须进 RAM：中断热路径 */
// #pragma CODE_SECTION(EPWM_GlobalISR,                    ".TI.ramfunc")
// #pragma CODE_SECTION(EPWM_LowerISR,                     ".TI.ramfunc")
// #pragma CODE_SECTION(Partial_InjectSecondEdge,           ".TI.ramfunc")
// #pragma CODE_SECTION(Partial_RestoreActiveCtx,           ".TI.ramfunc")
// #pragma CODE_SECTION(PartialHold_ReleaseAtMainIsr,       ".TI.ramfunc")
// #pragma CODE_SECTION(PartialHold_ArmFromRestore,         ".TI.ramfunc")
// #pragma CODE_SECTION(FindPendingEpwmModule,              ".TI.ramfunc")
// #pragma CODE_SECTION(EnableModuleCmpInterrupt,           ".TI.ramfunc")
// #pragma CODE_SECTION(DisableModuleCmpInterrupt,          ".TI.ramfunc")
// #pragma CODE_SECTION(TryArmUpperMainInterrupt,           ".TI.ramfunc")
// #pragma CODE_SECTION(QueueUpperMainInterrupt,            ".TI.ramfunc")
// #pragma CODE_SECTION(BridgeArmSort,                      ".TI.ramfunc")
// #pragma CODE_SECTION(Partial_WriteAllCmpForNextSubcycle, ".TI.ramfunc")
// #pragma CODE_SECTION(Partial_WriteArmCmpForNextSubcycle, ".TI.ramfunc")
// #pragma CODE_SECTION(Partial_WriteCmpAOrDefer,           ".TI.ramfunc")
// #pragma CODE_SECTION(Partial_WriteCmpBOrDefer,           ".TI.ramfunc")
// #pragma CODE_SECTION(Partial_GetActiveByModule,          ".TI.ramfunc")

// /* 推荐进 RAM：每周期调用的控制计算路径 */
// #pragma CODE_SECTION(Partial_BuildPrepared,              ".TI.ramfunc")
// #pragma CODE_SECTION(FindMaxCmp,                         ".TI.ramfunc")
// #pragma CODE_SECTION(Change_K_Model_PWMWorkingStatus,    ".TI.ramfunc")
// #pragma CODE_SECTION(EPWM_ErrorUpdate,                   ".TI.ramfunc")
// #pragma CODE_SECTION(EPWM_FrequencyUpdate,               ".TI.ramfunc")
// #pragma CODE_SECTION(Error_PID_Incremental_Calculate,    ".TI.ramfunc")
// #pragma CODE_SECTION(Frequence_PID_Incremental_Calculate,".TI.ramfunc")
// #pragma CODE_SECTION(Duty_PID_Incremental_Calculate,     ".TI.ramfunc")
// #pragma CODE_SECTION(change_CMP_Values_BasedOnError,     ".TI.ramfunc")
// #pragma CODE_SECTION(get_average_recent_voltage,         ".TI.ramfunc")
// #pragma CODE_SECTION(DecodeKDValue,                      ".TI.ramfunc")
// #pragma CODE_SECTION(Get_K_D_Method,                     ".TI.ramfunc")
// #pragma CODE_SECTION(DisableAllModuleInterrupts,         ".TI.ramfunc")


// // ---------------- 全局变量定义 ----------------
// // 索引含义： 0=CMPA0, 1=CMPB0, 2=CMPA1, 3=CMPB1, 4=CMPA2, 5=CMPB2
// volatile Uint16 CMP_Values[2 * PWM_PAIR_COUNT] = {450, 500, 550, 600, 650, 700};          // 存放所有比较值，便于排序
// volatile Uint16 CMP_addError_Values[2 * PWM_PAIR_COUNT] = {450, 500, 550, 600, 650, 700}; // 存放所有比较值，便于排序

// volatile PID fre_PID = {fre_PID_P, fre_PID_I, fre_PID_D, 0.0, 0.0, INIT_FREQUENCE};
// volatile PID err_PID = {err_PID_P, err_PID_I, err_PID_D, 0.0, 0.0, INIT_ERROR};
// volatile PID duty_PID = {duty_PID_P, duty_PID_I, duty_PID_D, 0.0, 0.0, INIT_DUTY_CYCLE};
// volatile float32 arm_per_voltage_P = 0.0;
// volatile float32 arm_per_voltage_N = 0.0;

// volatile Uint16 updat_voltage_index_record[MMC_NUMBER] = {0};
// volatile Uint16 *updat_voltage_index_record_ptr = updat_voltage_index_record; // 指向上面数组的指针，便于传参
// volatile int32 predict_voltage_diff = 0;                                      // 存放预测的电压差值（单位是AD7606的码值）
// #define CMPA_IDX(i) (2 * (i))                                                 // 给 ePWM i 的 A 通道的 slot 索引（0/2/4）
// #define CMPB_IDX(i) (2 * (i) + 1)                                             // 给 ePWM i 的 B 通道的 slot 索引（1/3/5）

// // 记录当前被 arm 为中断源的模块与比较类型
// // TODO 这两个量被替换掉了，后面可能要删除，注意检查
// // static int16 g_prevModule = -1;   // 0..2 对应 ePWM1..ePWM3
// // static int16 g_prevIsA = -1;      // 1->CMPA,0->CMPB

// /*
//  * 当前正在使用的上桥臂主中断源。
//  * 每个子周期由 P_MMC_array 中 position_id == 5 的那个 compare 触发。
//  * 这个时刻：上下桥臂的所有原始半插入 compare 都已完成。
//  */
// static int16  g_upperCurrModule = -1;  /* 0..2，对应 ePWM1..3 */
// static int16  g_upperCurrIsA    = -1;  /* 1=CMPA, 0=CMPB */

// /* BridgeArmSort 计算出的"下一次主中断源" */
// static int16  g_upperNextModule = -1;
// static int16  g_upperNextIsA    = -1;
// static Uint16 g_upperNextPending = 0;  /* 1=待挂接（可能因模块冲突延后） */

// volatile float32 g_partialDuty = 0.0f;

// volatile PartialPreparedCtx g_partialPrepared = {0};
// volatile PartialActiveCtx   g_partialActive   = {0};
// static volatile PartialHoldCtx g_partialHold = {0};

// volatile Uint32 g_isrToggleCnt = 0; // 中断切换计数器，计数为奇数时为下桥臂更新，计数为偶数为上桥臂更新
// static Uint16 g_firstIsrDone = 0; // 标记是否已执行第一次中断（用于禁用ZRO动作）
// Uint16 g_ADCIsrFlag = 0; // 标记是否进入ADC的中断内的桥臂控制函数(预充电期间不执行，闭环期间执行)

// volatile Uint16 toggle_flag = 0;

// // ----- 便利 ePWM 指针数组（便于循环访问） -----
// // ==== 修改/优化 ==== 把常用 ePWM 指针放在文件作用域，避免反复 switch 调用
// volatile struct EPWM_REGS *ePWM_all[6] = {&EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs};

// // 返回指定模块的寄存器指针（moduleIndex: 0..2） - 保留旧接口兼容
// static inline volatile struct EPWM_REGS *GetEPwmRegsByModule(Uint16 moduleIndex)
// {
//     if (moduleIndex < 6)
//         return ePWM_all[moduleIndex];
//     return &EPwm1Regs;
// }

// // 计算 TBPRD（上计数模式），公式 TBPRD = TBCLK / freq
// static Uint16 Calc_TBPRD_forFreq(Uint32 freqHz)
// {
//     if (freqHz < PWM_FREQ_MIN_HZ)
//         freqHz = PWM_FREQ_MIN_HZ;
//     if (freqHz > PWM_FREQ_MAX_HZ)
//         freqHz = PWM_FREQ_MAX_HZ;

//     Uint64 tbclk = (Uint64)SYSCLK_HZ;
//     Uint64 tbprd = tbclk / freqHz;
//     if (tbprd < 1)
//         tbprd = 1;
//     if (tbprd > 0xFFFF)
//         tbprd = 0xFFFF;
//     return (Uint16)tbprd;
// }

// static inline Uint16 GetCurrentTBPRD(void)
// {
//     return ePWM_all[0]->TBPRD;
// }

// // 禁用所有模块中断并清标志（安全）
// void DisableAllModuleInterrupts(void)
// {
//     Uint16 i;
//     for (i = 0; i < 6; i++)
//     {
//         ePWM_all[i]->ETSEL.bit.INTEN = 0;
//         ePWM_all[i]->ETCLR.bit.INT = 1;
//         ePWM_all[i]->ETPS.bit.INTPRD = ET_1ST;
//     }

//     PieCtrlRegs.PIEIER3.bit.INTx1 = 0;
//     PieCtrlRegs.PIEIER3.bit.INTx2 = 0;
//     PieCtrlRegs.PIEIER3.bit.INTx3 = 0;
//     PieCtrlRegs.PIEIER3.bit.INTx4 = 0;
//     PieCtrlRegs.PIEIER3.bit.INTx5 = 0;
//     PieCtrlRegs.PIEIER3.bit.INTx6 = 0;
// }

// // 使能指定模块的 CMPA/CMPB 中断 (moduleIndex 0..2, isA 1=CMPA,0=CMPB)
// void EnableModuleCmpInterrupt(Uint16 moduleIndex, Uint16 isA)
// {
//     volatile struct EPWM_REGS *E;

//     if (moduleIndex >= 6) return;

//     E = ePWM_all[moduleIndex];
//     E->ETCLR.bit.INT = 1;
//     E->ETSEL.bit.INTSEL = isA ? ET_CTRU_CMPA : ET_CTRU_CMPB;
//     E->ETSEL.bit.INTEN  = 0;

//     switch (moduleIndex)
//     {
//     case 0: PieCtrlRegs.PIEIER3.bit.INTx1 = 1; break;
//     case 1: PieCtrlRegs.PIEIER3.bit.INTx2 = 1; break;
//     case 2: PieCtrlRegs.PIEIER3.bit.INTx3 = 1; break;
//     case 3: PieCtrlRegs.PIEIER3.bit.INTx4 = 1; break;
//     case 4: PieCtrlRegs.PIEIER3.bit.INTx5 = 1; break;
//     case 5: PieCtrlRegs.PIEIER3.bit.INTx6 = 1; break;
//     default: break;
//     }
//     IER |= M_INT3;
//     E->ETSEL.bit.INTEN  = 1;
// }

// // 关闭指定模块的 CMPA/CMPB 中断 (moduleIndex 0..2, isA 1=CMPA,0=CMPB)
// static void DisableModuleCmpInterrupt(Uint16 moduleIndex)
// {
//     volatile struct EPWM_REGS *E;
    
//     if (moduleIndex >= MMC_NUMBER)
//         return; // only valid for 0..5

//     E = ePWM_all[moduleIndex];
//     E->ETSEL.bit.INTEN = 0;
//     E->ETCLR.bit.INT = 1;

//     switch (moduleIndex)
//     {
//     case 0: PieCtrlRegs.PIEIER3.bit.INTx1 = 0; break;
//     case 1: PieCtrlRegs.PIEIER3.bit.INTx2 = 0; break;
//     case 2: PieCtrlRegs.PIEIER3.bit.INTx3 = 0; break;
//     case 3: PieCtrlRegs.PIEIER3.bit.INTx4 = 0; break;
//     case 4: PieCtrlRegs.PIEIER3.bit.INTx5 = 0; break;
//     case 5: PieCtrlRegs.PIEIER3.bit.INTx6 = 0; break;
//     default: break;
//     }
// }

// /* 扫描某个 ePWM 范围内谁真的触发了中断 */
// static int16 FindPendingEpwmModule(Uint16 startModule, Uint16 endModule)
// {
//     Uint16 i;
//     for (i = startModule; i <= endModule; i++)
//     {
//         if (ePWM_all[i]->ETFLG.bit.INT == 1U)
//             return (int16)i;
//     }
//     return -1;
// }

// // TODO
// // 在 3 对 CMP 中找出全局最大比较点，返回 moduleIndex (0..2) 和 isA (1=CMPA,0=CMPB)
// // 使用上组 CMP_order（upper mapping）
// // static void FindMaxCmp(Uint16 *outModule, Uint16 *outIsA)
// // {
// //     // Uint32 maxV = 0;
// //     Uint16 bestModule = 0;
// //     Uint16 bestIsA = 1;
// //     for (Uint16 i = 0; i < PWM_PAIR_COUNT; ++i)
// //     {
// //         if (P_MMC_array[CMPA_IDX(i)].position_id == (MMC_NUMBER - 1))
// //         {
// //             bestModule = i;
// //             bestIsA = 1;
// //             break;
// //         }

// //         if (P_MMC_array[CMPB_IDX(i)].position_id == (MMC_NUMBER - 1))
// //         {
// //             bestModule = i;
// //             bestIsA = 0;
// //             break;
// //         }
// //     }

// //     *outModule = bestModule;
// //     *outIsA = bestIsA;
// // }

// static void FindMaxCmp(Uint16 *outModule, Uint16 *outIsA)
// {
//     Uint32 maxV = 0;
//     Uint16 bestModule = 0;
//     Uint16 bestIsA = 1;
//     for (Uint16 i = 0; i < PWM_PAIR_COUNT; ++i)
//     {
//         Uint16 idxA = P_MMC_array[CMPA_IDX(i)].position_id; // 0..5
//         Uint16 idxB = P_MMC_array[CMPB_IDX(i)].position_id; // 0..5

//         Uint32 valA = (Uint32)CMP_Values[idxA];
//         Uint32 valB = (Uint32)CMP_Values[idxB];
//         if (valA > maxV)
//         {
//             maxV = valA;
//             bestModule = i;
//             bestIsA = 1;
//         }
//         if (valB > maxV)
//         {
//             maxV = valB;
//             bestModule = i;
//             bestIsA = 0;
//         }
//     }

//     *outModule = bestModule;
//     *outIsA = bestIsA;
// }

// // 初始化函数
// static void Partial_ClearPrepared(volatile PartialPreparedCtx *ctx)
// {
//     ctx->valid = 0;
//     ctx->arm = 0;
//     ctx->localModule = 0;
//     ctx->globalModule = 0;
//     ctx->channelIsA = 1;
//     ctx->ownBaseCmp = 0;
//     ctx->firstCmp = 0;
//     ctx->secondCmp = 0;
// }

// // 初始化函数
// static void Partial_ClearActive(volatile PartialActiveCtx *ctx)
// {
//     ctx->active = 0;
//     ctx->arm = PARTIAL_ARM_P;
//     ctx->localModule = 0;
//     ctx->globalModule = 0;
//     ctx->channelIsA = 1;
//     ctx->secondCmpCurrent = 0;
//     ctx->savedBorrowCmpCurrent = 0;
//     ctx->savedPairedAction = 0;
//     ctx->deferredBorrowCmpNext = 0;
//     ctx->deferredBorrowCmpValid = 0;
//     ctx->deferredPairedActionNext = 0;
//     ctx->deferredPairedActionValid = 0;
// }

// static void PartialHold_Clear(volatile PartialHoldCtx *ctx)
// {
//     ctx->valid = 0;
//     ctx->globalModule = 0;
//     ctx->channelIsA = 1;
//     ctx->savedOwnAction = 0;
//     ctx->deferredOwnAction = 0;
//     ctx->deferredOwnActionValid = 0;
// }

// static void PartialHold_ArmFromRestore(volatile PartialActiveCtx *act)
// {
//     volatile struct EPWM_REGS *E;

//     if (!act->active)
//         return;

//     E = ePWM_all[act->globalModule];

//     PartialHold_Clear(&g_partialHold);

//     g_partialHold.valid = 1;
//     g_partialHold.globalModule = act->globalModule;
//     g_partialHold.channelIsA = act->channelIsA;

//     if (act->channelIsA)
//     {
//         /* Partial 在 A 通道：
//          * own action 是 AQCTLA.bit.CAU
//          */
//         g_partialHold.savedOwnAction = E->AQCTLA.bit.CAU;

//         /* 下一子周期先抑制 own action，保持高电平 */
//         E->AQCTLA.bit.CAU = AQ_NO_ACTION;
//     }
//     else
//     {
//         /* Partial 在 B 通道：
//          * own action 是 AQCTLB.bit.CBU
//          */
//         g_partialHold.savedOwnAction = E->AQCTLB.bit.CBU;

//         E->AQCTLB.bit.CBU = AQ_NO_ACTION;
//     }
// }

// // 已废弃
// // static Uint16 PartialHold_DeferOwnActionIfNeeded(Uint16 globalModule, Uint16 channelIsA, Uint16 action)
// // {
// //     if (!g_partialHold.valid)
// //         return 0;

// //     if ((g_partialHold.globalModule == globalModule) && (g_partialHold.channelIsA == channelIsA))
// //     {
// //         g_partialHold.deferredOwnAction = action;
// //         g_partialHold.deferredOwnActionValid = 1;
// //         return 1;
// //     }

// //     return 0;
// // }

// static void PartialHold_ReleaseAtMainIsr(void)
// {
//     volatile struct EPWM_REGS *E;
//     Uint16 restoreAction;

//     if (!g_partialHold.valid)
//         return;

//     E = ePWM_all[g_partialHold.globalModule];

//     restoreAction = g_partialHold.deferredOwnActionValid
//                   ? g_partialHold.deferredOwnAction
//                   : g_partialHold.savedOwnAction;

//     if (g_partialHold.channelIsA)
//     {
//         /* 恢复 A 通道的 own action */
//         E->AQCTLA.bit.CAU = restoreAction;
//     }
//     else
//     {
//         /* 恢复 B 通道的 own action */
//         E->AQCTLB.bit.CBU = restoreAction;
//     }

//     PartialHold_Clear(&g_partialHold);
// }

// // TODO 未考虑D处于边界时可能会在0.5和0之间来回切换，导致控制不稳定，后续需在实验时进行测试
// /*
//  * 拆解 Get_K_D_Method() 的返回值：
//  *   整数部分 = K
//  *   小数部分 / 2 = D （D 范围 0~0.5）
//  *
//  * 例如：
//  *   2.60 → K=2, D=0.30
//  *   3.00 → K=3, D=0.00（无 Partial）
//  */
// static void DecodeKDValue(float32 kdValue, Uint16 *kOut, float32 *dOut)
// {
//     int16 kInt;
//     float32 frac;

//     kInt = (int16)kdValue;
//     frac = kdValue - (float32)kInt;

//     if (frac < 0.0f) frac = 0.0f;
//     else if (frac > 0.99f) 
//     {
//         frac = 0;
//         kInt += 1;
//     }

//     if (kInt < 0)   kInt = 0;
//     if (kInt > MMC_NUMBER) kInt = MMC_NUMBER;

//     *kOut = (Uint16)kInt;
//     *dOut = 0.5f * frac;

//     if (*dOut < PARTIAL_D_EPS) *dOut = 0.0f;
//     // if (*dOut > 0.499f) *dOut = 0.499f;
// }

// /*
//  * 构造"下一子周期"的 Partial 计划。
//  *
//  * 参数：
//  *   armArray    : P_MMC_array 或 N_MMC_array
//  *   cmpValueArray : CMP_Values 或 CMP_addError_Values
//  *   moduleBase  : 0（P桥臂，ePWM1~3）或 3（N桥臂，ePWM4~6）
//  *   dutyD       : Partial 占空比 D（0~0.5）
//  *   ctx         : 输出的 Prepared 上下文
//  *
//  * 选取规则：
//  *   position_id == K_value 的子模块就是 Partial 模块。
//  *   这是因为 K 模块（position_id < K_value）全周期导通，
//  *   Partial 是紧接 K 之后的第一个子模块。
//  *
//  * compare 计算：
//  *   ownBaseCmp = cmpValueArray[K_value]   （Partial 模块原来的 compare）
//  *   firstCmp   = ownBaseCmp + T*D         （第一翻转延后量）
//  *   secondCmp  = ownBaseCmp + (1-D)*T     （第二翻转时刻）
//  *
//  * 返回 1 表示成功构造 Partial，0 表示退化为普通半开通。
//  */
// static Uint16 Partial_BuildPrepared(volatile mmc_info *armArray,
//                                     volatile Uint16 *cmpValueArray,
//                                     Uint16 moduleBase,
//                                     float32 dutyD,
//                                     Uint16 nextBoundaryCmp,
//                                     volatile PartialPreparedCtx *ctx)
// {
//     Uint16 smIndex = 0xFFFF;
//     Uint16 i;
//     Uint16 tbprd, ownBase, deltaCnt, firstCmp, secondCmp;
//     Uint32 secondCmpRaw, secondCmpLimit;

//     Partial_ClearPrepared(ctx);

//     /* 无 Partial 需求 */
//     if (dutyD <= PARTIAL_D_EPS)
//         return 0;

//     /* 全部已经 K 了，没有半插入模块，也不需要 Partial */
//     // if (K_value >= MMC_NUMBER)
//     //     return 0;

//     /* 找到 position_id == K_value 的物理子模块索引 */
//     for (i = 0; i < MMC_NUMBER; i++)
//     {
//         if (armArray[i].position_id == K_value)
//         {
//             smIndex = i;
//             break;
//         }
//     }

//     // if (smIndex >= MMC_NUMBER)
//     //     return 0;

//     ownBase = cmpValueArray[K_value];
//     tbprd = GetCurrentTBPRD() - ownBase + nextBoundaryCmp;

//     deltaCnt = (Uint16)(dutyD * (float32)tbprd + 0.5f);
//     firstCmp  = ownBase + deltaCnt;
//     secondCmpRaw = (Uint32)ownBase + (Uint32)((1.0f - dutyD) * (float32)tbprd + 0.5f);

//     // if (firstCmp >= tbprd)
//     //     firstCmp = tbprd - 2U;

//     /* 安全保护：两次翻转太近 */
//     if (secondCmpRaw <= ((Uint32)firstCmp + PARTIAL_MIN_GAP_CNT))
//         return 0;

//     /* 安全保护：second edge 太晚，窗口不够 restore */
//     if (nextBoundaryCmp <= PARTIAL_RESTORE_GUARD_CNT)
//         return 0;

//     secondCmpLimit = (Uint32)tbprd + (Uint32)nextBoundaryCmp - (Uint32)PARTIAL_RESTORE_GUARD_CNT;

//     if (secondCmpRaw >= secondCmpLimit)
//         return 0;

//     if (secondCmpRaw >= (Uint32)tbprd - nextBoundaryCmp)
//     {
//         secondCmp = (Uint16)(secondCmpRaw - (Uint32)tbprd + (Uint32)nextBoundaryCmp);
//         if (secondCmp == 0U)
//             secondCmp = 1U;   /* 避免等于0 */
//     }
//     else
//     {
//         secondCmp = (Uint16)secondCmpRaw;
//     }

//     ctx->valid = 1;
//     ctx->arm = (moduleBase == 0) ? PARTIAL_ARM_P : PARTIAL_ARM_N;  // 判断属于哪个桥臂
//     ctx->localModule  = (smIndex >> 1);
//     ctx->globalModule = moduleBase + ctx->localModule;  // 判断属于ePWMx
//     ctx->channelIsA   = ((smIndex & 0x01U) == 0U) ? 1U : 0U;  // 根据奇偶判断需要Partial的是A通道还是B通道
//     ctx->ownBaseCmp   = ownBase;  // 该通道本来的半开通 compare值
//     ctx->firstCmp     = firstCmp;  // partial开通的第一次翻转时刻
//     ctx->secondCmp    = secondCmp;  // partial开通的第二次翻转时刻

//     return 1;
// }

// /*
//  * 检查某个 ePWM 模块的 borrowed compare 是否正被 Active Partial 占用。
//  * 如果是，返回 Active 上下文指针；否则返回 NULL。
//  */
// static volatile PartialActiveCtx *Partial_GetActiveByModule(Uint16 globalModule)
// {
//     if (g_partialActive.active && (g_partialActive.globalModule == globalModule))
//         return &g_partialActive;

//     return (volatile PartialActiveCtx *)0;
// }

// /*
//  * 写 CMPA：如果这个模块的 CMPA 正被 Active Partial 借用，则 defer。
//  *
//  * "被借用" 的判断：
//  *   Active Partial 在 B 通道时，借用的是 CMPA → 此时不能直接写 CMPA。
//  */
// static void Partial_WriteCmpAOrDefer(Uint16 globalModule, Uint16 cmpA)
// {
//     volatile PartialActiveCtx *act = Partial_GetActiveByModule(globalModule);

//     if (act && act->active && (act->channelIsA == 0U))
//     {
//         /* Partial 在 B 通道，CMPA 被借用，不能直接写 */
//         act->deferredBorrowCmpNext = cmpA;
//         act->deferredBorrowCmpValid = 1;
//     }
//     else
//     {
//         ePWM_all[globalModule]->CMPA.half.CMPA = cmpA;
//     }
// }

// /*
//  * 写 CMPB：如果这个模块的 CMPB 正被 Active Partial 借用，则 defer。
//  *
//  * "被借用" 的判断：
//  *   Active Partial 在 A 通道时，借用的是 CMPB → 此时不能直接写 CMPB。
//  */
// static void Partial_WriteCmpBOrDefer(Uint16 globalModule, Uint16 cmpB)
// {
//     volatile PartialActiveCtx *act = Partial_GetActiveByModule(globalModule);

//     if (act && act->active && (act->channelIsA == 1U))
//     {
//         /* Partial 在 A 通道，CMPB 被借用，不能直接写 */
//         act->deferredBorrowCmpNext = cmpB;
//         act->deferredBorrowCmpValid = 1;
//     }
//     else
//     {
//         ePWM_all[globalModule]->CMPB = cmpB;
//     }
// }

// // 已废弃
// /*
//  * 写 AQ 动作：如果目标 AQ 位正被 Active Partial 锁住，则 defer。
//  *
//  * "被锁住" 的判断：
//  *   Active Partial 在 A 通道 → 借用 CMPB → 锁住了 B 输出的 CBU
//  *   Active Partial 在 B 通道 → 借用 CMPA → 锁住了 A 输出的 CAU
//  */
// // static void Partial_WritePairedActionOrDefer(Uint16 globalModule, Uint16 targetChannelIsA, Uint16 action)
// // {
// //     volatile PartialActiveCtx *act = Partial_GetActiveByModule(globalModule);

// //     if (act && act->active)
// //     {
// //         /*
// //          * 判断是否正好命中被锁住的 AQ 位：
// //          * - Partial 在 A → 锁的是 B 的 CBU → targetChannelIsA==0 时命中
// //          * - Partial 在 B → 锁的是 A 的 CAU → targetChannelIsA==1 时命中
// //          */
// //         if ((act->channelIsA && (targetChannelIsA == 0U)) ||
// //             ((!act->channelIsA) && (targetChannelIsA == 1U)))
// //         {
// //             act->deferredPairedActionNext = action;
// //             act->deferredPairedActionValid = 1;
// //             return;
// //         }
// //     }

// //     /* 没有冲突，直接写硬件 */
// //     if (targetChannelIsA)
// //         ePWM_all[globalModule]->AQCTLA.bit.CAU = action;
// //     else
// //         ePWM_all[globalModule]->AQCTLB.bit.CBU = action;
// // }

// /*
//  * 把某桥臂下一子周期的 compare 写入硬件。
//  *
//  * 对于 Prepared Partial 模块：
//  *   - Partial 自己那个通道的 compare 写成 firstCmp（替代原 ownBaseCmp）
//  *   - 同模块另一通道不变
//  *
//  * 对于 Active Partial 正占用的 borrowed compare：
//  *   - 不直接写硬件，defer 到 restore ISR
//  */
// static void Partial_WriteArmCmpForNextSubcycle(volatile mmc_info *armArray,
//                                                volatile Uint16 *cmpValueArray,
//                                                Uint16 moduleBase,
//                                                volatile PartialPreparedCtx *prepCtx)
// {
//     Uint16 i;

//     for (i = 0; i < 3; ++i)
//     {
//         Uint16 idxA = armArray[CMPA_IDX(i)].position_id;
//         Uint16 idxB = armArray[CMPB_IDX(i)].position_id;
//         Uint16 cmpA = cmpValueArray[idxA];
//         Uint16 cmpB = cmpValueArray[idxB];
//         Uint16 globalModule = moduleBase + i;

//         /*
//          * 如果这个模块正好是 Partial 所在模块，
//          * 则把 Partial 自己那个通道的 compare 改成 firstCmp。
//          */
//         if (prepCtx->valid && (prepCtx->localModule == i))
//         {
//             if (prepCtx->channelIsA)
//                 cmpA = prepCtx->firstCmp;
//             else
//                 cmpB = prepCtx->firstCmp;
//         }

//         Partial_WriteCmpAOrDefer(globalModule, cmpA);
//         Partial_WriteCmpBOrDefer(globalModule, cmpB);
//     }
// }

// /*
//  * 把上下桥臂 12 路 compare 全部写入下一子周期。
//  *
//  * 这个函数体现了你的修正：
//  * 每个子周期都必须重写全部 12 路 compare，
//  * 因为 CMP_addError_Values 会加入桥臂间误差补偿。
//  */
// static void Partial_WriteAllCmpForNextSubcycle(volatile Uint16 *cmpValueArray, volatile PartialPreparedCtx *prepCtx)
// {
//     /*
//      * 根据 Partial 所在桥臂决定写法：
//      *
//      * 如果 Partial 在 P 桥臂（moduleBase=0）：
//      *   P 桥臂写入时考虑 Partial firstCmp 替换
//      *   N 桥臂正常写入
//      *
//      * 如果 Partial 在 N 桥臂（moduleBase=3）：
//      *   P 桥臂正常写入
//      *   N 桥臂写入时考虑 Partial firstCmp 替换
//      *
//      * 如果没有 Partial：
//      *   全部正常写入
//      */

//     PartialPreparedCtx emptyPrep = {0}; /* 无 Partial 的空计划 */

//     if (prepCtx->valid && (prepCtx->arm == PARTIAL_ARM_P))
//     {
//         /* P 桥臂下周期有 Partial，同时还要对本周期N桥臂Partial被占用的通道做特殊处理 */
//         Partial_WriteArmCmpForNextSubcycle(P_MMC_array, cmpValueArray, 0, prepCtx);
//         Partial_WriteArmCmpForNextSubcycle(N_MMC_array, cmpValueArray, 3, &emptyPrep);
//     }
//     else if (prepCtx->valid && (prepCtx->arm == PARTIAL_ARM_N))
//     {
//         /* N 桥臂下周期有 Partial，同时还要对本周期P桥臂Partial被占用的通道做特殊处理 */
//         Partial_WriteArmCmpForNextSubcycle(P_MMC_array, cmpValueArray, 0, &emptyPrep);
//         Partial_WriteArmCmpForNextSubcycle(N_MMC_array, cmpValueArray, 3, prepCtx);
//     }
//     else
//     {
//         /* 无 Partial，全部正常写入 */
//         Partial_WriteArmCmpForNextSubcycle(P_MMC_array, cmpValueArray, 0, &emptyPrep);
//         Partial_WriteArmCmpForNextSubcycle(N_MMC_array, cmpValueArray, 3, &emptyPrep);
//     }
// }

// /*
//  * 注入 second edge：
//  *
//  * 执行时机：EPWM_GlobalISR() 中（上下桥臂所有原始 compare 都已完成后）。
//  *
//  * 操作：
//  * 1. 借用同模块另一 compare 寄存器
//  * 2. 设为 secondCmp 值
//  * 3. 切为 Immediate 模式使其立即生效
//  * 4. 让 Partial 自己的输出在 borrowed compare 处翻转
//  * 5. 屏蔽 paired output 对 borrowed compare 的响应
//  * 6. 给 borrowed compare 注册一个中断（用于 restore）
//  */
// static void Partial_InjectSecondEdge(volatile PartialPreparedCtx *prep, volatile PartialActiveCtx *act)
// {
//     volatile struct EPWM_REGS *E;

//     if (!prep->valid)
//         return;

//     /* 理论上 Active 不应同时存在（P/N 交替出现），但加保护 */
//     if (act->active)
//         return;

//     E = ePWM_all[prep->globalModule];

//     /* 填充 Active 上下文 */
//     Partial_ClearActive(act);
//     act->active = 1;
//     act->arm = prep->arm;
//     act->localModule = prep->localModule;
//     act->globalModule = prep->globalModule;
//     act->channelIsA = prep->channelIsA;
//     act->secondCmpCurrent = prep->secondCmp;

//     if (prep->channelIsA)
//     {
//         /*
//          * Partial 在 A 通道，借用 CMPB 来制造 A 的第二翻转。
//          *
//          * 步骤：
//          * 1) 保存当前 CMPB 值和 B 输出对 CBU 的 AQ 动作
//          * 2) 把 CMPB 切为 Immediate 模式，写入 secondCmp
//          * 3) 设置 A 输出在 CMPB 处翻转：AQCTLA.CBU = TOGGLE
//          * 4) 屏蔽 B 输出对 CMPB 的响应：AQCTLB.CBU = NO_ACTION
//          * 5) 使能 CMPB 中断，用于 second edge 发生后恢复
//          */
//         act->savedBorrowCmpCurrent = E->CMPB;
//         act->savedPairedAction = E->AQCTLB.bit.CBU;

//         E->CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;
        
//         E->AQCTLB.bit.CBU = AQ_NO_ACTION;
//         E->AQCTLA.bit.CBU = AQ_TOGGLE;
        
//         E->CMPB = prep->secondCmp;
//         /*
//          * 为 second edge 注册恢复中断。
//          *
//          * 对于 P 桥臂（ePWM1~3）：恢复中断进入 EPWM_GlobalISR()
//          * 对于 N 桥臂（ePWM4~6）：恢复中断进入 EPWM_LowerISR()
//          */
//         EnableModuleCmpInterrupt(prep->globalModule, 0); /* 0 -> CMPB */
//     }
//     else
//     {
//         /*
//          * Partial 在 B 通道，借用 CMPA 来制造 B 的第二翻转。
//          */
//         act->savedBorrowCmpCurrent = E->CMPA.half.CMPA;
//         act->savedPairedAction = E->AQCTLA.bit.CAU;

//         E->CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
        
//         E->AQCTLA.bit.CAU = AQ_NO_ACTION;
//         E->AQCTLB.bit.CAU = AQ_TOGGLE;
        
//         E->CMPA.half.CMPA = prep->secondCmp;
//         EnableModuleCmpInterrupt(prep->globalModule, 1); /* 1 -> CMPA */
//     }

//     /* 这份计划已被当前子周期消费 */
//     prep->valid = 0;
// }

// /*
//  * 恢复 Active Partial：
//  *
//  * 执行时机：second edge 到达后，在恢复 ISR 中调用。
//  *
//  * 操作：
//  * 1. 取消 Partial 自己输出对 borrowed compare 的 AQ 翻转
//  * 2. 恢复 paired output 的 AQ 行为（使用 deferred 值或原始保存值）
//  * 3. 把 borrowed compare 切回 Shadow 模式
//  * 4. 写入下一子周期的 compare 值（使用 deferred 值或原始保存值）
//  * 5. 关闭恢复中断
//  * 6. 清空 Active 上下文
//  */
// static void Partial_RestoreActiveCtx(volatile PartialActiveCtx *act)
// {
//     volatile struct EPWM_REGS *E;
//     Uint16 restoreCmp;
//     Uint16 restoreAction;

//     if (!act->active)
//         return;

//     E = ePWM_all[act->globalModule];

//     /*
//      * 确定恢复值：
//      * 如果 BridgeArmSort 在 Active 期间写了 deferred 值，用 deferred；
//      * 否则用注入时保存的原值。
//      */
//     restoreCmp = act->deferredBorrowCmpValid
//                      ? act->deferredBorrowCmpNext
//                      : act->savedBorrowCmpCurrent;

//     restoreAction = act->deferredPairedActionValid
//                         ? act->deferredPairedActionNext
//                         : act->savedPairedAction;

//     if (act->channelIsA)
//     {
//         /* Partial 在 A 通道，借的是 CMPB */
//         E->AQCTLA.bit.CBU = AQ_NO_ACTION;  /* A 输出不再在 CMPB 处翻转 */
//         // TODO 这里需要考虑两个子周期过去后下一周期是全开通还是半开通的情况是否合理
//         E->AQCTLB.bit.CBU = restoreAction; /* 恢复 B 输出未来 AQ 动作 */

//         /* 关键顺序：
//          * 1. 仍处于 Immediate 模式时，先恢复 active CMPB
//          * 2. 再切回 Shadow 模式
//          * 3. 再写一次 restoreCmp 给 Shadow
//          */
//         E->CMPB = restoreCmp;
//         E->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
//         E->CMPB = restoreCmp;

//         DisableModuleCmpInterrupt(act->globalModule);
//     }
//     else
//     {
//         /* Partial 在 B 通道，借的是 CMPA */
//         E->AQCTLB.bit.CAU = AQ_NO_ACTION;  /* B 输出不再在 CMPA 处翻转 */
//         E->AQCTLA.bit.CAU = restoreAction; /* 恢复 A 输出未来 AQ 动作 */

//         E->CMPA.half.CMPA = restoreCmp;
//         E->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
//         E->CMPA.half.CMPA = restoreCmp;

//         DisableModuleCmpInterrupt(act->globalModule);
//     }

//     PartialHold_ArmFromRestore(act);

//     Partial_ClearActive(act);
// }

// void EPWM_ResetControlState(void)
// {
//     fre_PID.prev_error = 0.0f;
//     fre_PID.prev2_error = 0.0f;
//     fre_PID.output = INIT_FREQUENCE;

//     err_PID.prev_error = 0.0f;
//     err_PID.prev2_error = 0.0f;
//     err_PID.output = INIT_ERROR;

//     arm_per_voltage_P = 0.0f;
//     arm_per_voltage_N = 0.0f;
// }

// /*
//  * 主中断挂接管理：
//  *
//  * 为什么需要 Queue 机制？
//  * 因为下一次主中断想用的 ePWM 模块可能正好是当前 Active Partial 的模块。
//  * 如果直接挂接，会与 Partial 的 restore 中断冲突。
//  * 所以先 Queue 起来，等 Active Partial restore 完成后再真正挂接。
//  */
// static void TryArmUpperMainInterrupt(void)
// {
//     // TODO 直接注释这两个判断
//     if (g_upperNextPending == 0U)
//         return;

//     // /* 如果 Active Partial 正在占用目标模块，延后 */
//     if (g_partialActive.active && ((Uint16)g_partialActive.globalModule == (Uint16)g_upperNextModule))
//         return;

//     EnableModuleCmpInterrupt((Uint16)g_upperNextModule, (Uint16)g_upperNextIsA);
//     g_upperCurrModule = g_upperNextModule;
//     g_upperCurrIsA    = g_upperNextIsA;
//     g_upperNextPending = 0U;
// }

// static void QueueUpperMainInterrupt(Uint16 moduleIndex, Uint16 isA)
// {
//     g_upperNextModule  = (int16)moduleIndex;
//     g_upperNextIsA     = (int16)isA;
//     g_upperNextPending = 1U;
//     // TODO
//     EnableModuleCmpInterrupt(g_upperNextModule, g_upperNextIsA);
//     // TryArmUpperMainInterrupt();
// }



// // ---------------- 统一的闭环ePWM初始化函数 ----------------
// void EPWM_SwitchToClosedLoop(Uint16 fromPrecharge)
// {
//     EALLOW;
//     // 先禁止 TBCLK 同步以便批量配置
//     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
//     EDIS;

//     // 冷启动时需要初始化GPIO为ePWM功能
//     if (!fromPrecharge)
//     {
//         InitEPwm1Gpio();
//         InitEPwm2Gpio();
//         InitEPwm3Gpio();
//         InitEPwm4Gpio();
//         InitEPwm5Gpio();
//         InitEPwm6Gpio();
//     }

//     EALLOW;
//     // 计算 TBPRD 并写入所有模块
//     Uint16 tbprd = Calc_TBPRD_forFreq(INIT_FREQUENCE);

//     for (Uint16 i = 0; i < 6; ++i)
//     {
//         volatile struct EPWM_REGS *E = ePWM_all[i];

//         // 清除所有配置
//         E->TBCTL.all = 0;
//         E->AQCTLA.all = 0;
//         E->AQCTLB.all = 0;
//         E->AQSFRC.all = 0;
//         E->AQCSFRC.all = 0;

//         // TBCTL
//         // E->TBCTL.all = 0;
//         E->TBCTL.bit.PHSEN = (i == 0) ? TB_DISABLE : TB_ENABLE; // ePWM1为主
//         E->TBCTL.bit.PRDLD = TB_SHADOW;
//         E->TBCTL.bit.HSPCLKDIV = TB_DIV1; // HSPCLKDIV = 1
//         E->TBCTL.bit.CLKDIV = TB_DIV1;    // CLKDIV = 1
//         E->TBPRD = tbprd;
//         E->TBPHS.half.TBPHS = 0;
//         E->TBCTR = 0;
//         E->TBCTL.bit.CTRMODE = TB_COUNT_UP;

//         // 同步选择： ePWM1 发 SYNCOUT，其他为 SYNC_IN
//         E->TBCTL.bit.SYNCOSEL = (i == 0) ? TB_CTR_ZERO : TB_SYNC_IN;

//         // 关闭影子寄存器，修改CMP的值直接生效（也可以选择在 CTR==0 时载入，见后面设置）
//         // E->CMPCTL.bit.SHDWAMODE = CC_IMMEDIATE;
//         // E->CMPCTL.bit.SHDWBMODE = CC_IMMEDIATE;

//         // CMP 影子寄存器设置（在 CTR==0 时载入）
//         E->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
//         E->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
//         E->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
//         E->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

//         // 写入初始 CMP 影子值（通过 CMP_order 映射）
//         Uint16 idxA = (i < 3) ? P_MMC_array[CMPA_IDX(i)].position_id : N_MMC_array[CMPA_IDX(i - 3)].position_id;
//         Uint16 idxB = (i < 3) ? P_MMC_array[CMPB_IDX(i)].position_id : N_MMC_array[CMPB_IDX(i - 3)].position_id;
//         E->CMPA.half.CMPA = CMP_Values[idxA];
//         E->CMPB = CMP_Values[idxB];

//         // 使用AQCSFRC设置初始电平
//         if (i < 3)
//         {
//             // 上桥臂
//             // Uint16 smA = i * 2;
//             // Uint16 smB = i * 2 + 1;

//             if (P_MMC_array[CMPA_IDX(i)].position_id < K_value)
//             {
//                 // K模块：强制高，保持高
//                 E->AQCSFRC.bit.CSFA = 2;
//                 E->AQCTLA.bit.CAU = AQ_SET;
//             }
//             else
//             {
//                 // 非K模块：强制低，电平清零
//                 E->AQCSFRC.bit.CSFA = 1;
//                 E->AQCTLA.bit.CAU = AQ_CLEAR;
//             }

//             if (P_MMC_array[CMPB_IDX(i)].position_id < K_value)
//             {
//                 E->AQCSFRC.bit.CSFB = 2;
//                 E->AQCTLB.bit.CBU = AQ_SET;
//             }
//             else
//             {
//                 E->AQCSFRC.bit.CSFB = 1;
//                 E->AQCTLB.bit.CBU = AQ_CLEAR;
//             }
//         }
//         else
//         {
//             // 下桥臂
//             Uint16 lowerIdx = i - 3;
//             // Uint16 smA = lowerIdx * 2;
//             // Uint16 smB = lowerIdx * 2 + 1;

//             if (N_MMC_array[CMPA_IDX(lowerIdx)].position_id < K_value)
//             {
//                 // K模块：强制高，保持高
//                 E->AQCSFRC.bit.CSFA = 2;
//                 E->AQCTLA.bit.CAU = AQ_SET;
//             }
//             else
//             {
//                 // 非K模块：强制高，正常翻转
//                 E->AQCSFRC.bit.CSFA = 2;
//                 E->AQCTLA.bit.CAU = AQ_SET;
//             }

//             if (N_MMC_array[CMPB_IDX(lowerIdx)].position_id < K_value)
//             {
//                 E->AQCSFRC.bit.CSFB = 2;
//                 E->AQCTLB.bit.CBU = AQ_SET;
//             }
//             else
//             {
//                 E->AQCSFRC.bit.CSFB = 2;
//                 E->AQCTLB.bit.CBU = AQ_SET;
//             }
//         }

//         // ==== 修改/优化 ==== 设为每次事件都触发中断（ET_1ST），注释已修正
//         E->ETPS.bit.INTPRD = ET_1ST; // 每次匹配事件都会触发中断（ET_1ST）
//         // 清除中断标志并禁止中断（后面会按最大值选举启用）
//         E->ETCLR.bit.INT = 1;
//         E->ETSEL.all = 0;
//     }

//     // 把 ePWM 的 PIE 向量映射到同一个全局 ISR（上组 ePWM1/2/3 的向量）
//     PieVectTable.EPWM1_INT = &EPWM_GlobalISR;
//     PieVectTable.EPWM2_INT = &EPWM_GlobalISR;
//     PieVectTable.EPWM3_INT = &EPWM_GlobalISR;
//     PieVectTable.EPWM4_INT = &EPWM_LowerISR;
//     PieVectTable.EPWM5_INT = &EPWM_LowerISR;
//     PieVectTable.EPWM6_INT = &EPWM_LowerISR;

//     // 如果从预充电切换，先切换MUX
//     if (fromPrecharge)
//     {
//         GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
//         GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
//         GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
//         GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
//         GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
//         GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;
//         GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
//         GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;
//         GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;
//         GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;
//         GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;
//         GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;
//     }

//     // 使能 PIE 与 CPU 中断（ePWM 在 PIE group3）
//     PieCtrlRegs.PIEIER3.bit.INTx1 = 1; // ePWM1
//     PieCtrlRegs.PIEIER3.bit.INTx2 = 1; // ePWM2
//     PieCtrlRegs.PIEIER3.bit.INTx3 = 1; // ePWM3  
//     PieCtrlRegs.PIEIER3.bit.INTx4 = 1; // ePWM4 (N 桥臂 Partial restore)
//     PieCtrlRegs.PIEIER3.bit.INTx5 = 1; // ePWM5
//     PieCtrlRegs.PIEIER3.bit.INTx6 = 1; // ePWM6

//     // 第一次选举最大比较点并启用其中断（安全切换）
//     {
//         Uint16 m, isA;
//         DisableAllModuleInterrupts();

//         // TODO

//         // Partial_ClearPrepared(&g_partialPrepared);
//         // Partial_ClearActive(&g_partialActive);

//         // g_upperCurrModule  = -1;
//         // g_upperCurrIsA     = -1;
//         // g_upperNextModule  = -1;
//         // g_upperNextIsA     = -1;
//         // g_upperNextPending = 0U;

//         FindMaxCmp(&m, &isA);
//         EnableModuleCmpInterrupt(m, isA);

//         g_upperCurrModule = (int16)m;
//         g_upperCurrIsA    = (int16)isA;
//         // TODO 这两个在这里被注释掉了
//         // g_prevModule = (int16)m;
//         // g_prevIsA = (int16)isA;
//     }

//     // 恢复 TBCLK 同步，这样所有 ePWM 一起开始计数
//     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;

//     EDIS;

//     IER |= M_INT3;

//     printf("[ePWM] Init complete (fromPrecharge=%d, K=%d)\r\n", fromPrecharge, K_value);
// }

// // PID增量计算函数
// void Frequence_PID_Incremental_Calculate(volatile PID *pid, float setpoint, float feedback)
// {
//     // 当前误差
//     float error = feedback - setpoint;

//     // 增量计算公式： Δu = kp * (e[n] - e[n-1]) + ki * e[n] + kd * (e[n] - 2*e[n-1] + e[n-2])
//     float delta_output = pid->kp * (error - pid->prev_error) + pid->ki * error + pid->kd * (error - 2 * pid->prev_error + pid->prev2_error);

//     // 更新输出值
//     pid->output += delta_output;
//     // 限幅
//     if (pid->output > FREQ_MAX)
//     {
//         pid->output = FREQ_MAX;
//     }
//     if (pid->output < FREQ_MIN)
//     {
//         pid->output = FREQ_MIN;
//     }

//     // 更新历史误差
//     pid->prev2_error = pid->prev_error;
//     pid->prev_error = error;
// }

// void Error_PID_Incremental_Calculate(volatile PID *pid, float setpoint, float feedback)
// {
//     // 当前误差
//     float error = feedback - setpoint;

//     // 增量计算公式： Δu = kp * (e[n] - e[n-1]) + ki * e[n] + kd * (e[n] - 2*e[n-1] + e[n-2])
//     float delta_output = pid->kp * (error - pid->prev_error) + pid->ki * error + pid->kd * (error - 2 * pid->prev_error + pid->prev2_error);

//     // 更新输出值
//     pid->output += delta_output;
//     // 限幅
//     if (pid->output > ERROR_MAX)
//     {
//         pid->output = ERROR_MAX;
//     }
//     if (pid->output < ERROR_MIN)
//     {
//         pid->output = ERROR_MIN;
//     }

//     // 更新历史误差
//     pid->prev2_error = pid->prev_error;
//     pid->prev_error = error;
// }

// float Duty_PID_Incremental_Calculate(volatile PID *pid, float setpoint, float feedback)
// {
//     // 当前误差
//     float delta_output;
//     float error = feedback - setpoint;

//     if(fabsf(error) > 80.0)
//     {
//         delta_output = pid->kp * (error - pid->prev_error) + pid->ki * error + pid->kd * (error - 2 * pid->prev_error + pid->prev2_error);
//     }
//     else
//     {
//         delta_output = pid->kp * (error - pid->prev_error) + pid->kd * (error - 2 * pid->prev_error + pid->prev2_error);
//     }

//     // 更新输出值
//     pid->output += delta_output;
//     // 限幅
//     if (pid->output > DUTY_MAX)
//     {
//         pid->output = DUTY_MAX;
//     }
//     if (pid->output < DUTY_MIN)
//     {
//         pid->output = DUTY_MIN;
//     }

//     // 更新历史误差
//     pid->prev2_error = pid->prev_error;
//     pid->prev_error = error;

//     // 返回最新输出值
//     return pid->output;
// }

// // 主动触发一次写入硬件（将 CMP... 写进影子寄存器 / TBPRD）
// void EPWM_FrequencyUpdate(void)
// {
//     // U_load = AD7606_2_Buf[6] * 5 / 32768.0 * 100.8; // 100.8 = 2016kΩ / 20kΩ
//     U_load = AD7606_2_Buf[6] / 32768.0 * 504;

//     // TODO 调试期间暂时不进行闭环控制，后续会进行是否进入循环的电平翻转检测
//     // Frequence_PID_Incremental_Calculate(&fre_PID, TARGET_VOLTAGE, U_load);

//     // Uint16 tbprd = Calc_TBPRD_forFreq((Uint32)g_targetFreqHz);
//     Uint16 tbprd = Calc_TBPRD_forFreq((Uint32)fre_PID.output);

//     for (Uint16 i = 0; i < 6; ++i)
//     {
//         ePWM_all[i]->TBPRD = tbprd;
//     }
// }

// float32 Get_K_D_Method(double U_in)
// {
//     static Uint16 updata_duty_flag = 0;
//     float32 K_ff = Get_K_ff_Method(U_in);
//     if (++updata_duty_flag >= 10)
//     {
//         updata_duty_flag = 0;
//         // TODO 调试期间暂时不进行闭环控制，后续会进行是否进入循环的电平翻转检测
//         // Duty_PID_Incremental_Calculate(&duty_PID, 10000, U_in);
//     }
//     return (K_ff + duty_PID.output);
// }

// float get_average_recent_voltage(volatile mmc_info *mmc_array)
// {
//     int16 min = mmc_array->recent_voltage;
//     int16 max = mmc_array->recent_voltage;

//     for (int i = 1; i < MMC_NUMBER; ++i)
//     {
//         int16 val = mmc_array[i].recent_voltage;
//         if (val < min)
//             min = val;
//         if (val > max)
//             max = val;
//     }

//     return (min + max) / 2.0f;
// }

// void change_CMP_Values_BasedOnError(volatile Uint16 *CMP_addError_Values)
// {
//     int32 error = (int32)err_PID.output;
//     for (Uint16 i = 0; i < MMC_NUMBER; i++)
//     {
//         CMP_addError_Values[i] = CMP_Values[i] + error;
//     }
// }

// void EPWM_ErrorUpdate(ArmUpdateStatus status)
// {
//     switch (status)
//     {
//     case Update_P_K_Arm:
//         arm_per_voltage_P = get_average_recent_voltage(P_MMC_array);
//         // TODO 调试期间暂时不进行闭环控制，后续会进行是否进入循环的电平翻转检测
//         // Error_PID_Incremental_Calculate(&err_PID, TARGET_ERROR, arm_per_voltage_P - arm_per_voltage_N);
//         // change_CMP_Values_BasedOnError(CMP_addError_Values);
//         break;

//     case Update_N_K_Arm:
//         arm_per_voltage_N = get_average_recent_voltage(N_MMC_array);
//         break;

//     default:
//         break;
//     }
// }

// // interrupt void EPWM_GlobalISR(void)
// // {
// //     // SOFT_PWM2_TOGGLE;
// //     // 清触发模块的中断标志（若存在）
// //     int16 srcModule;
// //     srcModule = FindPendingEpwmModule(0, 2);
// //     if (srcModule < 0)
// //     {
// //         PieCtrlRegs.PIEACK.all = EPWM_PIE_GROUP_ACK;
// //         return;
// //     }
// //     ePWM_all[srcModule]->ETCLR.bit.INT = 1;

// //     /* ====================================================================
// //      * 角色 2 判断：是否为 P 桥臂 Active Partial 的 restore 中断？
// //      *
// //      * 判断条件：
// //      * - Active Partial 正在运行
// //      * - Active Partial 在 P 桥臂（ePWM1~3）
// //      * - 当前触发模块就是 Active Partial 所在模块
// //      * ==================================================================== */
// //     if (g_partialActive.active &&
// //         (g_partialActive.arm == PARTIAL_ARM_P) &&
// //         ((Uint16)srcModule == g_partialActive.globalModule))
// //     {
// //         /* second edge 已发生，恢复 borrowed compare */
// //         Partial_RestoreActiveCtx(&g_partialActive);

// //         /*
// //          * 如果下一次主中断挂接因模块冲突而延后，现在补挂。
// //          * 场景：
// //          *   BridgeArmSort 计算出的下一次主中断和 Partial 恰好在同一模块，
// //          *   之前被 defer 了，现在 Partial 已结束，可以挂上了。
// //          */
// //         TryArmUpperMainInterrupt();

// //         PieCtrlRegs.PIEACK.all = EPWM_PIE_GROUP_ACK;
// //         return;
// //     }

// //     /* 关闭当前主中断源（下一次由 BridgeArmSort 重新选举挂接） */  
// //     if(g_firstIsrDone)
// //         DisableModuleCmpInterrupt((Uint16)srcModule);

// //     // ======================= 第一次中断处理 =======================
// //     if (!g_firstIsrDone)
// //     {
// //         EALLOW;
// //         for (Uint16 i = 0; i < 6; ++i)
// //         {
// //             volatile struct EPWM_REGS *E = ePWM_all[i];
// //             E->AQCSFRC.bit.CSFA = 0; // 禁用软件强制
// //             E->AQCSFRC.bit.CSFB = 0;
// //         }
// //         // 此时已经过了第一个CMP点，电平已经确认，后续动作时开始正常翻转
// //         // TODO 需要对第一次进入后的下桥臂的Partial模块进行特殊处理，这里先不完成，等待我完成对预充电阶段的配置后考虑
// //         for (Uint16 i = 0; i < 6; ++i)
// //         {
// //             volatile struct EPWM_REGS *E = ePWM_all[i];
// //             // 上桥臂
// //             if (i < 3)
// //             {
// //                 E->AQCTLA.bit.CAU = (P_MMC_array[CMPA_IDX(i)].position_id < K_value)
// //                                         ? AQ_SET : AQ_TOGGLE;
// //                 E->AQCTLB.bit.CBU = (P_MMC_array[CMPB_IDX(i)].position_id < K_value)
// //                                         ? AQ_SET : AQ_TOGGLE;
// //             }
// //             // 下桥臂
// //             else
// //             {
// //                 Uint16 li = i - 3;
// //                 E->AQCTLA.bit.CAU = (N_MMC_array[CMPA_IDX(li)].position_id < K_value)
// //                                         ? AQ_SET : AQ_TOGGLE;
// //                 E->AQCTLB.bit.CBU = (N_MMC_array[CMPB_IDX(li)].position_id < K_value)
// //                                         ? AQ_SET : AQ_TOGGLE;
// //             }
// //         }
// //         EDIS;

// //         g_firstIsrDone = 1;
// //     }

// //     /* 释放上一拍 Partial 留下的“下一子周期保持高”状态 */
// //     PartialHold_ReleaseAtMainIsr();

// //     /* -------- 注入 Partial second edge --------
// //      *
// //      * 此时上下桥臂的所有原始 half-insert compare 都已完成。
// //      * 如果 Prepared 上下文有效，说明本子周期有一个桥臂需要 Partial。
// //      *
// //      * 由于 P/N 交替出现（偶数子周期P，奇数子周期N），
// //      * 这里的 g_partialPrepared 要么是 P 桥臂的，要么是 N 桥臂的。
// //      */
// //     if (g_partialPrepared.valid)
// //     {
// //         Partial_InjectSecondEdge(&g_partialPrepared, &g_partialActive);
// //     }

// //     if (g_ADCIsrFlag)
// //     {
// //         g_isrToggleCnt++;
// //         // 触发后延迟2us再启动,等待开关管完全关断
// //         StartAD760x();
// //         // SOFT_PWM1_TOGGLE;
// //         // LED3_ON;
// //     }
// //     else
// //     {
// //         g_ADCIsrFlag = 1;
// //         EnableModuleCmpInterrupt(g_upperCurrModule, g_upperCurrIsA);
// //     }

// //     // SOFT_PWM2_TOGGLE;
// //     // if(!toggle_flag)
// //     // {
// //     // }
// //     // toggle_flag++;
// //     // toggle_flag %= 10;


// //     PieCtrlRegs.PIEACK.all = EPWM_PIE_GROUP_ACK;

// //     // LED1_TOGGLE;
// // }

// interrupt void EPWM_GlobalISR(void)
// {
//     // SOFT_PWM1_TOGGLE;
//     // 清触发模块的中断标志（若存在）
//     if (g_upperCurrModule >= 0 && g_upperCurrModule < 3)
//     {
//         volatile struct EPWM_REGS *Etrig = GetEPwmRegsByModule((Uint16)g_upperCurrModule);
//         Etrig->ETCLR.bit.INT = 1;
//     }

//     // ======================= 第一次中断处理 =======================
//     if (!g_firstIsrDone)
//     {
//         // DisableAllZROActions();

//         // 禁用AQCSFRC，让AQ自动动作接管

//         EALLOW;

//         for (Uint16 i = 0; i < 6; ++i)
//         {
//             volatile struct EPWM_REGS *E = ePWM_all[i];
//             E->AQCSFRC.bit.CSFA = 0; // 禁用软件强制
//             E->AQCSFRC.bit.CSFB = 0;
//         }
//         // 此时已经过了第一个CMP点，电平已经确认，后续动作时开始正常翻转
//         for (Uint16 i = 0; i < 6; ++i)
//         {
//             volatile struct EPWM_REGS *E = ePWM_all[i];
//             // 使用AQCSFRC设置初始电平
//             if (i < 3)
//             {
//                 // 上桥臂
//                 // Uint16 smA = i * 2;
//                 // Uint16 smB = i * 2 + 1;

//                 if (P_MMC_array[CMPA_IDX(i)].position_id < K_value)
//                 {
//                     // K模块：强制高，保持高
//                     // E->AQCSFRC.bit.CSFA = 2;
//                     E->AQCTLA.bit.CAU = AQ_SET;
//                 }
//                 else
//                 {
//                     // 非K模块：强制低，正常翻转
//                     // E->AQCSFRC.bit.CSFA = 1;
//                     E->AQCTLA.bit.CAU = AQ_TOGGLE;
//                 }

//                 if (P_MMC_array[CMPB_IDX(i)].position_id < K_value)
//                 {
//                     // E->AQCSFRC.bit.CSFB = 2;
//                     E->AQCTLB.bit.CBU = AQ_SET;
//                 }
//                 else
//                 {
//                     // E->AQCSFRC.bit.CSFB = 1;
//                     E->AQCTLB.bit.CBU = AQ_TOGGLE;
//                 }
//             }
//             else
//             {
//                 // 下桥臂
//                 Uint16 lowerIdx = i - 3;
//                 // Uint16 smA = lowerIdx * 2;
//                 // Uint16 smB = lowerIdx * 2 + 1;

//                 if (N_MMC_array[CMPA_IDX(lowerIdx)].position_id < K_value)
//                 {
//                     // K模块：强制高，保持高
//                     // E->AQCSFRC.bit.CSFA = 2;
//                     E->AQCTLA.bit.CAU = AQ_SET;
//                 }
//                 else
//                 {
//                     // 非K模块：强制高，正常翻转
//                     // E->AQCSFRC.bit.CSFA = 2;
//                     E->AQCTLA.bit.CAU = AQ_TOGGLE;
//                 }

//                 if (N_MMC_array[CMPB_IDX(lowerIdx)].position_id < K_value)
//                 {
//                     // E->AQCSFRC.bit.CSFB = 2;
//                     E->AQCTLB.bit.CBU = AQ_SET;
//                 }
//                 else
//                 {
//                     // E->AQCSFRC.bit.CSFB = 2;
//                     E->AQCTLB.bit.CBU = AQ_TOGGLE;
//                 }
//             }
//         }
//         EDIS;

//         g_firstIsrDone = 1;
//         g_ADCIsrFlag = 1;

//         // 现在反倒不需要这个中断了
//         // EnableEPWM4Interrupt();
//         PieCtrlRegs.PIEACK.all = EPWM_PIE_GROUP_ACK;
//         return;
//     }

//     toggle_flag = (toggle_flag == 0) ? 1 : 0;
//     if(!toggle_flag)
//     {
//         PieCtrlRegs.PIEACK.all = EPWM_PIE_GROUP_ACK;
//         return;
//     }

//     PieCtrlRegs.PIEACK.all = EPWM_PIE_GROUP_ACK;
//     if (g_ADCIsrFlag)
//     {
//         g_isrToggleCnt++;
//         // 触发后延迟2us再启动,等待开关管完全关断
//         // TODO
//         // StartAD760x();
//         BridgeArmSort();
//     }
//     else
//     {
//         g_ADCIsrFlag = 1;
//     }

//     // SOFT_PWM1_TOGGLE;
// }

// /* ================================================================
//  *                    EPWM_LowerISR
//  * ================================================================
//  *
//  * 这个 ISR 由 ePWM4~6 中的某一个触发。
//  *
//  * 唯一用途：N 桥臂 Active Partial 的 restore 中断。
//  *
//  * 场景：
//  *   奇数子周期（Update_N_K_Arm），N 桥臂有 Partial。
//  *   Partial 在 ePWM4~6 的某个模块上，借用了 borrowed compare。
//  *   到达 secondCmp 后触发本 ISR，恢复 borrowed compare。
//  */
// interrupt void EPWM_LowerISR(void)
// {
//     // TODO
//     // // SOFT_PWM2_TOGGLE;
//     // int16 srcModule;
//     // srcModule = FindPendingEpwmModule(3, 5);
//     // if (srcModule < 0)
//     // {
//     //     PieCtrlRegs.PIEACK.all = EPWM_PIE_GROUP_ACK;
//     //     return;
//     // }
//     // ePWM_all[srcModule]->ETCLR.bit.INT = 1;

//     // /* 只处理 N 桥臂 Active Partial 的 restore */
//     // if (g_partialActive.active &&
//     //     (g_partialActive.arm == PARTIAL_ARM_N) &&
//     //     ((Uint16)srcModule == g_partialActive.globalModule))
//     // {
//     //     Partial_RestoreActiveCtx(&g_partialActive);
//     // }
//     // else
//     // {
//     //     /* 意外的中断，关掉防止反复进入 */
//     //     DisableModuleCmpInterrupt((Uint16)srcModule);
//     // }
//     // // LED2_TOGGLE;

//     PieCtrlRegs.PIEACK.all = EPWM_PIE_GROUP_ACK;
// }

// void Change_K_Model_PWMWorkingStatus(Uint16 position, ChangeKStatus order)
// {
//     Uint16 moduleIndex;

//     if (position >= MMC_NUMBER)
//         return;

//     switch (order)
//     {
//     case Set_New_Channel_N:
//         moduleIndex = (position >> 1) + 3;
//         if ((position & 0x01U) == 0U)
//             ePWM_all[moduleIndex]->AQCTLA.bit.CAU = AQ_SET;
//         else
//             ePWM_all[moduleIndex]->AQCTLB.bit.CBU = AQ_SET;
//         break;

//     case Reset_Old_Channel_N:
//         moduleIndex = (position >> 1) + 3;
//         if ((position & 0x01U) == 0U)
//             ePWM_all[moduleIndex]->AQCTLA.bit.CAU = AQ_TOGGLE;
//         else
//             ePWM_all[moduleIndex]->AQCTLB.bit.CBU = AQ_TOGGLE;
//         break;

//     case Set_New_Channel_P:
//         moduleIndex = (position >> 1);
//         if ((position & 0x01U) == 0U)
//             ePWM_all[moduleIndex]->AQCTLA.bit.CAU = AQ_SET;
//         else
//             ePWM_all[moduleIndex]->AQCTLB.bit.CBU = AQ_SET;
//         break;

//     case Reset_Old_Channel_P:
//         moduleIndex = (position >> 1);
//         if ((position & 0x01U) == 0U)
//             ePWM_all[moduleIndex]->AQCTLA.bit.CAU = AQ_TOGGLE;
//         else
//             ePWM_all[moduleIndex]->AQCTLB.bit.CBU = AQ_TOGGLE;
//         break;

//     default:
//         break;
//     }
//     // LED2_ON;
// }

// // 加入排序，然后在dma的中断函数中调用
// // void BridgeArmSort(void)
// // {
// //     // SOFT_PWM2_TOGGLE;
// //     switch (g_isrToggleCnt & 0x01)
// //     {
// //     case Update_P_K_Arm:
// //         SOFT_PWM2_TOGGLE;
// //         /* --- 1. 更新 P 桥臂电压信息并排序 --- */
// //         // P排序
// //         // 记录上周期的K值，后续更新上桥臂的电压信息与开通顺序需要用到
// //         K_last_value_P = K_value;
// //         g_partialDuty_P = g_partialDuty;
// //         // 更新上桥臂的电压信息与预计的开通顺序,需要使用K_last_value,因为上周期的K模块数决定了要把几个模块放到最后面开通
// //         // TODO 测试不进行排序
// //         updata_voltage_information(UpdataUpperArm);
// //         // updata_voltage_information(UpdataLowerArm);
// //         // updata_voltage_information_forTest(UpdataUpperArm);

// //         /* --- 2. 计算新 K 和 D --- */
// //         // 更新K值
// //         // 注释：U_in = AD7606_1_Buf[6] * 5 / 32768.0 * 400; // 400 = 8000kΩ / 20kΩ
// //         // U_in = AD7606_1_Buf[6] / 32768.0 * 2000;
// //         // if (U_in < TARGET_LLC_VOLTAGE) // 过低的输入电压可能是测量异常
// //         //     U_in = TARGET_LLC_VOLTAGE;
// //         // else if (U_in > 800) // 过高的输入电压可能是测量异常
// //         //     U_in = 800;

// //         // TODO 为了测试把输入电压U_in全部设置为260，后面记得改回来
// //         // U_in = 260.0;

// //         if (SoftStart_IsActive())
// //         {
// //             /* 软启动期间：K/D 由软启动状态机给出，不根据输入电压实时变化 */
// //             SoftStart_GetCommand(&K_value, &g_partialDuty);
// //         }
// //         else
// //         {
// //             float32 kdValue;
// //             Uint16 newK;
// //             float32 newD;

// //             kdValue = Get_K_D_Method(U_in);
// //             DecodeKDValue(kdValue, &newK, &newD);

// //             K_value = newK;
// //             g_partialDuty = newD;
// //         }

// //         /* --- 3. 切换对家 N 桥臂 K 模块 --- */
// //         // 更新对家K模块导通动作并且预测上周期的K模块上周期结束时的电压情况(切换K模块时先关断以前的再开启新的，顺序不能反)

// //         if (K_last_value_N != 0)
// //         {
// //             for (Uint16 i = 0; i < MMC_NUMBER; i++)
// //             {
// //                 if (N_MMC_array[i].last_position_id < K_last_value_N)
// //                 {
// //                     Change_K_Model_PWMWorkingStatus(i, Reset_Old_Channel_N);
// //                     *updat_voltage_index_record_ptr = i;
// //                     updat_voltage_index_record_ptr++;
// //                 }
// //                 else if ((N_MMC_array[i].position_id == K_last_value_N) && (g_partialDuty_N > 0.0f))
// //                 {
// //                     *updat_voltage_index_record_ptr = i;
// //                     updat_voltage_index_record_ptr++;
// //                 }
// //                 else
// //                 {
// //                     predict_voltage_diff += (AD7606_2_Buf[i] - N_MMC_array[i].recent_voltage);
// //                 }
// //             }
// //             predict_voltage_diff /= (MMC_NUMBER - K_last_value_N - (g_partialDuty_N > 0.0f ? 1 : 0));
// //             for (Uint16 i = 0; i < (K_last_value_N + (g_partialDuty_N > 0.0f ? 1 : 0)); i++)
// //             {
// //                 N_MMC_array[updat_voltage_index_record[i]].recent_voltage = AD7606_2_Buf[updat_voltage_index_record[i]] - predict_voltage_diff;
// //             }
// //             predict_voltage_diff = 0;
// //             updat_voltage_index_record_ptr = updat_voltage_index_record; // 重置指针，准备下一次记录
// //         }
// //         if (K_value != 0)
// //         {
// //             // 更新对家桥臂的K模块PWM变换方式
// //             for (Uint16 i = 0; i < MMC_NUMBER; i++)
// //             {
// //                 if (N_MMC_array[i].position_id < K_value)
// //                 {
// //                     Change_K_Model_PWMWorkingStatus(i, Set_New_Channel_N);
// //                 }
// //             }
// //         }

// //         if (SoftStart_FreqClosedLoopEnabled())
// //         {
// //             /* --- 4. 更新频率（影响下一子周期 T=TBPRD） --- */
// //             EPWM_FrequencyUpdate();
// //             /* --- 5. 更新桥臂误差 PID --- */
// //             EPWM_ErrorUpdate(Update_P_K_Arm);
// //         }


// //         /* --- 6. 为下一子周期（奇数）构造 N 桥臂 Partial 计划 ---
// //         *
// //         * 下一子周期使用 CMP_Values 表（偶数→奇数切换时装载）。
// //         * N 桥臂模块基址 = 3（ePWM4~6）。
// //         *
// //         * 注意这里只构造 Prepared，不注入 secondCmp。
// //         * secondCmp 在下一次 EPWM_GlobalISR 中注入。
// //         */
// //         Partial_BuildPrepared(N_MMC_array, CMP_Values, 3, g_partialDuty, CMP_addError_Values[0], &g_partialPrepared);

// //         /* --- 7. 写入上下桥臂全部 12 路 compare ---
// //         *
// //         * 使用 CMP_Values 表。
// //         * Partial 模块的 own compare 被替换为 firstCmp。
// //         * 如果某 borrowed compare 正被 Active Partial 占用，defer。
// //         */
// //         Partial_WriteAllCmpForNextSubcycle(CMP_Values, &g_partialPrepared);

// //         /* --- 8. 选举下一次主中断源 --- */
// //         Uint16 m, isA;
// //         FindMaxCmp(&m, &isA);
// //         QueueUpperMainInterrupt(m, isA);

// //         SOFT_PWM2_TOGGLE;
// //         break;

// //     // &0x01后为1
// //     case Update_N_K_Arm:
// //         /* 每经过一个完整周期推进一次软启动状态机。
// //         * 放在 Update_N 的开头，是为了让状态变化立刻作用到“即将写入下一周期”的 PWM 数据。
// //         */
// //         SOFT_PWM1_TOGGLE;
// //         if (SoftStart_IsActive())
// //         {
// //             SoftStart_OnFullCycleTick();
// //             SoftStart_GetCommand(&K_value, &g_partialDuty);
// //         }
// //         // N排序
// //         /* --- 1. 更新 N 桥臂电压信息并排序 --- */
// //         K_last_value_N = K_value;
// //         g_partialDuty_N = g_partialDuty; // 重置N桥臂的partial duty cycle
// //         // TODO 测试不进行排序
// //         updata_voltage_information(UpdataLowerArm);
// //         // updata_voltage_information(UpdataUpperArm);
// //         // updata_voltage_information_forTest(UpdataLowerArm);

// //         /* --- 2. 更新误差 PID --- */
// //         // 更新上下桥臂电压误差
// //         EPWM_ErrorUpdate(Update_N_K_Arm);
    
// //         /* --- 3. 为下一子周期（偶数）构造 P 桥臂 Partial 计划 ---
// //          *
// //          * 下一子周期使用 CMP_addError_Values 表。
// //          * P 桥臂模块基址 = 0（ePWM1~3）。
// //          */
// //         Partial_BuildPrepared(P_MMC_array, CMP_addError_Values, 0, g_partialDuty, CMP_Values[0], &g_partialPrepared);

// //         /* --- 4. 写入上下桥臂全部 12 路 compare ---
// //          * 使用 CMP_addError_Values 表。
// //          */
// //         Partial_WriteAllCmpForNextSubcycle(CMP_addError_Values, &g_partialPrepared);

// //         /* --- 5. 切换对家 P 桥臂 K 模块 --- */
// //         // 更新对家K模块导通动作并且预测上周期的K模块上周期结束时的电压情况(切换K模块时先关断以前的再开启新的，顺序不能反)
// //         if (K_last_value_P != 0)
// //         {
// //             for (Uint16 i = 0; i < MMC_NUMBER; i++)
// //             {
// //                 if (P_MMC_array[i].last_position_id < K_last_value_P)
// //                 {
// //                     Change_K_Model_PWMWorkingStatus(i, Reset_Old_Channel_P);
// //                     *updat_voltage_index_record_ptr = i;
// //                     updat_voltage_index_record_ptr++;
// //                 }
// //                 else if ((P_MMC_array[i].position_id == K_last_value_P) && (g_partialDuty_P > 0.0f))
// //                 {
// //                     *updat_voltage_index_record_ptr = i;
// //                     updat_voltage_index_record_ptr++;
// //                 }
// //                 else
// //                 {
// //                     predict_voltage_diff += (AD7606_1_Buf[i] - P_MMC_array[i].recent_voltage);
// //                 }
// //             }
// //             predict_voltage_diff /= (MMC_NUMBER - K_last_value_P - (g_partialDuty_P > 0.0f ? 1 : 0));
// //             for (Uint16 i = 0; i < (K_last_value_P + (g_partialDuty_P > 0.0f ? 1 : 0)); i++)
// //             {
// //                 P_MMC_array[updat_voltage_index_record[i]].recent_voltage = AD7606_1_Buf[updat_voltage_index_record[i]] - predict_voltage_diff;
// //             }
// //             predict_voltage_diff = 0;
// //             updat_voltage_index_record_ptr = updat_voltage_index_record; // 重置指针，准备下一次记录
// //         }
// //         if (K_value != 0)
// //         {
// //             // 更新对家桥臂的K模块PWM变换方式
// //             for (Uint16 i = 0; i < MMC_NUMBER; i++)
// //             {
// //                 if (P_MMC_array[i].position_id < K_value)
// //                 {
// //                     Change_K_Model_PWMWorkingStatus(i, Set_New_Channel_P);
// //                 }
// //             }
// //         }

// //         /* --- 6. 选举下一次主中断源 --- */
// //         {
// //             Uint16 m, isA;
// //             FindMaxCmp(&m, &isA);
// //             QueueUpperMainInterrupt(m, isA);
// //         }

// //         SOFT_PWM1_TOGGLE;

// //         break;

// //     default:
// //         break;
// //     }
// //     // SOFT_PWM2_TOGGLE;
// // }

// // 加入排序，然后在dma的中断函数中调用
// void BridgeArmSort(void)
// {
//     switch (g_isrToggleCnt & 0x01)
//     {
//         case Update_P_K_Arm:
//         SOFT_PWM2_TOGGLE;
//         // P排序
//         // 记录上周期的K值，后续更新上桥臂的电压信息与开通顺序需要用到
//         K_last_value_P = K_value;
//         // 更新上桥臂的电压信息与预计的开通顺序,需要使用K_last_value,因为上周期的K模块数决定了要把几个模块放到最后面开通
//         updata_voltage_information(UpdataUpperArm);

//         for (Uint16 i = 0; i < 3; ++i)
//         {
//             Uint16 idxA = P_MMC_array[CMPA_IDX(i)].position_id;
//             Uint16 idxB = P_MMC_array[CMPB_IDX(i)].position_id;
//             ePWM_all[i]->CMPA.half.CMPA = CMP_Values[idxA];
//             ePWM_all[i]->CMPB = CMP_Values[idxB];
//         }

//         for (Uint16 i = 3; i < 6; ++i)
//         {
//             Uint16 idxA = N_MMC_array[CMPA_IDX(i - 3)].position_id;
//             Uint16 idxB = N_MMC_array[CMPB_IDX(i - 3)].position_id;
//             ePWM_all[i]->CMPA.half.CMPA = CMP_Values[idxA];
//             ePWM_all[i]->CMPB = CMP_Values[idxB];
//         }

//         // 更新K值
//         // U_in = AD7606_1_Buf[6] * 5 / 32768.0 * 400; // 400 = 8000kΩ / 20kΩ
//         // U_in = (AD7606_1_Buf[6] / 32768.0) * 2000;

//         if (U_in < 200) // 过低的输入电压可能是测量异常
//         {
//             U_in = 200;
//         }
//         else if (U_in > 800) // 过高的输入电压可能是测量异常
//         {
//             U_in = 800;
//         }  
//         K_value = Get_K_FloorMethod(U_in);

//         // 更新对家K模块导通动作并且预测上周期的K模块上周期结束时的电压情况(切换K模块时先关断以前的再开启新的，顺序不能反)
//         if (K_last_value_N != 0)
//         {
//             for (Uint16 i = 0; i < MMC_NUMBER; i++)
//             {
//                 if (N_MMC_array[i].last_position_id < K_last_value_N)
//                 {
//                     Change_K_Model_PWMWorkingStatus(i, Reset_Old_Channel_N);
//                     *updat_voltage_index_record_ptr = i;
//                     updat_voltage_index_record_ptr++;
//                 }
//                 else
//                 {
//                     predict_voltage_diff += (AD7606_2_Buf[i] - N_MMC_array[i].recent_voltage);
//                 }
//             }
//             predict_voltage_diff /= (MMC_NUMBER - K_last_value_N);
//             for (Uint16 i = 0; i < K_last_value_N; i++)
//             {
//                 N_MMC_array[updat_voltage_index_record[i]].recent_voltage = AD7606_2_Buf[updat_voltage_index_record[i]] - predict_voltage_diff;
//             }
//             predict_voltage_diff = 0;
//             updat_voltage_index_record_ptr = updat_voltage_index_record; // 重置指针，准备下一次记录
//         }
//         if (K_value != 0)
//         {
//             // 更新对家桥臂的K模块PWM变换方式
//             for (Uint16 i = 0; i < MMC_NUMBER; i++)
//             {
//                 if (N_MMC_array[i].position_id < K_value)
//                 {
//                     Change_K_Model_PWMWorkingStatus(i, Set_New_Channel_N);
//                 }
//             }
//         }

//         // 更新方波频率
//         EPWM_FrequencyUpdate();

//         // 更新上下桥臂电压误差
//         EPWM_ErrorUpdate(Update_P_K_Arm);

//         // 重新选举下一个最大CMP并安全切换中断源（若发生变化）
//         // Uint16 m, isA;
//         // // FindMaxCmp(&m, &isA);
//         // isA = (g_upperCurrIsA == 0) ? 1 : 0;
//         // m = (g_upperCurrModule >= 2) ? 0 : (g_upperCurrModule + 1);
//         // if ((int16)m != g_upperCurrModule || (int16)isA != g_upperCurrIsA)
//         // {
//         //     DisableModuleCmpInterrupt(g_upperCurrModule); // 只关掉上一个
//         //     // DisableAllModuleInterrupts();
//         //     EnableModuleCmpInterrupt(m, isA);
//         //     g_upperCurrModule = (int16)m;
//         //     g_upperCurrIsA = (int16)isA;
//         // }
//         DisableAllModuleInterrupts();
//         // DisableModuleCmpInterrupt(g_upperCurrModule);
//         g_upperCurrIsA = (g_upperCurrIsA == 0) ? 1 : 0;
//         g_upperCurrModule = (g_upperCurrModule >= 2) ? 0 : (g_upperCurrModule + 1);
//         EnableModuleCmpInterrupt(g_upperCurrModule, g_upperCurrIsA);
//         SOFT_PWM2_TOGGLE;
//         break;

//     // &0x01后为1
//     case Update_N_K_Arm:
//         // N排序
//         SOFT_PWM3_TOGGLE;
//         K_last_value_N = K_value;
//         updata_voltage_information(UpdataLowerArm);
//         for (Uint16 i = 0; i < 3; ++i)
//         {
//             Uint16 idxA = P_MMC_array[CMPA_IDX(i)].position_id;
//             Uint16 idxB = P_MMC_array[CMPB_IDX(i)].position_id;
//             ePWM_all[i]->CMPA.half.CMPA = CMP_addError_Values[idxA];
//             ePWM_all[i]->CMPB = CMP_addError_Values[idxB];
//         }

//         for (Uint16 i = 3; i < 6; ++i)
//         {
//             Uint16 idxA = N_MMC_array[CMPA_IDX(i - 3)].position_id;
//             Uint16 idxB = N_MMC_array[CMPB_IDX(i - 3)].position_id;
//             ePWM_all[i]->CMPA.half.CMPA = CMP_addError_Values[idxA];
//             ePWM_all[i]->CMPB = CMP_addError_Values[idxB];
//         }

//         // 更新对家K模块导通动作并且预测上周期的K模块上周期结束时的电压情况(切换K模块时先关断以前的再开启新的，顺序不能反)
//         if (K_last_value_P != 0)
//         {
//             for (Uint16 i = 0; i < MMC_NUMBER; i++)
//             {
//                 if (P_MMC_array[i].last_position_id < K_last_value_P)
//                 {
//                     Change_K_Model_PWMWorkingStatus(i, Reset_Old_Channel_P);
//                     *updat_voltage_index_record_ptr = i;
//                     updat_voltage_index_record_ptr++;
//                 }
//                 else
//                 {
//                     predict_voltage_diff += (AD7606_1_Buf[i] - P_MMC_array[i].recent_voltage);
//                 }
//             }
//             predict_voltage_diff /= (MMC_NUMBER - K_last_value_P);
//             for (Uint16 i = 0; i < K_last_value_P; i++)
//             {
//                 P_MMC_array[updat_voltage_index_record[i]].recent_voltage = AD7606_1_Buf[updat_voltage_index_record[i]] - predict_voltage_diff;
//             }
//             predict_voltage_diff = 0;
//             updat_voltage_index_record_ptr = updat_voltage_index_record; // 重置指针，准备下一次记录
//         }
//         if (K_value != 0)
//         {
//             // 更新对家桥臂的K模块PWM变换方式
//             for (Uint16 i = 0; i < MMC_NUMBER; i++)
//             {
//                 if (P_MMC_array[i].position_id < K_value)
//                 {
//                     Change_K_Model_PWMWorkingStatus(i, Set_New_Channel_P);
//                 }
//             }
//         }

//         // 更新上下桥臂电压误差
//         EPWM_ErrorUpdate(Update_N_K_Arm);
//         SOFT_PWM3_TOGGLE;

//         break;

//     default:
//         break;
//     }
// }

// void EPWM_ResetISRFlags(void)
// {
//     g_firstIsrDone = 0;
//     g_isrToggleCnt = 0;
//     g_ADCIsrFlag = 0;

//     g_upperCurrModule  = -1;
//     g_upperCurrIsA     = -1;
//     g_upperNextModule  = -1;
//     g_upperNextIsA     = -1;
//     g_upperNextPending = 0U;

//     g_partialDuty = 0.0f;
//     Partial_ClearPrepared(&g_partialPrepared);
//     Partial_ClearActive(&g_partialActive);

//     DisableAllModuleInterrupts();
// }

// // ==================== 安全停止 ====================
// void EPWM_SafeStop(void)
// {
//     EALLOW;
    
//     DisableAllModuleInterrupts();

//     /* 清空 Partial 状态（这部分逻辑仍然需要，防止中断残留） */
//     Partial_ClearPrepared(&g_partialPrepared);
//     Partial_ClearActive(&g_partialActive);
    
//     // 切换回GPIO模式
//     GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;
    
//     // 全部置高（安全状态）
//     GpioDataRegs.GPASET.all = 0x00000FFF;
    
//     EDIS;
    
//     PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

//     g_ADCIsrFlag = 0;
    
//     printf("[EPWM] SafeStop (GPIO mode, all high)\r\n");
// }

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
