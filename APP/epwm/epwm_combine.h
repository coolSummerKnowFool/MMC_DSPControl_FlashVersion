#ifndef __EPWM_COMBINE_H
#define __EPWM_COMBINE_H

#include "DSP2833x_Device.h"   // DSP2833x 头文件
#include "DSP2833x_Examples.h" // DSP2833x 例子相关头文件
#include "basic_control.h"

// ---------------- 配置参数（如需调整在此修改） ----------------
#define SYSCLK_HZ           150000000UL   // 系统时钟（请与你的板子实际值一致）
#define PWM_FREQ_MIN_HZ     16000U         // 8 kHz
#define PWM_FREQ_MAX_HZ     24000U        // 12 kHz
#define PWM_PAIR_COUNT      3             // ePWM1/2/3 三对 (每对 A/B)
#define EPWM_PIE_GROUP_ACK  PIEACK_GROUP3 // ePWM 通常在 PIE group3（如不同请修改）

// ---------------- 全局变量（外部可读/写，供上层使用） ----------------
// 三对 CMP 的影子值（用户可在主循环或其它任务中更新这些数组，ISR 会写入硬件）
// extern volatile Uint16 CMPA_val[PWM_PAIR_COUNT]; // 每对的 CMPA（用于 A 通道）
// extern volatile Uint16 CMPB_val[PWM_PAIR_COUNT]; // 每对的 CMPB（用于 B 通道）

// 目标频率（Hz），主循环可修改，ISR 在写硬件时使用该值计算 TBPRD
// extern volatile Uint32 g_targetFreqHz;

/* ================ Partial 模式参数 ================ */
#define PARTIAL_D_EPS              0.002f   /* D 小于此值时不做 Partial（Test2 确认） */
#define PARTIAL_D_MAX              0.497f   /* D 大于此值时钳位（Test2 确认） */
#define PARTIAL_MIN_GAP_CNT        32U      /* firstCmp 与 secondCmp 最小间隔 */
#define PARTIAL_INJECT_GUARD_CNT   100U     /* 注入前确认所有 CMP 已翻转的裕量 */

/* secondCmp 距离 TBPRD 的最小间隔
 * 保证第二次翻转的硬件 AQ 动作在 PRD 事件之前完成。
 * 8 counts ≈ 53ns @150MHz，对 ePWM 硬件足够 */
#define PARTIAL_SECOND_GUARD_CNT   8U

/* restore 中断固定使用 ePWM4 的 PRD 事件
 * 全局模块索引：ePWM4 = 3 */
#define PARTIAL_RESTORE_MODULE     3U

typedef enum ArmUpdateStatus{
    Update_N_K_Arm,
    Update_P_K_Arm,
} ArmUpdateStatus;

typedef enum ChangeKStatus{
    Set_New_Channel_N,
    Reset_Old_Channel_N,
    Set_New_Channel_P,
    Reset_Old_Channel_P
} ChangeKStatus;

typedef enum
{
    PARTIAL_ARM_P = 0,    /* P桥臂（上桥臂），对应 ePWM1~3 */
    PARTIAL_ARM_N = 1     /* N桥臂（下桥臂），对应 ePWM4~6 */
} PartialArmSel;

/*
 * Partial Prepared 上下文
 *
 * 生命周期：BridgeArmSort(子周期n) 构造 → BridgeArmSort(子周期n+1) 开头消费
 */
typedef struct
{
    Uint16 valid;
    Uint16 arm;             /* PARTIAL_ARM_P / PARTIAL_ARM_N */

    Uint16 smIndex;         /* 物理子模块索引 0..5 */
    Uint16 localModule;     /* 桥臂内 ePWM 对索引 0..2 */
    Uint16 globalModule;    /* 全局 ePWM 索引 0..5 */
    Uint16 channelIsA;      /* 1=A 通道，0=B 通道 */

    Uint16 ownBaseCmp;      /* 原始 compare = t */
    Uint16 firstCmp;        /* 第一翻转 = t + T_p * D */
    Uint16 secondCmp;       /* 第二翻转 = t + T_p * (1-D) */
} PartialPreparedCtx;

/*
 * Partial Active 上下文
 *
 * 生命周期：BridgeArmSort 注入 → EPWM_RestoreISR(ePWM4 PRD) 恢复
 *
 * 相比旧方案：去掉了 savedRestoreCmp/savedRestoreAQ 及其 deferred 字段
 * 因为 ePWM4 不再被借用，完全不碰 ePWM4 的任何寄存器
 */
typedef struct
{
    Uint16 active;
    Uint16 arm;

    Uint16 smIndex;
    Uint16 localModule;
    Uint16 globalModule;    /* Partial 所在 ePWM 索引 */
    Uint16 channelIsA;

    Uint16 secondCmpVal;

    /* borrowed compare 保存信息（Partial 所在 ePWM 的另一路） */
    Uint16 savedBorrowCmp;
    Uint16 savedPairedAQ;

    /* deferred：BridgeArmSort 想写 borrowed compare 时缓存 */
    Uint16 deferredBorrowCmp;
    Uint16 deferredBorrowCmpValid;

    /* deferred：想修改 paired AQ 时缓存 */
    Uint16 deferredPairedAQ;
    Uint16 deferredPairedAQValid;
} PartialActiveCtx;

// TODO 这些变量记得好好查查有没有必要extern
extern Uint16 CMP_Values[];
extern Uint16 CMP_addError_Values[];
extern Uint16 g_ADCIsrFlag; // 标记是否进入ADC的中断内的桥臂控制函数(预充电期间不执行，闭环期间执行)

// extern volatile Uint16 SortOverFlag;

extern volatile float32 g_partialDuty;
extern PartialPreparedCtx g_partialPrepared;
extern volatile PartialActiveCtx   g_partialActive;

extern volatile Uint32 g_partialInjectSkipCnt;
extern volatile Uint32 g_partialRestoreMissCnt;
extern volatile Uint16  g_partialTimeBase;
extern volatile float32 g_partialDcrit;

extern PID fre_PID;
extern PID err_PID;
extern PID duty_PID;
extern float32 arm_per_voltage_P;
extern float32 arm_per_voltage_N;


// fromPrecharge: 1=从预充电模式切换（GPIO已配置），0=冷启动（需要配置GPIO）
void EPWM_SwitchToClosedLoop(Uint16 fromPrecharge);
#define EPWM_ModuleInit()  EPWM_SwitchToClosedLoop(0)
// void EPWM_ModuleInit(void);
interrupt void EPWM_GlobalISR(void);
// interrupt void EPWM_LowerISR(void);

// void EnableEPWM4Interrupt(void);
// void DisableEPWM4Interrupt(void);
// interrupt void EPWM4_ISR(void);

void BridgeArmSort(void);
// 重置函数声明
void EPWM_ResetISRFlags(void);
// void EPWM_SafeStop(void);

// void EPWM_ResetControlState(void);
// float32 Get_K_D_Method(double U_in);

/* 对外接口 */
// interrupt void EPWM_RestoreISR(void);
Uint16 Partial_TryInjectNow(void);     /* BridgeArmSort 开头调用 */
void Partial_ForceAbort(void);       /* 异常/停机时强制恢复 */

void EPWM_ResetControlState(void);
void EPWM_SafeStop(void);

#endif /* __EPWM_COMBINE_H */
