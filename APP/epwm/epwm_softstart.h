#ifndef __EPWM_SOFTSTART_H
#define __EPWM_SOFTSTART_H

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "basic_control.h"

/* ══════════════════════════════════════════════════════
 *                   软启动参数配置
 * ══════════════════════════════════════════════════════ */

/* 软启动期间频率固定为 INIT_FREQUENCE（子周期频率）
 * 完整周期频率 = 子周期频率 / 2 */
#define SOFTSTART_FULLCYCLE_HZ          ((Uint32)(INIT_FREQUENCE / 2))

/* 毫秒 → 完整周期数 */
#define SOFTSTART_MS_TO_CYCLES(ms) \
    ((Uint32)((SOFTSTART_FULLCYCLE_HZ * (Uint32)(ms)) / 1000UL))

/* ---------- 各阶段停留时间 ---------- */

/* K 递减阶段：每个 K 值的最小停留时间 */
#define SOFTSTART_K_DWELL_MIN_MS        150UL
/* K 递减阶段：每个 K 值的最大停留时间（超时强制推进） */
#define SOFTSTART_K_DWELL_MAX_MS        400UL

/* 到达 targetK 后（D 仍为 0）的稳定时间 */
#define SOFTSTART_TARGET_K_HOLD_MS      200UL

/* 启用 Partial 后的预热时间 */
#define SOFTSTART_PARTIAL_PRIME_MS      150UL

/* 最终稳定时间（之后开启频率闭环） */
#define SOFTSTART_FINAL_SETTLE_MS       200UL

/* ---------- 电压稳定判据 ---------- */

/* 使能电压稳定判据（1=在最小/最大停留时间之间用电压判据提前推进）
 * 0 = 只用固定时间（最简单，最保守） */
#define SOFTSTART_USE_VOLT_CRITERION    0

/* 桥臂内电压极差阈值（AD 码值）
 * AD7606C ±5V 差分，16bit → 1 LSB ≈ 152.6μV
 * 若子模块电压量程为 0~50V，分压比 1:10，则 1V ≈ 655 码值
 * 阈值 400 ≈ 0.6V 极差 */
#define SOFTSTART_VOLT_SPREAD_TH        400

/* 电压稳定需要连续满足的完整周期数 */
#define SOFTSTART_VOLT_STABLE_CYCLES    50UL

/* ---------- ADC 预采样 ---------- */

/* 软启动前手动触发 ADC 后的等待时间
 * AD7606C 转换 ~3.45μs + DMA 搬运 + 余量 */
#define SOFTSTART_ADC_WAIT_US           30U

/* 预采样次数（取多次采样让数据稳定） */
#define SOFTSTART_PRESAMPLE_COUNT       4U

/* ══════════════════════════════════════════════════════
 *                      状态机定义
 * ══════════════════════════════════════════════════════ */

typedef enum
{
    SS_STAGE_IDLE = 0,          /* 未启动 */
    SS_STAGE_RAMP_K,            /* K 从 6 递减到 targetK，D=0 */
    SS_STAGE_TARGET_K_HOLD,     /* K=targetK, D=0，稳定等待 */
    SS_STAGE_PARTIAL_PRIME,     /* K=targetK, D=targetD，预热 */
    SS_STAGE_FINAL_SETTLE,      /* 最终稳定，准备开环→闭环 */
    SS_STAGE_DONE,              /* 完成，频率闭环已开启 */
    SS_STAGE_FAULT              /* 故障 */
} SoftStartStage;

typedef struct
{
    /* ---- 运行标志 ---- */
    Uint16 active;              /* 1=软启动进行中 */
    Uint16 complete;            /* 1=软启动完成 */
    Uint16 freqLoopEnable;      /* 1=允许频率 PID 生效 */

    SoftStartStage stage;

    /* ---- 目标值（启动时根据 U_in 计算一次） ---- */
    float32 targetKD;           /* Get_K_D_Method 的原始返回值 */
    Uint16  targetK;
    float32 targetD;

    /* ---- 当前输出给 BridgeArmSort 的值 ---- */
    Uint16  currentK;
    float32 currentD;

    /* ---- 计时 ---- */
    Uint32 stageCycles;         /* 当前阶段已运行的完整周期数 */
    Uint32 voltStableCycles;    /* 电压连续稳定的周期数 */

    /* ---- 预计算的周期阈值 ---- */
    Uint32 kDwellMinCycles;
    Uint32 kDwellMaxCycles;
    Uint32 targetKHoldCycles;
    Uint32 partialPrimeCycles;
    Uint32 finalSettleCycles;

    /* ---- 统计 ---- */
    Uint32 totalCycles;         /* 软启动总共运行的完整周期数 */
    Uint16 kStepCount;          /* K 递减了多少次 */
} SoftStartManager;

extern volatile SoftStartManager g_softStart;

/* ══════════════════════════════════════════════════════
 *              异步消息（供主循环打印，避免中断内 printf）
 * ══════════════════════════════════════════════════════ */

typedef enum
{
    SS_MSG_NONE = 0,
    SS_MSG_K_STEP,              /* K 递减一步 */
    SS_MSG_ENTER_TARGET_HOLD,   /* 进入 targetK 保持阶段 */
    SS_MSG_ENTER_PARTIAL,       /* 启用 Partial */
    SS_MSG_ENTER_SETTLE,        /* 进入最终稳定 */
    SS_MSG_COMPLETE             /* 完成 */
} SoftStartMsgType;

typedef struct
{
    Uint16  pending;            /* 1=有待打印消息 */
    Uint16  type;               /* SoftStartMsgType */
    Uint16  k;
    float32 d;
    Uint32  cycles;             /* 该阶段耗费的周期数 */
} SoftStartMsg;

extern volatile SoftStartMsg g_ssMsg;

/* ══════════════════════════════════════════════════════
 *                       对外接口
 * ══════════════════════════════════════════════════════ */

void   SoftStart_Init(void);
void   SoftStart_Reset(void);

/* 启动软启动：内部完成 ADC 预采样、目标 KD 计算、MMC 数组初始化
 * 返回 1=成功，0=失败（应转入 FAULT） */
Uint16 SoftStart_Start(void);

/* 状态查询 */
Uint16 SoftStart_IsActive(void);
Uint16 SoftStart_IsComplete(void);
Uint16 SoftStart_FreqLoopEnabled(void);

/* 获取当前应使用的 K / D */
void   SoftStart_GetCommand(volatile Uint16 *k, volatile float32 *d);

/* 每个完整周期调用一次，推进状态机
 * 调用位置：BridgeArmSort() 的 Update_P_K_Arm 分支中，K/D 赋值之前 */
void   SoftStart_OnFullCycleTick(void);

/* 主循环调用：打印待处理的异步消息 */
void   SoftStart_ProcessMessages(void);

/* 调试：打印当前完整状态 */
void   SoftStart_PrintStatus(void);

#endif /* __EPWM_SOFTSTART_H */
