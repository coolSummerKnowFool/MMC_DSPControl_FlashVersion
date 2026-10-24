#ifndef __EPWM_PRECHARGE_H
#define __EPWM_PRECHARGE_H

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "basic_control.h"

// ==================== 预充电均压配置参数 ====================
#define PRECHARGE_STABLE_DELAY_US   500U        // 每次切换后等待稳定时间(us)
#define PRECHARGE_VOLTAGE_TOLERANCE 50          // 均压完成判定阈值（AD码值）
#define PRECHARGE_MAX_ITERATIONS    2000U       // 最大迭代次数

#define UPPER_ARM_COUNT             6           // 上桥臂子模块数
#define LOWER_ARM_COUNT             6           // 下桥臂子模块数
#define TOTAL_SM_COUNT              12          // 总子模块数

#define AD7606_CONVERSION_TIME_US   8           // AD7606C-16转换时间+余量

// ==================== 预充电状态枚举 ====================
typedef enum {
    PRECHARGE_IDLE = 0,
    PRECHARGE_WAIT_VOLTAGE,
    PRECHARGE_CALC_N,
    PRECHARGE_INITIAL_CHARGE,
    PRECHARGE_BALANCING,
    PRECHARGE_FINAL_SETTLE,
    PRECHARGE_COMPLETE,
    PRECHARGE_TIMEOUT,
    PRECHARGE_ERROR
} PrechargeState;

// ==================== 子模块状态 ====================
typedef enum {
    SM_BYPASSED = 0,            // 旁路（GPIO低）
    SM_INSERTED                 // 接入（GPIO高）
} SubmoduleState;

// ==================== 均压阶段 ====================
typedef enum {
    BALANCE_PHASE_CHARGE = 0,
    BALANCE_PHASE_DISCHARGE
} BalancePhase;

// ==================== 子模块信息 ====================
typedef struct {
    Uint16 smIndex;             // 物理索引 (0-5)
    Uint16 isUpperArm;          // 1=上桥臂, 0=下桥臂
    int16  voltage;             // 当前电压
} PrechargeSmInfo;

// ==================== 预充电管理结构体 ====================
typedef struct {
    PrechargeState state;                       // 当前运行状态
    Uint16 targetN;                             // N = 6 + K
    Uint16 K_value;                             // 当前K值  
    Uint16 currentBalanceLevel;                 // 从12递减到N
    Uint16 iterationCount;                      // 循环计数器，用于超时判定或者多次执行保证稳定
    BalancePhase phase;                         // 当前均压阶段（分为充电/放电两种）
    
    PrechargeSmInfo smList[TOTAL_SM_COUNT];     // 全部12个子模块
    
    int16  maxVoltage;
    int16  minVoltage;
    int16  voltageSpread;
    
    int16  upperVoltages[UPPER_ARM_COUNT];      // 上桥臂电压数组
    int16  lowerVoltages[LOWER_ARM_COUNT];      // 下桥臂电压数组
    
    Uint16 upperSortedIdx[UPPER_ARM_COUNT];     // 上桥臂排序索引
    
} PrechargeManager;

// ==================== 全局变量声明 ====================
extern volatile PrechargeManager g_precharge;

// ==================== GPIO控制函数 ====================
void GPIO_PrechargeInit(void);                              // 初始化为GPIO模式
void GPIO_SetUpperSM(Uint16 smIndex, SubmoduleState state);
void GPIO_SetLowerSM(Uint16 smIndex, SubmoduleState state);
void GPIO_SetAllUpperArm(SubmoduleState state);
void GPIO_SetAllLowerArm(SubmoduleState state);

// ==================== 预充电流程 ====================
void Precharge_Start(void);
PrechargeState Precharge_Process(void);
Uint16 Precharge_IsComplete(void);

// ==================== 辅助函数 ====================
void Precharge_SampleAllVoltages(void);
void Precharge_SortAllByVoltage(void);
Uint16 Precharge_CalculateN(void);

// ==================== 模式切换 ====================
void EPWM_DisableAllInterrupts(void);
void EPWM_EnableClosedLoopInterrupts(void);
void Precharge_PrepareMMCArrays(void);

void EPWM_DisableAllInterrupts(void);

#endif
