#ifndef __BASIC_CONTROL_H
#define __BASIC_CONTROL_H

#include "DSP2833x_Device.h"   // DSP2833x 头文件
#include "DSP2833x_Examples.h" // DSP2833x 例子相关头文件

#define MOS_WORK_DISABLE		(GpioDataRegs.GPACLEAR.bit.GPIO12 = 1)
#define MOS_WORK_ENABLE	        (GpioDataRegs.GPASET.bit.GPIO12 = 1)
#define MOS_WORK_STATUS_TOGGLE  (GpioDataRegs.GPATOGGLE.bit.GPIO12 = 1)

#define MMC_NUMBER 6
#define BASE_VOLTAGE 200.0f
#define TARGET_VOLTAGE 100.0f

#define INIT_FREQUENCE 20000.0f

// #define fre_PID_P 0.0
// #define fre_PID_I 0.0
// #define fre_PID_D 0.0
#define fre_PID_P 220.0f
#define fre_PID_I 8.0f
#define fre_PID_D 120.0f

// #define FREQ_MIN 20000  // 8000 * 2 Hz
// #define FREQ_MAX 20000 // 11500 * 2 Hz
#define FREQ_MIN 16000.0f  // 8000 * 2 Hz
#define FREQ_MAX 23000.0f // 11500 * 2 Hz

// #define err_PID_P 0.0
// #define err_PID_I 0.0
// #define err_PID_D 0.0
#define err_PID_P 1.0f
#define err_PID_I 0.005f
#define err_PID_D 0.1f

#define ERROR_MAX 200          
// #define ERROR_MAX 0          
#define ERROR_MIN (-ERROR_MAX) 
#define TARGET_ERROR 0.0f 
#define INIT_ERROR 0.0f

// #define duty_PID_P 0.0
// #define duty_PID_I 0.0
// #define duty_PID_D 0.0
#define duty_PID_P 0.00002f
#define duty_PID_I 0.000025f
#define duty_PID_D 0.000001f

#define DUTY_MAX 0.0f      
// #define DUTY_MAX 0.4f
#define DUTY_MIN (-DUTY_MAX) 
#define TARGET_LLC_VOLTAGE 260.0f  // 130*2  
#define INIT_DUTY_CYCLE 0.0f   

#define SORT6_BODY(cswap, arr) \
    cswap(arr,0,5); cswap(arr,1,3); cswap(arr,2,4); \
    cswap(arr,1,2); cswap(arr,3,4); \
    cswap(arr,0,3); cswap(arr,2,5); \
    cswap(arr,0,1); cswap(arr,2,3); cswap(arr,4,5); \
    cswap(arr,1,2); cswap(arr,3,4)

#define SORT5_BODY(cswap, arr) \
    cswap(arr,0,1); cswap(arr,3,4); \
    cswap(arr,2,4); cswap(arr,2,3); \
    cswap(arr,0,3); cswap(arr,1,4); \
    cswap(arr,1,2); cswap(arr,2,3); \
    cswap(arr,0,1)

#define SORT4_BODY(cswap, arr) \
    cswap(arr,0,1); cswap(arr,2,3); \
    cswap(arr,0,2); cswap(arr,1,3); \
    cswap(arr,1,2);

#define SORT3_BODY(cswap, arr) \
    cswap(arr,0,1); \
    cswap(arr,0,2); \
    cswap(arr,1,2);

#define SORT2_BODY(cswap, arr) \
    cswap(arr,0,1);

// /* Partial 受限通道定义
//  *
//  * P 桥臂：ePWM1 对应物理子模块 0（A通道）和 1（B通道）
//  * N 桥臂：ePWM4 对应物理子模块 0（A通道）和 1（B通道）
//  *
//  * 这些子模块不能被分配为 Partial 模块，
//  * 因为 ePWM1 CMPA 被主中断占用，ePWM4 CMPA 被恢复中断占用
//  */
// #define PARTIAL_RESTRICTED_SM_P0  0   /* ePWM1A 对应的物理子模块索引 */
// #define PARTIAL_RESTRICTED_SM_P1  1   /* ePWM1B 对应的物理子模块索引 */
// #define PARTIAL_RESTRICTED_SM_N0  0   /* ePWM4A 对应的物理子模块索引 */
// #define PARTIAL_RESTRICTED_SM_N1  1   /* ePWM4B 对应的物理子模块索引 */

typedef struct
{
    // int16 mmc_id;               // MMC的序号
    int16 voltage_diff;       // 本次与上次循环的电压差  (直接用来排序的值，单位是AD7606的码值，不转换成真是电压了)
    Uint16 recent_voltage;     // 本次循环时的电压
    Uint16 last_position_id;     // 上次循环中该MMC模块第几个导通
    Uint16 position_id;          // 本次循环中该MMC模块第几个导通
    // int16 work_state;           // 工作在正常状态还是K状态
} mmc_info; // 同时定义mmc_info和指向它的指针类型mmc_info_ptr

typedef mmc_info* mmc_info_ptr;

typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float prev_error;   // 上一次误差
    float prev2_error;  // 上两次误差
    float output;       // 当前输出值
} PID; // 定义PID结构体

typedef enum{
    UpdataUpperArm,
    UpdataLowerArm
} UpdataWhichArm;

extern mmc_info P_MMC_array[];
extern mmc_info N_MMC_array[];
extern volatile float U_in, U_load;
extern volatile Uint16 K_value;
extern Uint16 K_last_value_P;
extern Uint16 K_last_value_N;
extern float32 g_partialDuty_P;
extern float32 g_partialDuty_N;

void MosEnableGPIO_Init(void);
void MosWorkStatusChange(void);

void MMC_Init(mmc_info *P_MMC_array, mmc_info *N_MMC_array);
void updata_voltage_information(UpdataWhichArm flag);
void ensure_position(mmc_info_ptr *diff_pointer, mmc_info_ptr *voltage_pointer, Uint16 lastKValue);
int Get_K_FloorMethod(float U_in);
float32 Get_K_ff_Method(float U_in);

void updata_voltage_information_forTest(UpdataWhichArm flag);

// /* Partial 通道约束调整函数 */
// Uint16 adjust_partial_position(mmc_info *armArray, Uint16 currentK, float32 partialDuty, Uint16 restricted0, Uint16 restricted1);


#endif /* __BASIC_CONTROL_H */
