#include "basic_control.h"
#include "dma_xintf.h"
#include "math.h"
#include "epwm_combine.h"

mmc_info P_MMC_array[MMC_NUMBER];
mmc_info N_MMC_array[MMC_NUMBER];
volatile float U_in = 260.0f, U_load = 100.0f;
volatile Uint16 K_value = 0;
Uint16 K_last_value_P = 0;
Uint16 K_last_value_N = 0;
float32 g_partialDuty_P = 0.0f;
float32 g_partialDuty_N = 0.0f;

mmc_info_ptr P_MMC_pointers_by_diff[MMC_NUMBER];
mmc_info_ptr P_MMC_pointers_by_voltage[MMC_NUMBER];
mmc_info_ptr N_MMC_pointers_by_diff[MMC_NUMBER];
mmc_info_ptr N_MMC_pointers_by_voltage[MMC_NUMBER];

#pragma CODE_SECTION(sort_desc_diff, ".TI.ramfunc")
#pragma CODE_SECTION(sort_asc_voltage, ".TI.ramfunc")
#pragma CODE_SECTION(updata_voltage_information, ".TI.ramfunc")
#pragma CODE_SECTION(updata_voltage_information_forTest, ".TI.ramfunc")
#pragma CODE_SECTION(ensure_position, ".TI.ramfunc")
#pragma CODE_SECTION(move_small_last_pos_to_end, ".TI.ramfunc")
#pragma CODE_SECTION(Get_K_ff_Method, ".TI.ramfunc")
#pragma CODE_SECTION(cswap_desc_diff, ".TI.ramfunc")
#pragma CODE_SECTION(cswap_asc_voltage, ".TI.ramfunc")
#pragma CODE_SECTION(cswap_position, ".TI.ramfunc")
// #pragma CODE_SECTION(adjust_partial_position, ".TI.ramfunc")

#pragma FUNC_ALWAYS_INLINE(sort_desc_diff)
#pragma FUNC_ALWAYS_INLINE(sort_asc_voltage)

// 上半桥与下半桥MMC元素初始化
void MMC_Init(mmc_info *P_MMC_array, mmc_info *N_MMC_array)
{
    // 初始化全局mmc_info结构体变量
    for (int i = 0; i < MMC_NUMBER; i++)
    {
        P_MMC_array[i].voltage_diff = 0;
        P_MMC_array[i].recent_voltage = 0;
        P_MMC_array[i].last_position_id = i;
        P_MMC_array[i].position_id = i;

        P_MMC_pointers_by_diff[i] = &P_MMC_array[i];
        P_MMC_pointers_by_voltage[i] = &P_MMC_array[i];

        N_MMC_array[i].voltage_diff = 0;
        N_MMC_array[i].recent_voltage = 0;
        N_MMC_array[i].last_position_id = i;
        N_MMC_array[i].position_id = i;

        N_MMC_pointers_by_diff[i] = &N_MMC_array[i];
        N_MMC_pointers_by_voltage[i] = &N_MMC_array[i];
    }
}

// 降序：diff 大的在前
static inline void cswap_desc_diff(mmc_info_ptr *arr, int a, int b)
{
    if (arr[a]->voltage_diff < arr[b]->voltage_diff)
    {
        mmc_info_ptr t = arr[a];
        arr[a] = arr[b];
        arr[b] = t;
    }
}

// 升序：电压小的在前
static inline void cswap_asc_voltage(mmc_info_ptr *arr, int a, int b)
{
    if (arr[a]->recent_voltage > arr[b]->recent_voltage)
    {
        mmc_info_ptr t = arr[a];
        arr[a] = arr[b];
        arr[b] = t;
    }
}

// 交换两个指针的指向位置
static inline void cswap_position(mmc_info_ptr *arr, int a, int b)
{
    mmc_info_ptr t = arr[a];
    arr[a] = arr[b];
    arr[b] = t;
}

/**
 * @brief 将指针数组中满足 (last_position_id < K_last_value_P) 的元素全部移到末尾
 * @param arr             指向指针数组的指针（即 P_MMC_pointers_by_diff）
 * @param n               数组长度（MMC_NUMBER）
 * @param K_last_value_P  判定阈值
 */
void move_small_last_pos_to_end(mmc_info_ptr *arr, int n, int16 K_last_value_P)
{
    int left = 0;
    int right = n - 1;

    while (left < right) {
        /* 从左边找到第一个需要移到后面的元素（满足条件） */
        while (left < right && arr[left]->last_position_id >= K_last_value_P) {
            left++;
        }
        /* 从右边找到第一个可以留在前面的元素（不满足条件） */
        while (left < right && arr[right]->last_position_id < K_last_value_P) {
            right--;
        }
        /* 交换左右指针指向的指针，并移动指针 */
        if (left < right) {
            cswap_position(arr, left, right);
            left++;
            right--;
        }
    }
}

static inline void sort_desc_diff(mmc_info_ptr *arr, int n)
{
    switch (n)
    {
    case 6:
        SORT6_BODY(cswap_desc_diff, arr);
        break;
    case 5:
        SORT5_BODY(cswap_desc_diff, arr);
        break;
    case 4:
        SORT4_BODY(cswap_desc_diff, arr);
        break;
    case 3:
        SORT3_BODY(cswap_desc_diff, arr);
        break;
    case 2:
        SORT2_BODY(cswap_desc_diff, arr);
        break;
    default:
        break;
    }
}

static inline void sort_asc_voltage(mmc_info_ptr *arr, int n)
{
    switch (n)
    {
    case 6:
        SORT6_BODY(cswap_asc_voltage, arr);
        break;
    case 5:
        SORT5_BODY(cswap_asc_voltage, arr);
        break;
    case 4:
        SORT4_BODY(cswap_asc_voltage, arr);
        break;
    case 3:
        SORT3_BODY(cswap_asc_voltage, arr);
        break;
    case 2:
        SORT2_BODY(cswap_asc_voltage, arr);
        break;
    default:
        break;
    }
}

// 对电压差与当前电压进行排序并将下一次的开通顺序写入position_id中
void ensure_position(mmc_info_ptr *diff_pointer, mmc_info_ptr *voltage_pointer, Uint16 lastKValue)
{
    int activeCount = MMC_NUMBER - lastKValue;
    // mmc_info_ptr t;
    if (activeCount <= 0) return;

    // 根据电压差值进行降序排序
    sort_desc_diff(diff_pointer, activeCount);

    // 根据当前电压进行升序排序
    sort_asc_voltage(voltage_pointer, activeCount);

    // // 对 diff_pointer 按电压差降序排序 (插入排序，确定性)
    // for (int i = 1; i < activeCount; i++)
    // {
    //     for (int j = i; j > 0; j--)
    //     {
    //         // cswap_desc_diff(diff_pointer, j - 1, j);
    //         if (diff_pointer[j - 1]->voltage_diff < diff_pointer[j]->voltage_diff)
    //         {
    //             t = diff_pointer[j - 1];
    //             diff_pointer[j - 1] = diff_pointer[j];
    //             diff_pointer[j] = t;
    //         }
    //     }
    // }

    // // // 对 voltage_pointer 按当前电压升序排序
    // for (int i = 1; i < activeCount; i++)
    // {
    //     for (int j = i; j > 0; j--)
    //     {
    //         // cswap_asc_voltage(voltage_pointer, j - 1, j);
    //         if (voltage_pointer[j - 1]->recent_voltage > voltage_pointer[j]->recent_voltage)
    //         {
    //             t = voltage_pointer[j - 1];
    //             voltage_pointer[j - 1] = voltage_pointer[j];
    //             voltage_pointer[j] = t;
    //         }
    //     }
    // }

    // 将 diff 排序的旧位置映射到 voltage 排序后的模块
    for (int i = 0; i < activeCount; i++)
    {
        voltage_pointer[i]->position_id = diff_pointer[i]->last_position_id - lastKValue;
    }
}

/*
 * adjust_partial_position()
 *
 * 功能：
 *   排序完成后，检查 position_id == K_value 的子模块（Partial 候选）
 *   是否落在受限通道上。如果是，与相邻的半开通子模块交换 position_id。
 *
 * 受限通道：
 *   P 桥臂：物理子模块索引 0, 1（对应 ePWM1 的 A/B 通道）
 *   N 桥臂：物理子模块索引 0, 1（对应 ePWM4 的 A/B 通道）
 *
 * 参数：
 *   armArray    : P_MMC_array 或 N_MMC_array
 *   currentK    : 当前 K 值
 *   partialDuty : 当前 D 值（D <= 0 时无需调整）
 *   restricted0 : 受限的物理子模块索引 0（P桥臂传0，N桥臂传0）
 *   restricted1 : 受限的物理子模块索引 1（P桥臂传1，N桥臂传1）
 *
 * 算法：
 *   1. 找到 position_id == K_value 的物理子模块索引 partialSM
 *   2. 如果 partialSM 不在受限通道，直接返回
 *   3. 如果在受限通道，从 position_id == K_value+1 开始向后搜索
 *      找到第一个不在受限通道的半开通子模块，交换两者的 position_id
 *   4. 如果所有半开通子模块都在受限通道（极端情况），放弃 Partial
 *      （返回 0 表示失败，由调用者将 D 置零）
 *
 * 返回：
 *   1 = Partial 位置合法（或成功调整）
 *   0 = 无法安排 Partial（所有候选都在受限通道）
 */
// Uint16 adjust_partial_position(mmc_info *armArray,
//                                Uint16 currentK,
//                                float32 partialDuty,
//                                Uint16 restricted0,
//                                Uint16 restricted1)
// {
//     Uint16 posToSm[MMC_NUMBER];   /* posToSm[position_id] = 物理子模块索引 */
//     Uint16 partialSM;
//     Uint16 i;

//     if (partialDuty <= 0.001f)
//         return 1;

//     /* 一次遍历构建逆映射 */
//     for (i = 0; i < MMC_NUMBER; i++)
//         posToSm[armArray[i].position_id] = i;

//     partialSM = posToSm[currentK];

//     /* 不在受限通道 → 直接返回 */
//     if ((partialSM != restricted0) && (partialSM != restricted1))
//         return 1;

//     /* 向后搜索可交换目标（直接查表，无嵌套循环） */
//     {
//         Uint16 targetPosId;
//         for (targetPosId = currentK + 1; targetPosId < MMC_NUMBER; targetPosId++)
//         {
//             Uint16 candidate = posToSm[targetPosId];
//             if ((candidate != restricted0) && (candidate != restricted1))
//             {
//                 /* 交换 position_id */
//                 armArray[partialSM].position_id = targetPosId;
//                 armArray[candidate].position_id = currentK;
//                 // SortOverFlag = 1;
//                 return 1;
//             }
//         }
//     }

//     return 0;   /* 所有候选都在受限通道 */
// }

// 更新桥臂电压参数并排序
void updata_voltage_information(UpdataWhichArm flag)
{
    Uint16 excludedCount = 0;
    switch (flag)
    {
    case UpdataUpperArm:
        for (int i = 0; i < MMC_NUMBER; i++)
        {
            // 本周期全开通的模块，下周期放到最后面开通
            if (P_MMC_array[i].position_id < K_last_value_P)
            {
                P_MMC_array[i].last_position_id = P_MMC_array[i].position_id;
                P_MMC_array[i].position_id = MMC_NUMBER - K_last_value_P + P_MMC_array[i].last_position_id;
            }
            else if((P_MMC_array[i].position_id == K_last_value_P) && (g_partialDuty > 0.0f)) // 本周期是Partial模块，下周期放到最后面开通
            {
                P_MMC_array[i].last_position_id = P_MMC_array[i].position_id;
                P_MMC_array[i].position_id = MMC_NUMBER - K_last_value_P - 1;
            }
            // 其余模块正常更新电压差与电压值
            else
            {
                P_MMC_array[i].voltage_diff = (int16)(AD7606_1_Buf[i] - P_MMC_array[i].recent_voltage);
                P_MMC_array[i].recent_voltage = AD7606_1_Buf[i];
                P_MMC_array[i].last_position_id = P_MMC_array[i].position_id;
            }
        }
        // 将本周期全开通的子模块的position_id调整到最后面开通的位置上，后续排序直接忽略本周期全开通的子模块
        // 包括对Partial模块的处理
        excludedCount = K_last_value_P + (g_partialDuty > 0.0f ? 1 : 0);
        if(excludedCount > 0)
        {
            move_small_last_pos_to_end(P_MMC_pointers_by_diff, MMC_NUMBER, excludedCount);
            move_small_last_pos_to_end(P_MMC_pointers_by_voltage, MMC_NUMBER, excludedCount);
        }
        
        ensure_position(P_MMC_pointers_by_diff, P_MMC_pointers_by_voltage, excludedCount);
        /*
         * 【新增】Partial 通道约束后处理
         *
         * P 桥臂受限通道：物理子模块 0, 1（ePWM1 的 A/B）
         * 如果 Partial 候选落在子模块 0 或 1，与后续半开通子模块交换
         * 如果无法交换（极端情况），将 g_partialDuty 置零放弃 Partial
         *
         * 注意：这里使用的是当前周期的 K_value（不是 K_last_value_P）
         * 因为 ensure_position 产生的 position_id 是为下一子周期准备的
         * 而 K_value 在 BridgeArmSort 中已经被更新为最新值
         */
        // if (g_partialDuty > 0.0f)
        // {
        //     if (!adjust_partial_position(P_MMC_array, K_value, g_partialDuty,
        //                                  PARTIAL_RESTRICTED_SM_P0,
        //                                  PARTIAL_RESTRICTED_SM_P1))
        //     {
        //         /* 无法安排 Partial，退化为普通半开通 */
        //         g_partialDuty = 0.0f;
        //     }
        // }
        break;
    case UpdataLowerArm:
        for (int i = 0; i < MMC_NUMBER; i++)
        {
            // 本周期全开通的模块，下周期放到最后面开通
            if (N_MMC_array[i].position_id < K_last_value_N)
            {
                N_MMC_array[i].last_position_id = N_MMC_array[i].position_id;
                N_MMC_array[i].position_id = MMC_NUMBER - K_last_value_N + N_MMC_array[i].last_position_id;
                // N_MMC_array[i].recent_voltage = AD7606_2_Buf[i];
            }
            else if((N_MMC_array[i].position_id == K_last_value_N) && (g_partialDuty > 0.0f)) // 本周期是Partial模块，下周期放到最后面开通
            {
                N_MMC_array[i].last_position_id = N_MMC_array[i].position_id;
                N_MMC_array[i].position_id = MMC_NUMBER - K_last_value_N - 1;
                // N_MMC_array[i].recent_voltage = AD7606_2_Buf[i];
            }
            // 其余模块正常更新电压差与电压值
            else
            {
                N_MMC_array[i].voltage_diff = (int16)(AD7606_2_Buf[i] - N_MMC_array[i].recent_voltage);
                N_MMC_array[i].recent_voltage = AD7606_2_Buf[i];
                N_MMC_array[i].last_position_id = N_MMC_array[i].position_id;
            }
        }
        // 将本周期全开通的子模块的position_id调整到最后面开通的位置上，后续排序直接忽略本周期全开通的子模块
        // 包括对Partial模块的处理
        excludedCount = K_last_value_N + (g_partialDuty > 0.0f ? 1 : 0);
        if(excludedCount > 0)
        {
            move_small_last_pos_to_end(N_MMC_pointers_by_diff, MMC_NUMBER, excludedCount);
            move_small_last_pos_to_end(N_MMC_pointers_by_voltage, MMC_NUMBER, excludedCount);
        }
        
        ensure_position(N_MMC_pointers_by_diff, N_MMC_pointers_by_voltage, excludedCount);
        /*
         * 【新增】N 桥臂 Partial 通道约束后处理
         *
         * N 桥臂受限通道：物理子模块 0, 1（ePWM4 的 A/B）
         */
        // if (g_partialDuty > 0.0f)
        // {
        //     if (!adjust_partial_position(N_MMC_array, K_value, g_partialDuty,
        //                                  PARTIAL_RESTRICTED_SM_N0,
        //                                  PARTIAL_RESTRICTED_SM_N1))
        //     {
        //         g_partialDuty = 0.0f;
        //     }
        // }
        break;
    default:
        break;
    }
}

void updata_voltage_information_forTest(UpdataWhichArm flag)
{
    switch (flag)
    {
    case UpdataUpperArm:
        for (int i = 0; i < MMC_NUMBER; i++)
        {
            P_MMC_array[i].last_position_id = P_MMC_array[i].position_id;
            P_MMC_array[i].position_id += 1;
            P_MMC_array[i].position_id %= 6;
        }
        break;
    case UpdataLowerArm:
        for (int i = 0; i < MMC_NUMBER; i++)
        {
            N_MMC_array[i].last_position_id = N_MMC_array[i].position_id;
            N_MMC_array[i].position_id += 1;
            N_MMC_array[i].position_id %= 6;
        }
        break;
    default:
        break;
    }
}

// TODO 这个函数后期要删除
// 根据输入电压计算K值的大小
int Get_K_FloorMethod(float U_in)
{
    return (int)floor((U_in - BASE_VOLTAGE) / (U_in + BASE_VOLTAGE) * MMC_NUMBER);
}

float32 Get_K_ff_Method(float U_in)
{
    return (U_in - TARGET_LLC_VOLTAGE) / (U_in + TARGET_LLC_VOLTAGE) * MMC_NUMBER;
}

// ---高电平使能，低电平锁存！！！---  (更换元器件了)
// 目前是低电平使能，高电平锁存
void MosEnableGPIO_Init(void)
{
    EALLOW;                                  // 关闭写保护
    SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1; // 开启GPIO时钟

    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0; // 设置为通用GPIO功能
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;  // 设置GPIO方向为输出
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;  // 不使能GPIO上拉电阻

    // GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
    GpioDataRegs.GPASET.bit.GPIO12 = 1;

    EDIS; // 开启写保护
}

// 切换工作状态
void MosWorkStatusChange(void)
{
    MOS_WORK_STATUS_TOGGLE;
}
