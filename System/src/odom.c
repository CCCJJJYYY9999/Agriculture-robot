#include "odom.h"
#include "main.h"

/*
 * 这里填你的真实参数
 *
 * ENCODER_CPR 表示：轮子转一圈，编码器计数增加多少
 *
 * 例如：
 * 编码器 1024 线，AB 相四倍频，编码器直接装在轮子轴上：
 * ENCODER_CPR = 1024 * 4 = 4096
 *
 * 如果编码器装在电机轴，电机到轮子有 30:1 减速：
 * ENCODER_CPR = 1024 * 4 * 30 = 122880
 */
#define ENCODER_CPR             3136LL

/*
 * 轮子直径，单位：um
 * 65mm = 65000um
 */
#define WHEEL_DIAMETER_UM       63800LL

/*
 * 方向修正
 * 如果发现前进时距离是负的，把 1 改成 -1
 */
#define ENCODER_DIR             1

/*
 * Q16 定点数
 * 1um 被放大 65536 倍
 */
#define Q16                     65536LL

/*
 * PI_Q16 = pi * 65536
 * 3.1415926 * 65536 ≈ 205887
 */
#define PI_Q16                  205887LL

/*
 * 每个编码器计数对应多少 um，使用 Q16 定点数表示
 *
 * 距离公式：
 * distance = count * pi * wheel_diameter / ENCODER_CPR
 */
#define UM_PER_COUNT_Q16        ((WHEEL_DIAMETER_UM * PI_Q16) / ENCODER_CPR)

/*
 * 有符号位移，前进增加，后退减少
 * 单位：um * Q16
 */
static volatile int64_t g_position_um_q16 = 0;

/*
 * 总里程，只增加
 * 单位：um * Q16
 */
static volatile uint64_t g_mileage_um_q16 = 0;

/*
 * 总编码器计数，有正负
 */
static volatile int64_t g_total_count = 0;

/*
 * 当前速度，单位 mm/s
 */
static volatile int32_t g_speed_mm_s = 0;


uint32_t mileage_mm;
int32_t position_mm;
int32_t speed_mm_s;

/*
 * 保护 64 位变量读取
 * STM32F407 是 32 位 MCU，读写 int64_t 不是原子操作
 */
static uint32_t Odom_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Odom_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}


void Odom_Init(void)
{
    Odom_Reset();
}


void Odom_Reset(void)
{
    uint32_t primask = Odom_EnterCritical();

    g_position_um_q16 = 0;
    g_mileage_um_q16 = 0;
    g_total_count = 0;
    g_speed_mm_s = 0;

    Odom_ExitCritical(primask);
}


/*
 * 每 10ms 调用一次
 *
 * delta_cnt 是 10ms 内编码器增加的脉冲数
 * 前进为正，后退为负
 */
void Odom_UpdateDelta(int32_t delta_cnt)
{
    int32_t delta;
    int64_t delta_um_q16;
    uint32_t abs_delta;

    delta = delta_cnt * ENCODER_DIR;

    delta_um_q16 = (int64_t)delta * UM_PER_COUNT_Q16;

    g_total_count += delta;
    g_position_um_q16 += delta_um_q16;

    if (delta >= 0)
    {
        abs_delta = delta;
    }
    else
    {
        abs_delta = -delta;
    }

    g_mileage_um_q16 += (uint64_t)abs_delta * UM_PER_COUNT_Q16;

    /*
     * 10ms = 0.01s
     *
     * delta_um / 0.01 = delta_um * 100 um/s
     * 再除以 1000 变成 mm/s
     *
     * 所以：
     * speed_mm_s = delta_um / 10
     */
    g_speed_mm_s = (int32_t)(delta_um_q16 / (10LL * Q16));
}


int64_t Odom_GetPositionUm(void)
{
    int64_t temp;
    uint32_t primask = Odom_EnterCritical();

    temp = g_position_um_q16;

    Odom_ExitCritical(primask);

    return temp / Q16;
}


int32_t Odom_GetPositionMm(void)
{
    return (int32_t)(Odom_GetPositionUm() / 1000);
}


uint64_t Odom_GetMileageUm(void)
{
    uint64_t temp;
    uint32_t primask = Odom_EnterCritical();

    temp = g_mileage_um_q16;

    Odom_ExitCritical(primask);

    return temp / Q16;
}


uint32_t Odom_GetMileageMm(void)
{
    return (uint32_t)(Odom_GetMileageUm() / 1000);
}


int32_t Odom_GetSpeedMmS(void)
{
    int32_t temp;
    uint32_t primask = Odom_EnterCritical();

    temp = g_speed_mm_s;

    Odom_ExitCritical(primask);

    return temp;
}


int64_t Odom_GetTotalCount(void)
{
    int64_t temp;
    uint32_t primask = Odom_EnterCritical();

    temp = g_total_count;

    Odom_ExitCritical(primask);

    return temp;
}