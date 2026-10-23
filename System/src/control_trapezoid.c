#include "control_trapezoid.h"
#include "main.h"

/*
 * This file only calls existing public interfaces:
 * - set_Dmotor_speed()
 * - Odom_Reset()
 * - HAL_GetTick()/HAL_Delay()
 * - StepperMotorset_pos()
 *
 * It does not modify PID, encoder, odom, ultrasonic or stepper modules.
 */

static float s_cmd_speed = 0.0f;
static uint32_t s_last_tick = 0u;

static float trap_abs_f(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float trap_min_f(float a, float b)
{
    return (a < b) ? a : b;
}

static float trap_clamp_f(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}


static uint8_t trap_valid_csb_index(uint8_t index)
{
    return (index < 8u) ? 1u : 0u;
}

static uint16_t trap_min_csb(uint8_t csb_a, uint8_t csb_b)
{
    uint16_t da = 0xFFFFu;
    uint16_t db = 0xFFFFu;

    if (trap_valid_csb_index(csb_a)) da = dis[csb_a];
    if (trap_valid_csb_index(csb_b)) db = dis[csb_b];

    return (da < db) ? da : db;
}

static int32_t trap_travel_mm(int8_t dir)
{
    if (dir >= 0)
    {
        return position_mm;
    }
    else
    {
        return -position_mm;
    }
}

static float trap_speed_limit_by_odom(int8_t dir, int32_t target_mm)
{
    int32_t travel = trap_travel_mm(dir);
    int32_t remain = target_mm - travel;
    float speed = TRAPEZOID_V_MAX;

		if (remain <= 0)
		{
				return TRAPEZOID_V_MIN;
		}

    if (remain < TRAPEZOID_ODOM_DEC_MM)
    {
        float ratio = (float)remain / (float)TRAPEZOID_ODOM_DEC_MM;
        speed = TRAPEZOID_V_MIN + (TRAPEZOID_V_MAX - TRAPEZOID_V_MIN) * ratio;
        speed = trap_clamp_f(speed, TRAPEZOID_V_MIN, TRAPEZOID_V_MAX);
    }

    return speed;
}

static float trap_speed_limit_by_csb(uint8_t csb_a, uint8_t csb_b)
{
    uint16_t d = trap_min_csb(csb_a, csb_b);
    float speed = TRAPEZOID_V_MAX;

    /* If both indexes are invalid, do not limit speed by ultrasonic. */
    if (d == 0xFFFFu)
    {
        return speed;
    }

		if (d <= TRAPEZOID_CSB_STOP_MM)
		{
				return TRAPEZOID_V_MIN;
		}
    if (d < TRAPEZOID_CSB_SLOW_MM)
    {
        float ratio = ((float)d - (float)TRAPEZOID_CSB_STOP_MM) /
                      ((float)TRAPEZOID_CSB_SLOW_MM - (float)TRAPEZOID_CSB_STOP_MM);
        speed = TRAPEZOID_V_MIN + (TRAPEZOID_V_MAX - TRAPEZOID_V_MIN) * ratio;
        speed = trap_clamp_f(speed, TRAPEZOID_V_MIN, TRAPEZOID_V_MAX);
    }

    return speed;
}

static void trap_apply_speed(float target_speed)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms;
    float step;
    float diff;
    float rate;

    if (s_last_tick == 0u)
    {
        s_last_tick = now;
    }

    dt_ms = now - s_last_tick;

    /* Avoid a large speed jump after HAL_Delay() or debugger pause. */
    if (dt_ms > 100u)
    {
        dt_ms = 100u;
    }

    diff = target_speed - s_cmd_speed;
    rate = (trap_abs_f(target_speed) < trap_abs_f(s_cmd_speed)) ?
           TRAPEZOID_DEC_PER_SEC : TRAPEZOID_ACC_PER_SEC;

    step = rate * ((float)dt_ms / 1000.0f);

    if (step > 0.0f)
    {
        if (trap_abs_f(diff) <= step)
        {
            s_cmd_speed = target_speed;
        }
        else if (diff > 0.0f)
        {
            s_cmd_speed += step;
        }
        else
        {
            s_cmd_speed -= step;
        }
    }

    s_last_tick = now;

    set_Dmotor_speed(&left.speed, &right.speed, s_cmd_speed, s_cmd_speed);
}

void Trapezoid_Reset(void)
{
    s_cmd_speed = 0.0f;
    s_last_tick = 0u;
}

void Trapezoid_Stop(void)
{
    s_cmd_speed = 0.0f;
    s_last_tick = 0u;
    set_Dmotor_speed(&left.speed, &right.speed, 0.0f, 0.0f);
}

void Trapezoid_StopAndResetOdom(void)
{
    Trapezoid_Stop();
    Odom_Reset();
}

void Trapezoid_RunByTarget(int8_t dir,
                           int32_t target_mm,
                           uint8_t csb_a,
                           uint8_t csb_b,
                           uint8_t use_csb)
{
    float limit_speed = trap_speed_limit_by_odom(dir, target_mm);
    float target_speed;

    if (use_csb != 0u)
    {
        limit_speed = trap_min_f(limit_speed, trap_speed_limit_by_csb(csb_a, csb_b));
    }

    target_speed = (dir >= 0) ? limit_speed : -limit_speed;
    trap_apply_speed(target_speed);
}

uint8_t Trapezoid_ArrivedByOdom(int8_t dir, int32_t target_mm)
{
    int32_t travel = trap_travel_mm(dir);
    return (travel >= target_mm) ? 1u : 0u;
}

uint8_t Trapezoid_ArrivedByCsb(uint8_t csb_a, uint8_t csb_b)
{
    uint16_t d = trap_min_csb(csb_a, csb_b);

    if (d == 0xFFFFu)
    {
        return 0u;
    }

    return (d <= TRAPEZOID_CSB_STOP_MM) ? 1u : 0u;
}

static void trap_turn_to_270(void)
{
    StepperMotorset_pos(&huart5, &motor_l, 1, 100, 270);
    StepperMotorset_pos(&huart6, &motor_r, 1, 100, 270);
}

static void trap_turn_to_0(void)
{
    StepperMotorset_pos(&huart5, &motor_l, 0, 100, 0);
    StepperMotorset_pos(&huart6, &motor_r, 0, 100, 0);
}

static void trap_after_segment_common(void)
{
    HAL_Delay(1000);
    Trapezoid_Reset();
}

/*
 * Replacement route control.
 * To keep other program parts unchanged, leave Main_Control() in control.c,
 * and change only its body to: Main_Control_Trapezoid();
 */
void Main_Control_Trapezoid(void)
{
    switch (count)
    {
        case 0:
            Trapezoid_Reset();
            count = 1;
            break;

        case 1:
            Trapezoid_RunByTarget(1, Start_Y, 3, 2, TRAPEZOID_CSB_DISABLE);
            if (position_mm >= Start_Y)
            {
                if (dis[3] >= csb_X_far && dis[2] >= csb_X_far)
                {
                    count = 2;
                    Trapezoid_StopAndResetOdom();
                    trap_turn_to_270();
                    trap_after_segment_common();
                }
            }
            break;

        case 2:
            Trapezoid_RunByTarget(1, LongDao_X, 3, 2, TRAPEZOID_CSB_ENABLE);
            if (position_mm >= LongDao_X)
            {
                if (dis[3] <= csb_X_near && dis[2] <= csb_X_near &&
                    dis[1] >= csb_X_far && dis[4] >= csb_X_far)
                {
                    count = 3;
                    Trapezoid_StopAndResetOdom();
                    trap_turn_to_0();
                    trap_after_segment_common();
                }
            }
            break;

        case 3:
            Trapezoid_RunByTarget(1, Long_Go_Long, 1, 4, TRAPEZOID_CSB_DISABLE);
            if (position_mm >= Long_Go_Long)
            {
                if (dis[1] >= csb_X_far && dis[4] >= csb_X_far)
                {
                    count = 4;
                    Trapezoid_StopAndResetOdom();
                    trap_turn_to_270();
                    trap_after_segment_common();
                }
            }
            break;

        case 4:
            Trapezoid_RunByTarget(-1, LongDao_X, 1, 4, TRAPEZOID_CSB_ENABLE);
            if ((-position_mm) >= LongDao_X)
            {
                if (dis[1] <= csb_X_near && dis[4] <= csb_X_near &&
                    dis[2] >= csb_X_far && dis[3] >= csb_X_far)
                {
                    count = 5;
                    Trapezoid_StopAndResetOdom();
                    trap_turn_to_0();
                    trap_after_segment_common();
                }
            }
            break;

        case 5:
            Trapezoid_RunByTarget(1, Long_Go_Long, 2, 3, TRAPEZOID_CSB_DISABLE);
            if (position_mm >= Long_Go_Long)
            {
                if (dis[2] >= csb_X_far && dis[3] >= csb_X_far)
                {
                    count = 6;
                    Trapezoid_StopAndResetOdom();
                    trap_turn_to_270();
                    trap_after_segment_common();
                }
            }
            break;

        case 6:
            Trapezoid_RunByTarget(1, LongDao_X, 2, 3, TRAPEZOID_CSB_ENABLE);
            if (position_mm >= LongDao_X)
            {
                if (dis[2] <= csb_X_near && dis[3] <= csb_X_near &&
                    dis[1] >= csb_X_far && dis[4] >= csb_X_far)
                {
                    count = 7;
                    Trapezoid_StopAndResetOdom();
                    trap_turn_to_0();
                    trap_after_segment_common();
                }
            }
            break;

        case 7:
            Trapezoid_RunByTarget(1, Long_Go_Long, 1, 4, TRAPEZOID_CSB_DISABLE);
            if (position_mm >= Long_Go_Long)
            {
                if (dis[1] >= csb_X_far && dis[4] >= csb_X_far)
                {
                    count = 8;
                    Trapezoid_StopAndResetOdom();
                    trap_turn_to_270();
                    trap_after_segment_common();
                }
            }
            break;

        case 8:
            Trapezoid_RunByTarget(-1, LongDao_X, 1, 4, TRAPEZOID_CSB_ENABLE);
            if ((-position_mm) >= LongDao_X)
            {
                if (dis[1] <= csb_X_near && dis[4] <= csb_X_near &&
                    dis[2] >= csb_X_far && dis[3] >= csb_X_far)
                {
                    count = 9;
                    Trapezoid_StopAndResetOdom();
                    trap_turn_to_0();
                    trap_after_segment_common();
                }
            }
            break;

        case 9:
            Trapezoid_RunByTarget(1, Long_Go_Long, 2, 3, TRAPEZOID_CSB_DISABLE);
            if (position_mm >= Long_Go_Long)
            {
                if (dis[2] >= csb_X_far && dis[3] >= csb_X_far)
                {
                    count = 10;
                    Trapezoid_StopAndResetOdom();
                    trap_turn_to_270();
                    trap_after_segment_common();
                }
            }
            break;

        case 10:
            Trapezoid_RunByTarget(1, LongDao_X, 2, 3, TRAPEZOID_CSB_ENABLE);
            if (position_mm >= LongDao_X)
            {
                if (dis[2] <= csb_X_near && dis[3] <= csb_X_near &&
                    dis[1] >= csb_X_far && dis[4] >= csb_X_far)
                {
                    count = 11;
                    Trapezoid_StopAndResetOdom();
                    Trapezoid_Reset();
                }
            }
            break;

        case 11:
            Trapezoid_RunByTarget(-1, LongDao_X, 1, 4, TRAPEZOID_CSB_ENABLE);
            if ((-position_mm) >= LongDao_X)
            {
                if (dis[1] <= csb_X_near && dis[4] <= csb_X_near &&
                    dis[2] >= csb_X_far && dis[3] >= csb_X_far)
                {
                    count = 12;
                    Trapezoid_StopAndResetOdom();
                    trap_turn_to_0();
                    trap_after_segment_common();
                }
            }
            break;

        case 12:
            Trapezoid_RunByTarget(-1, back_len, 1, 4, TRAPEZOID_CSB_DISABLE);
            if ((-position_mm) >= back_len)
            {
                Trapezoid_Stop();
            }
            break;

        default:
            Trapezoid_Stop();
            break;
    }
}
