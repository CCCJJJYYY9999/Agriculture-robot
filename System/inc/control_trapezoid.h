#ifndef __CONTROL_TRAPEZOID_H__
#define __CONTROL_TRAPEZOID_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Independent trapezoid speed module for the existing controller1 project.
 *
 * Integration goal:
 * 1. Do not change PID, encoder, odom, ultrasonic, stepper or timer modules.
 * 2. Keep count as the task state number.
 * 3. Replace step speed commands such as set_Dmotor_speed(..., 6, 6)
 *    with ramped target speed commands.
 *
 * Put this file in:  System/inc/control_trapezoid.h
 * Put .c file in:    System/src/control_trapezoid.c
 */

/* Motor target speed used by your original program. */
#ifndef TRAPEZOID_V_MAX
#define TRAPEZOID_V_MAX             (6.0f)
#endif

/* Minimum moving speed during the deceleration area. */
#ifndef TRAPEZOID_V_MIN
#define TRAPEZOID_V_MIN             (1.5f)
#endif

/* Speed-unit per second. 10 means 0 -> 6 takes about 0.6 s. */
#ifndef TRAPEZOID_ACC_PER_SEC
#define TRAPEZOID_ACC_PER_SEC       (10.0f)
#endif

/* Speed-unit per second when reducing speed. */
#ifndef TRAPEZOID_DEC_PER_SEC
#define TRAPEZOID_DEC_PER_SEC       (12.0f)
#endif

/* Start reducing speed when remaining odom distance is below this value. */
#ifndef TRAPEZOID_ODOM_DEC_MM
#define TRAPEZOID_ODOM_DEC_MM       (250)
#endif

/* Start reducing speed when front ultrasonic distance is below this value. */
#ifndef TRAPEZOID_CSB_SLOW_MM
#define TRAPEZOID_CSB_SLOW_MM       (250)
#endif

/* Existing parking threshold. csb_X_near in csb.h is also 145. */
#ifndef TRAPEZOID_CSB_STOP_MM
#define TRAPEZOID_CSB_STOP_MM       (145)
#endif

#define TRAPEZOID_CSB_DISABLE       (0u)
#define TRAPEZOID_CSB_ENABLE        (1u)

/* Reset only the ramp generator. It does not reset odom or PID. */
void Trapezoid_Reset(void);

/* Immediately set motor target speed to zero and clear ramp state. */
void Trapezoid_Stop(void);

/* Immediately stop and reset odometer. */
void Trapezoid_StopAndResetOdom(void);

/*
 * Ramped straight motion.
 * dir: +1 forward, -1 backward.
 * target_mm: absolute travel distance after the last Odom_Reset().
 * csb_a/csb_b: two front ultrasonic indexes used for deceleration.
 * use_csb: TRAPEZOID_CSB_ENABLE or TRAPEZOID_CSB_DISABLE.
 */
void Trapezoid_RunByTarget(int8_t dir,
                           int32_t target_mm,
                           uint8_t csb_a,
                           uint8_t csb_b,
                           uint8_t use_csb);

uint8_t Trapezoid_ArrivedByOdom(int8_t dir, int32_t target_mm);
uint8_t Trapezoid_ArrivedByCsb(uint8_t csb_a, uint8_t csb_b);

/* Full count-state route control with trapezoid acceleration/deceleration. */
void Main_Control_Trapezoid(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_TRAPEZOID_H__ */
