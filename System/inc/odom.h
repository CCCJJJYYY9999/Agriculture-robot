#ifndef __ODOM_H
#define __ODOM_H

#include <stdint.h>

void Odom_Init(void);
void Odom_UpdateDelta(int32_t delta_cnt);
void Odom_Reset(void);

int64_t Odom_GetPositionUm(void);
int32_t Odom_GetPositionMm(void);

uint64_t Odom_GetMileageUm(void);
uint32_t Odom_GetMileageMm(void);

int32_t Odom_GetSpeedMmS(void);
int64_t Odom_GetTotalCount(void);

extern uint32_t mileage_mm;
extern int32_t position_mm;
extern int32_t speed_mm_s;

#endif