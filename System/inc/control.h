#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"

extern uint8_t count;
extern uint8_t key_flag;
extern uint8_t control_mode;
extern int center_dis;

void Main_Gyro_Close_Loop();
void Main_Control();
void Tim_Gyro_Close_Loop();
void Tim_Control();

#endif