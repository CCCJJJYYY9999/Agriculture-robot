#ifndef __JY931_H
#define __JY931_H

#include "main.h"
#include "usart.h"
#include "gpio.h" 

void jy931_ReceiveData(uint8_t RxData);
void IMU_Full_Init(void);
extern uint8_t g_usart3_receivedata;

extern float Roll,Pitch,Yaw;
extern float Ax,Ay,Az;

#endif
