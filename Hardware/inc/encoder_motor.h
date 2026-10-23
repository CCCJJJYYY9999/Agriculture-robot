#ifndef __ENCODER_MOTOR_H__
#define __ENCODER_MOTOR_H__

#include "main.h"
#include "tim.h"
#include "gpio.h"



void Load(int moto_F,int moto_B);

void Limit(float *motoA,float *motoB);

int Encoder_Read(TIM_HandleTypeDef *htim);



#endif