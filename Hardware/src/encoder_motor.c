#include "encoder_motor.h"
#include <math.h>
#include <stdlib.h>

#define PWM_MAX 900
#define PWM_MIN -900


int Encoder_Read(TIM_HandleTypeDef *htim)
{
	int16_t temp;
	temp = (short)__HAL_TIM_GetCounter(htim);
	__HAL_TIM_SetCounter(htim,0);
	return temp;
}

//控制电机转向和速度      int moto_F,int moto_B目前是范围是（0——1000）
void Load(int moto_F,int moto_B)
{
	if(moto_B>0)
	{
		HAL_GPIO_WritePin(GPIOB,IO2_Pin,GPIO_PIN_SET);//左电机IN1
	}
	else{
		HAL_GPIO_WritePin(GPIOB,IO2_Pin,GPIO_PIN_RESET);//左电机IN1	
	}
	if(moto_F>0)
	{
		HAL_GPIO_WritePin(GPIOB,IO1_Pin,GPIO_PIN_RESET);//左电机IN1
	}
	else{
		HAL_GPIO_WritePin(GPIOB,IO1_Pin,GPIO_PIN_SET);//左电机IN1	
	}	
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,abs(moto_F));//设置PWM占空比调速
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,abs(moto_B));	
}

//PWM占空比调节限幅
void Limit(float *motoA,float *motoB)
{
	if(*motoA > PWM_MAX) *motoA = PWM_MAX;
	if(*motoA < PWM_MIN) *motoA = PWM_MIN;	
	if(*motoB > PWM_MAX) *motoB = PWM_MAX;	
	if(*motoB < PWM_MIN) *motoB = PWM_MIN;	
	
}
