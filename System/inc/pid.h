#ifndef __PID_H__
#define __PID_H__

#include "main.h"
#include "tim.h"
#include "gpio.h"


#define pulse 3163.0   //一圈的脉冲数=编码器脉冲线数*减速比*定时器计数倍数（11*30*4）  
#define C 20.096      //轮子周长21.35cm

#define Start_Y 1000 //启动1000mm
#define LongDao_X 2600 //跨垄长2600mm
#define Long_Go_Long 700 //从一条垄到下一条垄的距离800
#define back_len 3900 //退回到起点的距离

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float actual;
	float Target;
	float Error;
	float LastError;
	float Output;
	float Integral_err;
	float Agvy;

}pid;



typedef struct
{
	pid location;
	pid speed;
	int Pwm;
	int encoder;

}work;

typedef struct
{
	uint8_t flag;
	uint32_t count;

}mark;

extern work left,right;
extern mark stop;
extern pid turn;
extern float turn_PWM;
extern float base_left_speed;
extern float base_right_speed;
extern uint8_t turn_flag;

extern int left_location;
extern int right_location;

float Incremental_PI (pid *p);
float Position_PID (pid *p);
float Turn_PID(pid *p);
void PID_Init();
void PID_Reset(pid *p);
void set_Dmotor_speed(pid *control_l,pid *control_r,float l_v,float r_v);


#endif