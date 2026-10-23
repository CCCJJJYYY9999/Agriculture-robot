#include "pid.h"
#include "encoder_motor.h"

work left,right;
mark stop;
pid turn;
float turn_PWM = 0;
float base_left_speed = 0;
float base_right_speed = 0;
uint8_t turn_flag = 0;



void PID_Init()
{
	turn.Kp = 0.2;
	turn.Kd = 0;

	left.speed.Kp = 8;
	left.speed.Ki = 18;
	left.location.Kp = 0;
	left.location.Ki = 0;
	left.location.Kd = 0;
	
	right.speed.Kp = 8;
	right.speed.Ki = 18;
	right.location.Kp = 0;
	right.location.Ki = 0;
	right.location.Kd = 0;
}

/**************************************************************************
函数功能：增量PI控制器
入口参数：speed结构体(编码器测量值，目标速度)
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
float Incremental_PI (pid *p)
{
	p->LastError = p->Error;	                   //保存上一次偏差 
	p->Error=p->Target-p->actual;                //计算偏差
	p->Output += p->Kp*(p->Error-p->LastError)+p->Ki*p->Error;   //增量式PI控制器
	if(p->Output>900)	p->Output = 900;
	else if(p->Output<-900) p->Output = -900;
		
	return p->Output;                         //增量输出
	
}


/**************************************************************************
函数功能：位置式PID控制器
入口参数：编码器测量位置信息，目标位置
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
float Position_PID (pid *p)
{ 	
	p->Error = p->Target - p->actual;

	p->Integral_err += p->Error;

	if(p->Integral_err > 1000.0f)
		p->Integral_err = 1000.0f;
	else if(p->Integral_err < -1000.0f)
		p->Integral_err = -1000.0f;

	p->Output = p->Kp * p->Error
	          + p->Ki * p->Integral_err
	          + p->Kd * (p->Error - p->LastError);

	p->LastError = p->Error;

	return p->Output;
}

//转向环PD控制器
//输入：角速度、角度值,
static float Angle_Normalize(float angle)
{
	while(angle > 180.0f)
		angle -= 360.0f;

	while(angle < -180.0f)
		angle += 360.0f;

	return angle;
}

void PID_Reset(pid *p)
{
	p->Error = 0;
	p->LastError = 0;
	p->Integral_err = 0;
	p->Output = 0;
}

float Turn_PID(pid *p)
{
	p->LastError = p->Error;

	p->Error = Angle_Normalize(p->Target - p->actual);

	p->Output = p->Kp * p->Error
	          + p->Kd * (p->Error - p->LastError);

	if(p->Output > 3.0f)
		p->Output = 3.0f;
	else if(p->Output < -3.0f)
		p->Output = -3.0f;

	return p->Output;
}

void set_Dmotor_speed(pid *control_l,pid *control_r,float l_v,float r_v)
{
	base_left_speed = l_v;
	base_right_speed = r_v;

	control_l->Target = l_v;
	control_r->Target = r_v;
}

