#include "main.h"
#include "control.h"
#include "control_trapezoid.h"

uint8_t count = 0;
uint8_t key_flag = 0;
uint8_t control_mode = 0;
int center_dis = 0;
uint8_t center_flag = 0;
uint8_t go_flag = 0;
int8_t center_angle = 0;
int8_t go_angle = 0;
int8_t m1_angle = 0;
int8_t m2_angle = 0;

void Main_Gyro_Close_Loop()		//陀螺仪闭环测试
{
	vofa_draw_graphical(4);	
	
	switch(count)			
	{
		case 0:
			set_Dmotor_speed(&left.speed,&right.speed,6,6);
			count = 1;
			break;
		case 1:
			if(position_mm >= Start_Y)
			{
				printf("a1");
				if(dis[3] >= csb_X_far && dis[2] >= csb_X_far )
				{
					set_Dmotor_speed(&left.speed,&right.speed,0,0);		//到达第一垄道左节点						
					Odom_Reset();
					StepperMotorset_pos(&huart5, &motor_l, 1, 100 , 270);
					StepperMotorset_pos(&huart6, &motor_r, 1, 100 , 270);			
					Delay_ms(1000);
					set_Dmotor_speed(&left.speed,&right.speed,6,6);
					count = 2;
				}					
			}
			break;		
		case 2:
			if(position_mm >= LongDao_X)
			{
				if(dis[1] >= csb_X_far && dis[4] >= csb_X_far && dis[3] <= csb_X_near && dis[2] <= csb_X_near )
				{
					Odom_Reset();
					set_Dmotor_speed(&left.speed,&right.speed,-6,-6);	
					count = 3;
				}
			}
			break;
		case 3:
			if((-position_mm) >= LongDao_X)
			{
				if(dis[1] <= csb_X_near && dis[4] <= csb_X_near && dis[3] >= csb_X_far && dis[2] >= csb_X_far )
				{
					Odom_Reset();
					set_Dmotor_speed(&left.speed,&right.speed,6,6);		
					count = 2;
				}
			}
			break;
		default:
			set_Dmotor_speed(&left.speed,&right.speed,0,0);		
			break;
	}
}

//dis[0],dis[1],dis[2],dis[3],dis[4],dis[5]);	0后，1左后，2右后，3右前，4左前，5前，6内前，7内后	
void Main_Control()			//跑全图控制
{
	switch(count)
	{
		case 0:
//				if(key_flag)
//				{
				set_Dmotor_speed(&left.speed,&right.speed,6,6);		//启动前进
				
				count = 1;
//				}
			break;
		
		case 1:
			if(position_mm >= Start_Y)
			{
				if(dis[3] >= csb_X_far && dis[2] >= csb_X_far )
				{
					count = 2;	
					
					set_Dmotor_speed(&left.speed,&right.speed,0,0);		//到达第一垄道左节点						
					Odom_Reset();
					
					StepperMotorset_pos(&huart5, &motor_l, 1, 100 , 270);
					StepperMotorset_pos(&huart6, &motor_r, 1, 100 , 270);			
					HAL_Delay(1000);
					
					set_Dmotor_speed(&left.speed,&right.speed,6,6);					
				}					
			}
			break;
		case 2:
			if(position_mm>=LongDao_X)
			{
				if(dis[3] <= csb_X_near && dis[2] <= csb_X_near && dis[1] >= csb_X_far && dis[4] >= csb_X_far)
				{
					count = 3;		
					
					set_Dmotor_speed(&left.speed,&right.speed,0,0);		//到达第一垄道右节点							
					Odom_Reset();
				
					StepperMotorset_pos(&huart5, &motor_l, 0, 100 , 0);
					StepperMotorset_pos(&huart6, &motor_r, 0, 100 , 0);
					HAL_Delay(1000);
					
					set_Dmotor_speed(&left.speed,&right.speed,6,6);						
				}				
			}
			break;
		case 3:
			if(position_mm>=Long_Go_Long)
			{
				if(dis[1] >= csb_X_far && dis[4] >= csb_X_far)
				{
					count = 4;		
					
					set_Dmotor_speed(&left.speed,&right.speed,0,0);		//到达第二垄道右节点						
					Odom_Reset();

					StepperMotorset_pos(&huart5, &motor_l, 1, 100 , 270);
					StepperMotorset_pos(&huart6, &motor_r, 1, 100 , 270);
					HAL_Delay(1000);

					set_Dmotor_speed(&left.speed,&right.speed,-6,-6);
				}
			}
			break;
		case 4:
			if((-position_mm) >= LongDao_X)		//注意，位移是负的，所以加负号
			{
				if(dis[1] <= csb_X_near && dis[4] <= csb_X_near && dis[2] >= csb_X_far && dis[3] >= csb_X_far)
				{
					count = 5;	
					
					set_Dmotor_speed(&left.speed,&right.speed,0,0);		//到达第二垄道左节点						
					Odom_Reset();

					StepperMotorset_pos(&huart5, &motor_l, 0, 100 , 0);
					StepperMotorset_pos(&huart6, &motor_r, 0, 100 , 0);
					HAL_Delay(1000);

					set_Dmotor_speed(&left.speed,&right.speed,6,6);
				}
			}
			break;
		case 5:
			if(position_mm >= Long_Go_Long)
			{
				if(dis[2] >= csb_X_far && dis[3] >= csb_X_far)
				{
					count = 6;
					
					set_Dmotor_speed(&left.speed,&right.speed,0,0);		//到达第三垄道左节点
					Odom_Reset();		
					
					StepperMotorset_pos(&huart5, &motor_l, 1, 100 , 270);
					StepperMotorset_pos(&huart6, &motor_r, 1, 100 , 270);
					HAL_Delay(1000);
					
					set_Dmotor_speed(&left.speed,&right.speed,6,6);			
				}
			}
			break;
		case 6:
			if(position_mm>=LongDao_X)
			{
				if(dis[2] <= csb_X_near && dis[3] <= csb_X_near && dis[1] >= csb_X_far && dis[4] >= csb_X_far)
				{
					count = 7;
					
					set_Dmotor_speed(&left.speed,&right.speed,0,0);		//到达第三垄道右节点							
					Odom_Reset();
				
					StepperMotorset_pos(&huart5, &motor_l, 0, 100 , 0);
					StepperMotorset_pos(&huart6, &motor_r, 0, 100 , 0);
					HAL_Delay(1000);
					
					set_Dmotor_speed(&left.speed,&right.speed,6,6);						
				}				
			}
			break;
		case 7:
			if(position_mm>=Long_Go_Long)
			{
				if(dis[1] >= csb_X_far && dis[4] >= csb_X_far)
				{
					count = 8;
					
					set_Dmotor_speed(&left.speed,&right.speed,0,0);		//到达第四垄道右节点						
					Odom_Reset();

					StepperMotorset_pos(&huart5, &motor_l, 1, 100 , 270);
					StepperMotorset_pos(&huart6, &motor_r, 1, 100 , 270);
					HAL_Delay(1000);

					set_Dmotor_speed(&left.speed,&right.speed,-6,-6);
				}
			}
			break;
		case 8:
			if((-position_mm) >= LongDao_X)		//注意，位移是负的，所以加负号
			{
				if(dis[1] <= csb_X_near && dis[4] <= csb_X_near && dis[2] >= csb_X_far && dis[3] >= csb_X_far)
				{
					count = 9;	
					
					set_Dmotor_speed(&left.speed,&right.speed,0,0);		//到达第四垄道左节点
					Odom_Reset();

					StepperMotorset_pos(&huart5, &motor_l, 0, 100 , 0);
					StepperMotorset_pos(&huart6, &motor_r, 0, 100 , 0);
					HAL_Delay(1000);

					set_Dmotor_speed(&left.speed,&right.speed,6,6);
				}
			}
			break;
		case 9:
			if(position_mm >= Long_Go_Long)
			{
				if(dis[2] >= csb_X_far && dis[3] >= csb_X_far)
				{
					count = 10;			
					
					set_Dmotor_speed(&left.speed,&right.speed,0,0);		//到达第五垄道左节点
					Odom_Reset();		
					
					StepperMotorset_pos(&huart5, &motor_l, 1, 100 , 270);
					StepperMotorset_pos(&huart6, &motor_r, 1, 100 , 270);
					HAL_Delay(1000);
					
					set_Dmotor_speed(&left.speed,&right.speed,6,6);
				}
			}
			break;
		case 10:
			if(position_mm>=LongDao_X)
			{
				if(dis[2] <= csb_X_near && dis[3] <= csb_X_near && dis[1] >= csb_X_far && dis[4] >= csb_X_far)
				{
					count = 11;
					
					set_Dmotor_speed(&left.speed,&right.speed,0,0);		//到达第五垄道右节点，接着往回走							
					Odom_Reset();
					set_Dmotor_speed(&left.speed,&right.speed,-6,-6);						
				}
			}
			break;
		case 11:
			if((-position_mm) >= LongDao_X)		//注意，位移是负的，所以加负号
			{
				if(dis[1] <= csb_X_near && dis[4] <= csb_X_near && dis[2] >= csb_X_far && dis[3] >= csb_X_far)
				{
					count = 12;				
					set_Dmotor_speed(&left.speed,&right.speed,0,0);		//退回到达第五垄道左节点
					Odom_Reset();

					StepperMotorset_pos(&huart5, &motor_l, 0, 100 , 0);
					StepperMotorset_pos(&huart6, &motor_r, 0, 100 , 0);
					HAL_Delay(1000);

					set_Dmotor_speed(&left.speed,&right.speed,-6,-6);
				}
			}
			break;
		case 12:
			if((-position_mm) >= back_len)
			{
				set_Dmotor_speed(&left.speed,&right.speed,0,0);		//退回到起点
			}
			break;
		default:
				set_Dmotor_speed(&left.speed,&right.speed,0,0);		//识别到蛋，停止准备抓取
			break;
	}
}


void Tim_Gyro_Close_Loop()		//陀螺仪测试
{
	
	if(count == 0 || count == 1)
	{
		control_mode = 1;
	}
	else if(count == 2)
	{
		control_mode = 2;
		center_dis = dis[6] - dis[7];
		if(center_dis >= 20)
		{
			StepperMotorset_pos_rel(&huart5,&motor_l, 1, 50, 1);
			StepperMotorset_pos_rel(&huart6,&motor_r, 1, 50, 1);				
		}
		else if(center_dis <= -20)
		{
			StepperMotorset_pos_rel(&huart5,&motor_l, 0, 50, 1);
			StepperMotorset_pos_rel(&huart6,&motor_r, 0, 50, 1);				
		}
		else if(center_dis <= 15 && center_dis >= -15)
		{
			StepperMotorset_pos(&huart5, &motor_l, 1, 50 , 270);
			StepperMotorset_pos(&huart6, &motor_r, 1, 50 , 270);
		}			
	}
	else if(count == 3)
	{
		control_mode = 2;
		center_dis = dis[6] - dis[7];
		if(center_dis >= 20)
		{
			StepperMotorset_pos_rel(&huart5,&motor_l, 0, 50, 1);
			StepperMotorset_pos_rel(&huart6,&motor_r, 0, 50, 1);				
		}
		else if(center_dis <= -20)
		{
			StepperMotorset_pos_rel(&huart5,&motor_l, 1, 50, 1);
			StepperMotorset_pos_rel(&huart6,&motor_r, 1, 50, 1);				
		}
		else if(center_dis <= 15 && center_dis >= -15)
		{
			StepperMotorset_pos(&huart5, &motor_l, 1, 50 , 270);
			StepperMotorset_pos(&huart6, &motor_r, 1, 50 , 270);
		}
	}
}

void Tim_Control()			//跑全图的中断控制
{
	if(count == 0 || count == 1 || count == 3 || count == 5 || count == 7 || count == 9 )
	{
		control_mode = 1;
	}
	else if(count == 2 || count == 6 || count == 10)
	{
		control_mode = 2;
		center_dis = dis[6] - dis[7];
		if(center_dis >= 20)
		{
			StepperMotorset_pos_rel(&huart5,&motor_l, 1, 50, 1);
			StepperMotorset_pos_rel(&huart6,&motor_r, 1, 50, 1);				
		}
		else if(center_dis <= -20)
		{
			StepperMotorset_pos_rel(&huart5,&motor_l, 0, 50, 1);
			StepperMotorset_pos_rel(&huart6,&motor_r, 0, 50, 1);				
		}
		else if(center_dis <= 15 && center_dis >= -15)
		{
			StepperMotorset_pos(&huart5, &motor_l, 1, 50 , 270);
			StepperMotorset_pos(&huart6, &motor_r, 1, 50 , 270);
		}
	}
	else if(count == 4 || count == 8 || count == 11)
	{
		control_mode = 2;
		center_dis = dis[6] - dis[7];
		if(center_dis >= 20)
		{
			StepperMotorset_pos_rel(&huart5,&motor_l, 0, 50, 1);
			StepperMotorset_pos_rel(&huart6,&motor_r, 0, 50, 1);				
		}
		else if(center_dis <= -20)
		{
			StepperMotorset_pos_rel(&huart5,&motor_l, 1, 50, 1);
			StepperMotorset_pos_rel(&huart6,&motor_r, 1, 50, 1);				
		}
		else if(center_dis <= 15 && center_dis >= -15)
		{
			StepperMotorset_pos(&huart5, &motor_l, 1, 50 , 270);
			StepperMotorset_pos(&huart6, &motor_r, 1, 50 , 270);
		}		
	}
	else if(count == 12)
	{
		control_mode = 1;		
		center_flag = 0;
		go_flag = 0;
		
		if((dis[1]+dis[4])/2 >= 85 )
		{
			
			center_flag = 1;
			center_angle += 1;

		}
		else if((dis[1]+dis[4])/2 <= 65)
		{
			center_flag = 1;
			center_angle -= 1;			
		}

		if((dis[1]-dis[4])>=2)
		{
			go_flag = 1;
			go_angle += 1;		
		}
		else if((dis[1] - dis[4])<= -2)
		{
			go_flag = 1;
			go_angle -=1;
		}
		
		m1_angle = go_angle + center_angle;
		m2_angle = center_angle - center_angle;
		printf("%d,%d,%d,%d\r\n",m1_angle,m2_angle,center_flag,go_flag);
		
		go_angle = 0;
		center_angle = 0;
		
		if(m1_angle>=0)
		{
			StepperMotorset_pos_rel(&huart5,&motor_l, 1, 50, m1_angle);			
		}
		else if(m1_angle<0)
		{
			StepperMotorset_pos_rel(&huart5,&motor_l, 0, 50, -m1_angle);				
		}
		
		if(m2_angle>=0)
		{
			StepperMotorset_pos_rel(&huart6,&motor_r, 1, 50, m2_angle);			
		}
		else if(m2_angle<0)
		{
			StepperMotorset_pos_rel(&huart6,&motor_r, 0, 50, -m2_angle);				
		}
		
		if(center_flag == 0)
		{
			StepperMotorset_pos(&huart5, &motor_l, 1, 50 , 0);
			StepperMotorset_pos(&huart6, &motor_r, 1, 50 , 0);		
		}

	}
}
