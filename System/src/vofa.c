/*******************************************************************************
 * @file        VOFA+的串口收发
 * @author      eternal_fu
 * @version     V0.0.1
 * @brief       
 * @Date 2025-5-11 18:12:50
 * @LastEditTime 2024-12-11 04:28:17
 * @**************************************************************************** 
 * @attention   修改说明
 * @    版本号      日期                作者      说明
 *      0.0.1       2025年5月11日      fu      仅支持FireWater数据引擎、收发数据
 *******************************************************************************/
#include "vofa.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // 包含字符串操作函数的头文件
#include "main.h"



/* VOFA命令处理缓冲区 */

uint8_t RxBuffer_vofa[1];//串口接收缓冲
uint16_t RxLine;//指令长度
uint8_t DataBuff[200];//指令内容

extern UART_HandleTypeDef huart2;
/*
注意去串口重定向，并勾选c语言扩展库
*/

/* 声明传输变量 */

/*******************************************************************************
 * @brief   通过串口向VOFA+发送数据，输出波形
 * @return {*}
 * @note    none
 * @param {uint8_t} _dataflag   当前打印的数据类别[标志位]
 *******************************************************************************/
void vofa_draw_graphical(uint8_t _dataflag)
{
/*
    FireWater协议格式:
        "<any>:ch0,ch1,ch2,...,chN\n"
        - any和冒号可以为空，但换行(\n)不可省略；
        - any不可以为"image"，这个前缀用于解析图片数据
	；
        - 此处\n为换行，并非指字符斜杠+字符n；
        - \n也可以为\n\r，或\r\n。
    案例:
        VOFA_printf("%.0f,"		
					"%.0f,"
					"%.4f,"
					"%.4f,"
                    "%.4f\r\n"
					,dis_pid.target, dis_pid.current, dis_pid.kp, dis_pid.kd,  dis_pid.out);
        发送4个曲线:
        VOFA_printf("channels: 1.386578,0.977929,-0.628913,-0.942729\n");
        VOFA_printf("1.386578,0.977929,-0.628913,-0.942729\n")
*/

	/* 可以写一个枚举，通过枚举选择打印指定数据 */
   switch(_dataflag)
	{
       /* 左轮-位置环 */
		case 0:
					VOFA_printf("%.2f,%.2f,%.2f,%.2f,%5d\r\n", 
					left.location.Kp,left.location.Kd,left.location.Target,left.location.actual,left.Pwm);
			break;
		/* 右轮-位置环 */
		case 1:
					VOFA_printf("%.2f,%.2f,%.2f,%.2f,%5d\r\n", 
					right.location.Kp,right.location.Kd,right.location.Target,right.location.actual,right.Pwm);
			break;
		/* 左轮-速度环 */
		case 2:
					VOFA_printf("%f,%.5f,%.2f,%.2f\r\n",
					left.speed.Kp, left.speed.Ki,left.speed.Target,left.speed.actual);
			break;
		/* 右轮-速度环 */
		case 3:
					VOFA_printf("%f,%.5f,%.2f,%.2f\r\n",
					right.speed.Kp, right.speed.Ki,right.speed.Target,right.speed.actual);
			break;	
		/* 转向环 */	
		case 4:
				VOFA_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
				turn.Kp, turn.Kd, Yaw, turn.Error, turn.Output);
			break;		
		//测试对象
//		case 5:
//					VOFA_printf("%d,%d,%d\r\n",
//					TargetNum,RoomNum,direction);
//			break;					
/*		 左轮-速度环 
		case PID_LEFT_MOTOR:
            VOFA_printf("%.2f,%.2f,%.2f,%2f\r\n", car_pid_kp, turn_pid_kp, now_speed, aim_speed);
			now_speed+=0.02;
			break;*/
    default:
			break;
	}
	
#if 0 /* 这里仅作参考(可删除) */
    switch(_dataflag)
	{
		case _left_motor_pid:
			//左轮速度环：【发送和接收】
			VOFA_printf("%.2f,"		
						"%.2f,"
						"%.2f,"
						"%.2f\r\n"
						,left_Motor.pid.target, left_Motor.pid.current, left_Motor.pid.kp, left_Motor.pid.ki);
			break;
		case _right_motor_pid:
			//右轮速度环：【发送和接收】
			VOFA_printf("%f,"
						"%f,"
						"%f,"
						"%f\r\n"
						,right_Motor.pid.target, right_Motor.pid.current, right_Motor.pid.kp, right_Motor.pid.ki);
			break;
		case _turn_pid:
			//巡线转向环(摄像头)：
			VOFA_printf("%.2f,"		
						"%.2f,"
						"%.2f\r\n"
						,turn_pid.target, turn_pid.current, turn_pid.kp);
			break;
		case _angle_pid:
			//陀螺仪转向环：
			VOFA_printf("%.0f,"		
						"%.0f,"
						"%.2f,"
						"%.0f\r\n"
						,angle_pid.target, angle_pid.current, angle_pid.kp, angle_pid.kd);
			break;
		case _track_pid:
			//巡线（灰度）转向环：
			VOFA_printf("%.0f,"		
						"%.0f,"
						"%.5f,"
						"%.0f\r\n"
						,track_pid.target, track_pid.current, track_pid.kp, track_pid.kd);
			break;
        case _dis_pid:
			//计步环：
			VOFA_printf("%.0f,"		
						"%.0f,"
						"%.4f,"
						"%.4f,"
                        "%.4f\r\n"
						,dis_pid.target, dis_pid.current, dis_pid.kp, dis_pid.kd,  dis_pid.out);
			break;
	}
#endif
}


/*
 * 解析出DataBuff中的数据
 * 返回解析得到的数据
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0; // 记录数据位结束的地方
    uint8_t data_Num = 0; // 记录数据位数
    uint8_t minus_Flag = 0; // 判断是不是负数
    float data_return = 0; // 解析得到的数据
    for(uint8_t i=0;i<200;i++) // 查找等号和感叹号的位置
    {
        if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if(DataBuff[i] == 0x0A)
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(DataBuff[data_Start_Num] == '-') // 如果是负数
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1; // 负数flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;
		if(DataBuff[data_Start_Num+1] == 46)
		{
			if(data_Num == 4) // 数据共4位
			{
					data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
									(DataBuff[data_Start_Num+3]-48)*0.01f;
			}
			else if(data_Num == 5) // 数据共5位
			{
					data_return = (DataBuff[data_Start_Num]-48) + (DataBuff[data_Start_Num+2]-48)*0.1f + (DataBuff[data_Start_Num+3]-48)*0.01f +
									(DataBuff[data_Start_Num+4]-48)*0.001f;
			}
			else if(data_Num == 6) // 数据共6位
			{
					data_return = (DataBuff[data_Start_Num]-48) + (DataBuff[data_Start_Num+2]-48)*0.1f + (DataBuff[data_Start_Num+3]-48)*0.01f +
									(DataBuff[data_Start_Num+4]-48)*0.001f + (DataBuff[data_Start_Num+5]-48)*0.0001f;
			}
			if(minus_Flag == 1)  data_return = -data_return;
	//    printf("data=%.2f\r\n",data_return);
			return data_return;
			
		}
		else if(DataBuff[data_Start_Num+1] != 46)
		{
			if(data_Num == 4) // 数据共4位
			{
					data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
									(DataBuff[data_Start_Num+3]-48)*0.01f;
			}
			else if(data_Num == 5) // 数据共5位
			{
					data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + (DataBuff[data_Start_Num+3]-48)*0.1f +
									(DataBuff[data_Start_Num+4]-48)*0.01f;
			}
			else if(data_Num == 6) // 数据共6位
			{
					data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + (DataBuff[data_Start_Num+2]-48) +
									(DataBuff[data_Start_Num+4]-48)*0.1f + (DataBuff[data_Start_Num+5]-48)*0.01f;
			}
			else if(data_Num == 7) // 数据共6位
			{
					data_return = (DataBuff[data_Start_Num]-48)*1000 + (DataBuff[data_Start_Num+1]-48)*100 + (DataBuff[data_Start_Num+2]-48)*10 +
									(DataBuff[data_Start_Num+3]-48) + (DataBuff[data_Start_Num+5]-48)*0.1f + (DataBuff[data_Start_Num+6]-48)*0.01f;
			}
			if(minus_Flag == 1)  data_return = -data_return;
	//    printf("data=%.2f\r\n",data_return);
			return data_return;
		}
		
}

/*
 * 根据串口信息进行PID调参
 */
void USART_PID_Adjust(uint8_t Motor_n)
{
    float data_Get = Get_Data(); // 存放接收到的数据
//    printf("data=%.2f\r\n",data_Get);
    if(Motor_n == 0)//左边电机
    {
        if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P1
            left.location.Kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I1
            left.location.Ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D1
            left.location.Kd = data_Get;
        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P2
            left.speed.Kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I2
            left.speed.Ki = data_Get;
        else if(DataBuff[0]=='S' && DataBuff[1]=='1') //目标速度S1
            left.speed.Target = data_Get;
        else if(DataBuff[0]=='T' && DataBuff[1]=='1') //目标位置T1
            left.location.Target= data_Get;
    }
    else if(Motor_n == 1) // 右边电机
    {
        if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
            right.location.Kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
            right.location.Ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
            right.location.Kd = data_Get;
        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
            right.speed.Kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
            right.speed.Ki = data_Get;
        else if(DataBuff[0]=='S' && DataBuff[1]=='1') //目标速度
            right.speed.Target = data_Get;
        else if(DataBuff[0]=='T' && DataBuff[1]=='1') //目标位置
            right.location.Target = data_Get;
    }
    else if(Motor_n == 3) // 转向
		{
			if(DataBuff[0]=='P' && DataBuff[1]=='3') // 角度环P3
					turn.Kp = data_Get;			
			else if(DataBuff[0]=='D' && DataBuff[1]=='3') // 角度环D3
					turn.Kd = data_Get;		
      else if(DataBuff[0]=='A' && DataBuff[1]=='1') //目标角度A1
          turn.Target= data_Get;					
		}
}


/*******************************************************************************
 * @brief   将接收到的命令，直接写入FLASH空间，掉电不丢失
 *          （提前在FLASH空间开辟了参数空间，实现了内部FLASH的API）
 * @return {*}
 * @note    none
 *******************************************************************************/
static void vofa_set_flash_data(float _data)
{
    
}



