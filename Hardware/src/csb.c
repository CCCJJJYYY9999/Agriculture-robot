#include "csb.h"
#include "delay.h"

uint8_t rx_bufer1[4],rx_bufer2[4];
uint32_t csb_t[12];
uint16_t dis[8];	//后，左后，右后，右前，左前，前，内前，内后

void csb_Trig_H()
{
	HAL_GPIO_WritePin(T1_GPIO_Port,T1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(T2_GPIO_Port,T2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(T3_GPIO_Port,T3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(T4_GPIO_Port,T4_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(T5_GPIO_Port,T5_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(T6_GPIO_Port,T6_Pin,GPIO_PIN_SET);
}

void csb_Trig_L()
{
	HAL_GPIO_WritePin(T1_GPIO_Port,T1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(T2_GPIO_Port,T2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(T3_GPIO_Port,T3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(T4_GPIO_Port,T4_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(T5_GPIO_Port,T5_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(T6_GPIO_Port,T6_Pin,GPIO_PIN_RESET);
}

// 重写外部中断回调函数（无__weak，无返回值）
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_1)  // 判断是PE0触发
    {
			if(GPIOE->IDR & GPIO_PIN_1)
			{
				csb_t[0] = __HAL_TIM_GetCounter(&htim6);
			}				
			else{
				csb_t[1] = __HAL_TIM_GetCounter(&htim6);
				dis[0] = (csb_t[1] - csb_t[0]) * 34/100/2;
			}
    }
		else if(GPIO_Pin == GPIO_PIN_3)
		{
			if(GPIOE->IDR & GPIO_PIN_3)
			{
				csb_t[2] = __HAL_TIM_GetCounter(&htim6);
			}				
			else{
				csb_t[3] = __HAL_TIM_GetCounter(&htim6);		
				dis[1] = (csb_t[3] - csb_t[2]) * 34/100/2;			
			}			
		}
		else if(GPIO_Pin == GPIO_PIN_5)
		{
			if(GPIOE->IDR & GPIO_PIN_5)
			{
				csb_t[4] = __HAL_TIM_GetCounter(&htim6);
			}				
			else{
				csb_t[5] = __HAL_TIM_GetCounter(&htim6);		
				dis[2] = (csb_t[5] - csb_t[4]) * 34/100/2;		
			}			
		}		
		else if(GPIO_Pin == GPIO_PIN_10)
		{
			if(GPIOD->IDR & GPIO_PIN_10)
			{
				csb_t[6] = __HAL_TIM_GetCounter(&htim6);
			}				
			else{
				csb_t[7] = __HAL_TIM_GetCounter(&htim6);		
				dis[3] = (csb_t[7] - csb_t[6]) * 34/100/2;		
			}			
		}
		else if(GPIO_Pin == GPIO_PIN_12)
		{
			if(GPIOD->IDR & GPIO_PIN_12)
			{
				csb_t[8] = __HAL_TIM_GetCounter(&htim6);
			}				
			else{
				csb_t[9] = __HAL_TIM_GetCounter(&htim6);	
				dis[4] = (csb_t[9] - csb_t[8]) * 34/100/2;					
			}			
		}		
		else if(GPIO_Pin == GPIO_PIN_14)
		{
			if(GPIOD->IDR & GPIO_PIN_14)
			{
				csb_t[10] = __HAL_TIM_GetCounter(&htim6);
			}				
			else{
				csb_t[11] = __HAL_TIM_GetCounter(&htim6);	
				dis[5] = (csb_t[11] - csb_t[10]) * 34/100/2;					
			}			
		}		
			
}

void read_csb7()
{
		if(rx_bufer1[0] == 0xFF)
		{
			dis[6] = rx_bufer1[1] * 256 + rx_bufer1[2];
		}
		memset(rx_bufer1,0,sizeof(rx_bufer1));
}

void read_csb8()
{
		if(rx_bufer2[0] == 0xFF)
		{
			dis[7] = rx_bufer2[1] * 256 + rx_bufer2[2];
		}
		memset(rx_bufer2,0,sizeof(rx_bufer2));		
}
