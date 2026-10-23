#ifndef __CSB_H__
#define __CSB_H__

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#define Y_FrontCsb_1 2900						//前面超声波在第一垄道的阈值
#define Y_FrontCsb_2 2200						//前面超声波在第二垄道的阈值
#define Y_FrontCsb_3 1500						//前面超声波在第三垄道的阈值
#define Y_FrontCsb_4 800						//前面超声波在第四垄道的阈值
#define Y_FrontCsb_5 100						//前面超声波在第五垄道的阈值
#define Y_BehindCsb_1 100						//后面超声波在第一垄道的阈值
#define Y_BehindCsb_2 800						//后面超声波在第二垄道的阈值
#define Y_BehindCsb_3 1500					//后面超声波在第三垄道的阈值
#define Y_BehindCsb_4 2200					//后面超声波在第四垄道的阈值
#define Y_BehindCsb_5 2900					//后面超声波在第五垄道的阈值
#define center_lengh 100						//车在跨垄时垄道与车的间距，用于舵轮内测超声波测距
#define X_RightCsb_L 2700						//右边超声波在左节点位置的阈值
#define X_RightCsb_R 200						//右边超声波在右节点位置的阈值
#define X_LeftCsb_L 200							//左边超声波在左节点位置的阈值
#define X_LeftCsb_R 2700						//左边超声波在右节点位置的阈值
#define Lun_Outside 100
#define csb_X_near 125										//车靠边的超声波阈值
#define csb_X_far 200										//车靠边时另一边超声波的阈值

extern uint16_t dis[8];
extern uint8_t rx_bufer1[4],rx_bufer2[4];

void csb_Trig_H();
void csb_Trig_L();
void csb_text(uint8_t flag);
void read_csb7();
void read_csb8();

#endif
