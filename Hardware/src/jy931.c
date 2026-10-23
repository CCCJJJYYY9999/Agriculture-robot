#include "jy931.h"
#include "stdio.h"
static uint8_t RxBuffer[11];/*接收数据数组*/
static volatile uint8_t RxState = 0;/*接收状态标志位*/
static uint8_t RxIndex = 0;/*接受数组索引*/
float Roll,Pitch,Yaw;/*角度信息，如果只需要整数可以改为整数类型*/
uint8_t g_usart3_receivedata = 0;
float Ax,Ay,Az;

/**
 * @brief       数据包处理函数
 * @param       串口接收的数据RxData
 * @retval      无
 */
void jy931_ReceiveData(uint8_t RxData)
{
	uint8_t i,sum=0;
	
	if (RxState == 0)	//等待包头
	{
		if (RxData == 0x55)	//收到包头
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 1;
			RxIndex = 1; //进入下一状态
		}
	}
	
	else if (RxState == 1)
	{
		if (RxData == 0x53)	/*判断数据内容，修改这里可以改变要读的数据内容，0x53为角度输出*/
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 2;
			RxIndex = 2; //进入下一状态
		}
	}
	else if (RxState == 2)	//接收数据
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//接收完成
		{
			for(i=0;i<10;i++)
			{
				sum = sum + RxBuffer[i]; //计算校验和
			}
			if(sum == RxBuffer[10])		//校验成功
			{
				/*计算数据，根据数据内容选择对应的计算公式*/
				Roll = ((uint16_t) ((uint16_t) RxBuffer[3] << 8 | (uint16_t) RxBuffer[2])) / 32768.0f * 180.0f;
				Pitch = ((uint16_t) ((uint16_t) RxBuffer[5] << 8 | (uint16_t) RxBuffer[4])) / 32768.0f * 180.0f;
				Yaw = ((uint16_t) ((uint16_t) RxBuffer[7] << 8 | (uint16_t) RxBuffer[6])) / 32768.0f * 180.0f;
			}
			RxState = 0;
			RxIndex = 0; //读取完成，回到最初状态，等待包头
		}
	}
}

static void UART3_SendBytes(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart3, data, len, 1000);
}

void IMU_Full_Init(void)
{
    uint8_t unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    uint8_t save[]   = {0xFF, 0xAA, 0x00, 0x00, 0x00};

    /* ========== 1. 解锁 ========== */
    HAL_Delay(3500);			
    UART3_SendBytes(unlock, 5);			
    HAL_Delay(200);	
		printf("IMU Unlock\n");
    /* ========== 2. 设置6轴模式 ========== */
    uint8_t axis6_cmd[5] = {0xFF, 0xAA, 0x24, 0x01, 0x00};
    UART3_SendBytes(axis6_cmd, 5);
    HAL_Delay(100);
		printf("AXIS6\n");

    /* ========== 3. 加速度校准 ========== */
    uint8_t acc_calib[] = {0xFF, 0xAA, 0x01, 0x01, 0x00};
    uint8_t acc_exit[]  = {0xFF, 0xAA, 0x01, 0x00, 0x00};

    UART3_SendBytes(acc_calib, 5);
    HAL_Delay(4000);

    UART3_SendBytes(acc_exit, 5);
    HAL_Delay(100);
		printf("Calibration completed\n");

    /* ========== 4. 设置100Hz输出 ========== */
    uint8_t rate_cmd[5] = {0xFF, 0xAA, 0x03, 0x09, 0x00};
    UART3_SendBytes(rate_cmd, 5);
    HAL_Delay(100);
		printf("Output 100HZ\n");

    /* ========== 5. Z轴归零（Yaw清零） ========== */
    uint8_t z_zero[] = {0xFF, 0xAA, 0x01, 0x04, 0x00};

    UART3_SendBytes(z_zero, 5);
    HAL_Delay(3000);
		printf("Yaw Reset\n");

    /* ========== 6. 保存所有配置 ========== */
    UART3_SendBytes(save, 5);
}
