#include "stm32f4xx.h"

void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 168 * xus;
	SysTick->VAL = 0x00;
	SysTick->CTRL = 0x00000005;
	while(!(SysTick->CTRL & 0x00010000));
	SysTick->CTRL = 0x00000004;
}

void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 

/**
 * @brief  168MHz 下 微秒级软件延时
 * @param  nus: 要延时的微秒数 1~23300
 */
void software_delay_us(uint32_t nus)
{
    uint32_t i;
    // 168MHz 1us = 42次循环
    for(i = 0; i < nus * 42; i++)
    {
        __NOP();
    }
}
