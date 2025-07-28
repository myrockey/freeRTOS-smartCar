#include "stm32f10x.h"                  // Device header
#include "Servo.h"
#include "Timer.h"

/**
  * 函    数：PWM初始化 舵机
  * 参    数：无
  * 返 回 值：无
  */
void Servo_Init(void)
{
		/*开启时钟*/
	SG90_GPIO_APBX(SG90_GPIO_CLK, ENABLE);			//开启GPIOA的时钟

	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = SG90_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SG90_GPIO_PORT, &GPIO_InitStructure);							//将PA2引脚初始化为复用推挽输出	
																	//受外设控制的引脚，均需要配置为复用模式
}

/*
0度   500
180度 2500
*/
void Servo_SetAngle(float Angle)
{
	TIM_SetCompare4(SG90_TIM, Angle/180*2000+500);		//设置CCR4的值
}
