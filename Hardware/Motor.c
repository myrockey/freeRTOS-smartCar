#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "Timer.h"

/**
  * 函    数：PWM初始化 电机
  * 参    数：无
  * 返 回 值：无
  */
void PWM_Init_Motor(void)
{
	/*开启时钟*/
	TB6612_GPIO_APBX(TB6612_GPIO_CLK, ENABLE);			//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = TB6612_GPIO_PIN_PWMA | TB6612_GPIO_PIN_PWMB;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TB6612_GPIO_PORT, &GPIO_InitStructure);							//将PA2引脚初始化为复用推挽输出	
																	//受外设控制的引脚，均需要配置为复用模式
	TIM_Motor();
}


/* TIM1 和 TIM2 各自的通道是独立的，能同时使用 通道1 */
/**
  * 函    数：PWM设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~100
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void PWM_SetCompare2_Motor(uint16_t Compare)
{
	TIM_SetCompare2(TB6612_TIM, Compare);		//设置CCR2的值
}

void PWM_SetCompare1_Motor(uint16_t Compare)
{
	TIM_SetCompare1(TB6612_TIM, Compare);		//设置CCR4的值
}

void Motor_Init(void)
{
	//开启RCC时钟 GPIOA
	TB6612_Motor_GPIO_APBX(TB6612_Motor_GPIO_CLK, ENABLE);
	
	//GPIOA初始化
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Pin = TB6612_GPIO_PIN_AIN1 | TB6612_GPIO_PIN_AIN2 | TB6612_GPIO_PIN_BIN1 | TB6612_GPIO_PIN_BIN2;//PA4 PA5 PA6 PA7 引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TB6612_Motor_GPIO_PORT,&GPIO_InitStructure);
	
	PWM_Init_Motor();//初始化直流电机的底层PWM
}

//左边电机
void MotorLeft_SetSpeed(int8_t Speed)
{
	if(Speed > 0)
	{
		//假设为正转
		GPIO_SetBits(TB6612_Motor_GPIO_PORT,TB6612_GPIO_PIN_AIN1);//置高电平
		GPIO_ResetBits(TB6612_Motor_GPIO_PORT,TB6612_GPIO_PIN_AIN2);//置低电平
		PWM_SetCompare2_Motor(Speed);
	}
	else if(Speed == 0)
	{
		//假设为正转
		GPIO_SetBits(TB6612_Motor_GPIO_PORT,TB6612_GPIO_PIN_AIN1);//置高电平
		GPIO_ResetBits(TB6612_Motor_GPIO_PORT,TB6612_GPIO_PIN_AIN2);//置低电平
		PWM_SetCompare2_Motor(Speed);
	}
	else
	{
		//反之则为反转
		GPIO_ResetBits(TB6612_Motor_GPIO_PORT,TB6612_GPIO_PIN_AIN1);//置低电平
		GPIO_SetBits(TB6612_Motor_GPIO_PORT,TB6612_GPIO_PIN_AIN2);//置高电平
		PWM_SetCompare2_Motor(-Speed);//PWM只能给正数
	}
}

//右边电机
void MotorRight_SetSpeed(int8_t Speed)
{
	if(Speed > 0)
	{
		//假设为正转
		GPIO_SetBits(TB6612_Motor_GPIO_PORT,TB6612_GPIO_PIN_BIN1);//置高电平
		GPIO_ResetBits(TB6612_Motor_GPIO_PORT,TB6612_GPIO_PIN_BIN2);//置低电平
		PWM_SetCompare1_Motor(Speed);
	}
	else if(Speed == 0)
	{
		//假设为正转
		GPIO_SetBits(TB6612_Motor_GPIO_PORT,TB6612_GPIO_PIN_BIN1);//置高电平
		GPIO_ResetBits(TB6612_Motor_GPIO_PORT,TB6612_GPIO_PIN_BIN2);//置低电平
		PWM_SetCompare1_Motor(Speed);
	}
	else
	{
		//反之则为反转
		GPIO_ResetBits(TB6612_Motor_GPIO_PORT,TB6612_GPIO_PIN_BIN1);//置低电平
		GPIO_SetBits(TB6612_Motor_GPIO_PORT,TB6612_GPIO_PIN_BIN2);//置高电平
		PWM_SetCompare1_Motor(-Speed);//PWM只能给正数
	}
}
