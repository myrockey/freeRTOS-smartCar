#include "stm32f10x.h"                  // Device header
#include "SmartCar.h"
#include "Motor.h"

int8_t Speed = 50;
/**
  * 函    数：初始化
  * 参    数：无
  * 返 回 值：无
  */
void SmartCar_Init(void)
{
	Motor_Init();
}

//设置小车速度0-100
void Car_SetSpeed(uint8_t percent)
{
	Speed = percent;
}

void Car_SetLeftSpeed(int8_t speed)
{
	MotorLeft_SetSpeed(speed);
}

void Car_SetRightSpeed(int8_t speed)
{
	MotorRight_SetSpeed(speed);
}

//向前
void Move_Forward(void)
{
	MotorLeft_SetSpeed(Speed);
	MotorRight_SetSpeed(Speed);
}

//向后
void Move_Backward(void)
{
	MotorLeft_SetSpeed(-Speed);
	MotorRight_SetSpeed(-Speed);
}

//停止
void Car_Stop(void)
{
	Car_SetSpeed(50);
	MotorLeft_SetSpeed(0);
	MotorRight_SetSpeed(0);
}

//向左转
void Turn_Left(void)
{
	MotorLeft_SetSpeed(Speed-40);
	MotorRight_SetSpeed(Speed);
}

//向右转
void Turn_Right(void)
{
	MotorLeft_SetSpeed(Speed);
	MotorRight_SetSpeed(Speed-40);
}

//顺时针旋转
void Clockwise_Rotation(void)
{
	MotorLeft_SetSpeed(Speed);
	MotorRight_SetSpeed(-Speed);
}


//逆时针旋转
void CounterClockwise_Rotation(void)
{
	MotorLeft_SetSpeed(-Speed);
	MotorRight_SetSpeed(Speed);
}
