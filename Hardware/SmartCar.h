#ifndef __SMARTCAR_H
#define __SMARTCAR_H

void SmartCar_Init(void);
void Car_SetSpeed(uint8_t percent);//设置小车速度0-100
//向前
void Move_Forward(void);
//向后
void Move_Backward(void);
//停止
void Car_Stop(void);
//向左转
void Turn_Left(void);
//向右转
void Turn_Right(void);
//顺时针旋转
void Clockwise_Rotation(void);
//逆时针旋转
void CounterClockwise_Rotation(void);
#endif

