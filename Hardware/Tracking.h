#ifndef __TRACKING_H
#define __TRACKING_H

#include "stm32f10x.h"                  // Device header

//三路循迹
#define   GPIO_PIN_R   GPIO_Pin_12
#define   GPIO_PIN_M   GPIO_Pin_13
#define   GPIO_PIN_L   GPIO_Pin_14
#define   R   GPIO_ReadInputDataBit(GPIOB, GPIO_PIN_R)
#define   M   GPIO_ReadInputDataBit(GPIOB, GPIO_PIN_M) 
#define   L   GPIO_ReadInputDataBit(GPIOB, GPIO_PIN_L) 

void Tracking_Init(void);
void Tracking_Run(void);

#endif

