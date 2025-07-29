#ifndef __ULTRA_SONIC_H
#define __ULTRA_SONIC_H

#include "BitAction.h"

//超声波模块
#define Hcsr04_GPIO_APBX RCC_APB2PeriphClockCmd
#define Hcsr04_GPIO_CLK  RCC_APB2Periph_GPIOB
#define Hcsr04_GPIO      GPIOB
#define TRIG_Pin         GPIO_Pin_0
#define TRIG_Send        PBout(0)
#define Echo_Pin         GPIO_Pin_1        
#define ECHO_Recieve     PBin(1)

void Ultrasonic_Init(void);
float Ultrasonic_Distance(void);
void Ultrasonic_Run(uint16_t distance);

#endif


