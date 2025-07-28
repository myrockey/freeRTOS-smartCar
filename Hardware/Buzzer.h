#ifndef __Buzzer_H
#define __Buzzer_H

#include "stm32f10x.h"                  // Device header

#define Buzzer_GPIO_APBX 	    RCC_APB2PeriphClockCmd
#define Buzzer_GPIO_CLK 	    RCC_APB2Periph_GPIOC		/* GPIO端口时钟 */
#define Buzzer_GPIO_PORT    	GPIOC		                /* GPIO端口 */
#define Buzzer_GPIO_PIN	        GPIO_Pin_13

//低电平触发有源蜂鸣器
#define Buzzer_ON             GPIO_ResetBits(Buzzer_GPIO_PORT,Buzzer_GPIO_PIN)
#define Buzzer_OFF			  GPIO_SetBits(Buzzer_GPIO_PORT,Buzzer_GPIO_PIN)

void Buzzer_Init(void);

#endif

