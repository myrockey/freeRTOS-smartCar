#ifndef __TIM_H
#define __TIM_H

// 配置舵机 SG90
#define SG90_TIM_APBX       RCC_APB2PeriphClockCmd
#define SG90_TIM_CLK        RCC_APB2Periph_TIM1
#define SG90_TIM    	    TIM1		                /* 定时器1 */

// 红外遥控 hx1838
#define IR_TIM    	      TIM1		                /* 定时器1 */
#define IR_TIM_UPDATE_IRQn     TIM1_UP_IRQn  /* TIM更新中断 */
#define IR_TIM_UPDATE_IRQHandler  TIM1_UP_IRQHandler
#define IR_TIM_CC_IRQn     TIM1_CC_IRQn  /* TIM更新中断 */
#define IR_TIM_CC_IRQHandler  TIM1_CC_IRQHandler

//电机 tb6612
#define TB6612_TIM_APBX       RCC_APB1PeriphClockCmd
#define TB6612_TIM_CLK        RCC_APB1Periph_TIM2
#define TB6612_TIM    	      TIM2		                /* 定时器2 */

//超声波
#define Ultrasonic_TIM_APBX 	  RCC_APB1PeriphClockCmd
#define Ultrasonic_TIM_CLK 	      RCC_APB1Periph_TIM3
#define Ultrasonic_TIM 	          TIM3
#define Ultrasonic_TIM_IRQn       TIM3_IRQn
#define Ultrasonic_TIM_IRQHandler TIM3_IRQHandler

//WIFI
#define WIFI_TIM_APBX 	    RCC_APB1PeriphClockCmd
#define WIFI_TIM_CLK 	    RCC_APB1Periph_TIM4
#define WIFI_TIM 	        TIM4
#define WIFI_TIM_IRQn       TIM4_IRQn
#define WIFI_TIM_IRQHandler TIM4_IRQHandler

void TIM1_Init(void);
void TIM_Motor(void);
void TIM_Ultrasonic(void);
void TIM_WIFI_ENABLE_30S(void);
void TIM_WIFI_ENABLE_2S(void);

#endif

