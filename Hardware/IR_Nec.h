#ifndef __IR_NEC_H
#define __IR_NEC_H

/* ------------------ 用户可修改的宏 ------------------ */

#define IR_GPIO_APBX      RCC_APB2PeriphClockCmd
#define IR_GPIO_CLK       RCC_APB2Periph_GPIOA       /* GPIO时钟 */
#define IR_GPIO_PORT      GPIOA	                     /* GPIO端口 */
#define IR_GPIO_PIN       GPIO_Pin_8
#define IR_TIM    	      TIM1		                /* 定时器1 */
#define IR_TIM_UPDATE_IRQn     TIM1_UP_IRQn  /* TIM更新中断 */
#define IR_TIM_UPDATE_IRQHandler  TIM1_UP_IRQHandler
#define IR_TIM_CC_IRQn     TIM1_CC_IRQn  /* TIM更新中断 */
#define IR_TIM_CC_IRQHandler  TIM1_CC_IRQHandler
/* --------------------------------------------------- */

//遥控器按键对应Hex值
#define IR_UP    0x18
#define IR_DOWN  0x52
#define IR_OK    0x1C
#define IR_LEFT  0x08
#define IR_RIGHT 0x5A
#define IR_1	 0x45
#define IR_2	 0x46
#define IR_3	 0x47
#define IR_4	 0x44
#define IR_5	 0x40
#define IR_6	 0x43
#define IR_7	 0x07
#define IR_8	 0x15
#define IR_9	 0x09
#define IR_0	 0x19
#define IR_X	 0x16
#define IR_J	 0x0D


void IR_Nec_Init(void);                 // 初始化
uint8_t IR_GetData(void);
uint8_t IR_GetDataFlag(void);
uint8_t IR_GetRepeatFlag(void);
uint8_t IR_GetAddress(void);
uint8_t IR_GetCommand(void);

#endif
