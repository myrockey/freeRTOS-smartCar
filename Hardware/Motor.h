#ifndef __MOTOR_H
#define __MOTOR_H

// 配置电机 tb6612
#define TB6612_Motor_GPIO_APBX      RCC_APB2PeriphClockCmd
#define TB6612_Motor_GPIO_CLK       RCC_APB2Periph_GPIOA       /* GPIO时钟 */
#define TB6612_Motor_GPIO_PORT    	 GPIOA		                /* GPIO端口 */
#define TB6612_GPIO_PIN_AIN1 GPIO_Pin_4
#define TB6612_GPIO_PIN_AIN2 GPIO_Pin_5
#define TB6612_GPIO_PIN_BIN1 GPIO_Pin_6
#define TB6612_GPIO_PIN_BIN2 GPIO_Pin_7

// PWM配置
#define TB6612_GPIO_APBX      RCC_APB2PeriphClockCmd
#define TB6612_GPIO_CLK       RCC_APB2Periph_GPIOA       /* GPIO时钟 */
#define TB6612_GPIO_PORT      GPIOA		                /* GPIO端口 */
#define TB6612_GPIO_PIN_PWMA  GPIO_Pin_1
#define TB6612_GPIO_PIN_PWMB  GPIO_Pin_0

void Motor_Init(void);
//左电机
void MotorLeft_SetSpeed(int8_t Speed);
//右电机
void MotorRight_SetSpeed(int8_t Speed);
#endif
