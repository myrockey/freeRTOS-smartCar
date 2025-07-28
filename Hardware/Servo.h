#ifndef __SERVO_H_
#define __SERVO_H_

// 配置舵机 SG90
#define SG90_GPIO_APBX      RCC_APB2PeriphClockCmd
#define SG90_GPIO_CLK       RCC_APB2Periph_GPIOA       /* GPIO时钟 */
#define SG90_GPIO_PORT    	GPIOA		                /* GPIO端口 */
#define SG90_GPIO_PIN       GPIO_Pin_11

//舵机
void Servo_Init(void);
void Servo_SetAngle(float Angle);

#endif
