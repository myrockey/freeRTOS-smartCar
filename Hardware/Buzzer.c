#include "Buzzer.h"

void Buzzer_Init(void)
{		
		/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*开启LED相关的GPIO外设时钟*/
		Buzzer_GPIO_APBX( Buzzer_GPIO_CLK , ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//开启AFIO复用时钟
		/*选择要控制的GPIO引脚*/
		GPIO_InitStructure.GPIO_Pin = Buzzer_GPIO_PIN;	

		/*设置引脚模式为通用推挽输出*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*设置引脚速率为50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*调用库函数，初始化GPIO*/
		GPIO_Init(Buzzer_GPIO_PORT, &GPIO_InitStructure);	
		//置高电平
        GPIO_SetBits(Buzzer_GPIO_PORT, Buzzer_GPIO_PIN);
}
