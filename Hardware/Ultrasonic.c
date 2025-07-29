#include "stm32f10x.h"                  // Device header
#include "Timer.h"
#include "Ultrasonic.h"
#include "SmartCar.h"
#include "Delay.h"

/*---------------- 仅需新增的全局变量 ----------------*/
static volatile int last_valid_distance = 0;   // 保存上一次有效距离(cm)
volatile uint32_t ic_rising = 0;
volatile uint32_t ic_falling = 0;
volatile uint8_t  ic_done   = 0;
volatile uint16_t ic_ovf    = 0;

//IO口初始化 及其他初始化
void Ultrasonic_Init(void)
{  
    GPIO_InitTypeDef GPIO_InitStructure;
    Hcsr04_GPIO_APBX(Hcsr04_GPIO_CLK, ENABLE);
   
    GPIO_InitStructure.GPIO_Pin = TRIG_Pin;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(Hcsr04_GPIO, &GPIO_InitStructure);
    GPIO_ResetBits(Hcsr04_GPIO ,TRIG_Pin);
     
    GPIO_InitStructure.GPIO_Pin = Echo_Pin;     
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(Hcsr04_GPIO, &GPIO_InitStructure);  
    GPIO_ResetBits(Hcsr04_GPIO,Echo_Pin);     

    TIM_Ultrasonic();
}

//获取定时器3计数器值
uint32_t GetEchoTimer(void)
{
    uint32_t t = 0;
   
    if (ic_falling >= ic_rising)
    {
        t = ic_falling - ic_rising;
    }
    else
    {
        t = (0xFFFF - ic_rising) + ic_falling + (uint32_t)ic_ovf * 0x10000UL;
    }

    return t;
}

//通过定时器3计数器值推算距离
float Ultrasonic_Distance(void)
{
   uint8_t valid_samples = 0;
   uint32_t t = 0;
   int i = 0;
   float sum = 0;
   uint32_t timeout = 0;

   for(i = 0; i < 5; i++)  //测量五次取平均
   {
        //timeout = 0;
        /* 触发10us脉冲 */
        TRIG_Send = 1;   //给控制端高电平
        Delay_us(20);
        TRIG_Send = 0;  //超声波模块已开始发送8个40khz脉冲

        /* 2. 等待捕获完成（超时 65 ms）*/
        ic_done = 0;
        ic_ovf  = 0;
        TIM_SetCounter(Ultrasonic_TIM, 0);
        while (!ic_done && timeout < 65000) { Delay_us(1); timeout++; }
        //while (!ic_done) {}
        
        /* 3. 计算距离 */
        if (ic_done)
        {
            t = GetEchoTimer();
            sum += ((float)t * 34/2000);   // us -> cm
            valid_samples++;
        }
    }

    if(valid_samples == 0)
    {
        return last_valid_distance;   // 保存上一次有效距离(cm)
    }
    last_valid_distance = sum / valid_samples;   // 只除有效次数
    return last_valid_distance; //取平均
}

//超声波避障
void Ultrasonic_Run(uint16_t distance)
{
    static uint8_t last_dir = 0;   // 0:左  1:右
    if (distance > 30) {
        // 距离远 → 全速
        Car_SetSpeed(80);
        Move_Forward();
    } else if (distance > 15) {
        // 中距 → 减速 40%
        Car_SetSpeed(40);
        Move_Forward();
    } else if (distance > 7) {
        // 近距 → 倒车
        Car_SetSpeed(60);
        Move_Backward();
    } else {
        // 危险区 → 交替转向
        Car_Stop();
        if (last_dir) {
            Turn_Left();
            last_dir = 0;
        } else {
            Turn_Right();
            last_dir = 1;
        }
        Delay_ms(300);
    }
}

//定时器3终中断
void Ultrasonic_TIM_IRQHandler(void)  
{
    /* 捕获中断 */
    if (TIM_GetITStatus(Ultrasonic_TIM, TIM_IT_CC4) != RESET)
    {
        TIM_ClearITPendingBit(Ultrasonic_TIM, TIM_IT_CC4);
        static uint8_t rising_captured = 0;
        if (!rising_captured)
        {
            /* 第一次上升沿 */
            ic_rising = TIM_GetCapture4(Ultrasonic_TIM);
            ic_ovf    = 0;
            rising_captured = 1;
            TIM_OC4PolarityConfig(Ultrasonic_TIM, TIM_ICPolarity_Falling);
        }
        else
        {
            /* 下降沿 */
            ic_falling = TIM_GetCapture4(Ultrasonic_TIM);
            ic_done    = 1;
            rising_captured = 0;
            TIM_OC4PolarityConfig(Ultrasonic_TIM, TIM_ICPolarity_Rising);
        }
    }

    /* 溢出中断：扩展计时范围 */
    if (TIM_GetITStatus(Ultrasonic_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(Ultrasonic_TIM, TIM_IT_Update);
        ic_ovf++;
    }
}
