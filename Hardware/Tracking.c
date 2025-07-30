#include "Tracking.h"
#include "SmartCar.h"
#include "Delay.h"

#define CLAMP(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define KP  25   // 比例系数：蛇形→减小，转弯不足→加大
#define BASE 50  // 基本速度：赛道直→提高，弯急→降低

//3路寻迹模块
void Tracking_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
	 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	  
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN_L|GPIO_PIN_M|GPIO_PIN_R;		
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 	 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 
  GPIO_Init(GPIOB, &GPIO_InitStructure);			     
}

uint8_t Tracking_Read(void)
{
  return (L * 100)+ (M * 10) + (R * 1);
}

//寻迹运动
//根据线的宽度来决定；
/*
传感器布局
循迹模块通常包含多个红外传感器（如3路或5路），这些传感器分布在沿路径方向的不同位置，用于检测路径的左右偏移情况。例如：
三路循迹模块：左传感器、中传感器、右传感器。
五路循迹模块：左2传感器、左1传感器、中传感器、右1传感器、右2传感器。

1.传感器输出
循迹模块的传感器会根据接收到的红外光强度输出不同的电平信号：
白色路径：反射光强，输出高电平（通常为逻辑1）。
黑色路径：反射光弱，输出低电平（通常为逻辑0）。

2.路径检测逻辑
根据多个传感器的输出，可以判断路径的偏移方向：
直线路径：左右传感器都检测到白色路径，中传感器检测到黑色。
右偏移：左传感器检测到白色，中传感器和右传感器检测到黑色。
左偏移：右传感器检测到白色，中传感器和左传感器检测到黑色。
完全偏离：所有传感器都检测到黑色或白色。

3. 控制器响应
控制器（如微控制器）根据传感器的输出信号，调整机器人的运动方向：
直线行驶：左右传感器都检测到白色路径，中传感器检测到黑色路径时，机器人直线行驶。
右转：检测到右偏移时，机器人右转。
左转：检测到左偏移时，机器人左转。
原地转向：完全偏离路径时，机器人原地转向，重新寻找路径。
*/
void Tracking_Run(uint8_t line)
{
    /* 1. 把线值映射成误差 [-3, 3]：0b001->-3 … 0b101->0 … 0b100->+3 */
    int8_t err;
    switch (line)
    {
        case 1: err = -3; break;     // 最左 0b001
        case 11: err = -2; break;    // 偏左 0b011
        case 10: err = -1; break;    // 略左 0b010 3路循迹这个case理论上不会出现
        case 101: err =  0; break;   // 正中 0b101
        case 110: err =  1; break;   // 略右 0b110
        case 100: err =  2; break;   // 偏右 0b100
        default:   
          Car_SetLeftSpeed(0);
          Car_SetRightSpeed(0);
          return; // 丢线或全黑 0b000 0b111
    }

    /* 2. 计算差速 */
    int8_t diff = KP * err;
    int8_t left  = BASE + diff;
    int8_t right = BASE - diff;

    /* 3. 限幅 -100-100 */
    left  = CLAMP(BASE + diff, -100, 100);
    right = CLAMP(BASE - diff, -100, 100);

    /* 4. 直接驱动左右轮 */
    Car_SetLeftSpeed(left);
    Car_SetRightSpeed(right);
}
