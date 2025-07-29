#include "Tracking.h"
#include "SmartCar.h"
#include "Delay.h"

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
  switch (line)
  {
    case 101: 
      Move_Forward(); 
      break; // 101 正中直行
    case 11: 
      Turn_Left(); 
      break; // 011 偏右小左转
    case 110: 
      Turn_Right();
      break; // 110 偏左小右转
    case 1: 
      Clockwise_Rotation(); 
      break; // 001 最左大右转
    case 100: 
      CounterClockwise_Rotation();
      break; // 100 最右大左转
    case 0:
    case 111: 
      Car_Stop(); 
      break; // 000 或 111 脱线停车
    default  : 
      Car_SetSpeed(40);
      Move_Forward(); 
      break; // 其余缓行
  }
}

void Tracking_Run2(uint8_t line)
{
    const int8_t kp = 25;   // PID 比例系数，现场微调
    const int8_t base = 50;

    /* 1. 把线值映射成误差 [-3, 3]：0b001->-3 … 0b101->0 … 0b100->+3 */
    int8_t err;
    switch (line)
    {
        case 0b001: err = -3; break;   // 最左
        case 0b011: err = -2; break;   // 偏左
        case 0b010: err = -1; break;   // 略左
        case 0b101: err =  0; break;   // 正中
        case 0b110: err =  1; break;   // 略右
        case 0b100: err =  2; break;   // 偏右
        default:    err =  0; break;   // 丢线或全黑
    }

    /* 2. 计算差速 */
    int8_t diff = kp * err;
    int8_t left  = base + diff;
    int8_t right = base - diff;

    /* 3. 限幅 0-100 */
    left  = left  < 0 ? 0 : (left  > 100 ? 100 : left);
    right = right < 0 ? 0 : (right > 100 ? 100 : right);

    /* 4. 直接驱动左右轮 */
    Car_SetLeftSpeed(left);
    Car_SetRightSpeed(right);
}