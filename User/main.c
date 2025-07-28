#include "FreeRTOS.h"
#include "task.h"

//全局变量头文件
#include "globals.h"

/* 开发版硬件bsp头文件 */
// #include <stdio.h>
// #include <stdlib.h>
#include "Delay.h"
#include "cJSON.h"
#include <string.h>
#include "OLED.h"
#include "Delay.h"
#include "RingBuff.h"
#include "Timer.h"
#include "SmartCar.h"
#include "Bluetooth.h"
#include "Ultrasonic.h"
#include "WIFI.h"
#include "Tracking.h"
//#include "Buzzer.h"
#include "DHT11.h"
#include "LED.h"
#include "Servo.h"
#include "IR_Nec.h"
#include "VoiceIdentify.h"

/* 任务句柄 */
/* 任务句柄是1个指针，用于指向1个任务，当任务创建好之后，它就具有了1个任务句柄
以后如果我们想要操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么这个句柄
可以为NULL */

/* 创建任务句柄 */
TaskHandle_t AppTaskCreate_Handle = NULL;
TaskHandle_t WIFI_Task_Handle = NULL;
TaskHandle_t IR_Task_Handle = NULL;
TaskHandle_t Bluetooth_Task_Handle = NULL;
TaskHandle_t VoiceIdentify_Task_Handle = NULL;
TaskHandle_t Ultrasonic_Distance_Task_Handle = NULL;
TaskHandle_t Receive_Task_Handle = NULL;


/* 内核对象句柄 */
/* 信号量、消息队列、事件标志组、软件定时器这些都属于内核的对象，要想使用这些内核对象
必须先创建，创建成功之后会返回1个相应的句柄。实际上是1个指针，后续我们就可以通过这个
句柄操作这些内核对象

内核对象说白了就是一种全局的数据结构，通过这些数据结构我们可以实现任务间的通信，任务间
的事件同步等各种功能。至于这些功能的实现我们是通过调用这些内核对象的函数来完成的 */


/* 全局变量声明 */
/* 当我们再写应用程序的时候，可能需要用到一些全局变量。 */
#define STACK_SIZE 256
cJSON* cjson_test = NULL;//json
cJSON* cjson_params = NULL;

uint8_t RxData;//串口接收数据的变量
uint8_t RxDataClearFlag;//串口接收的数据已读后是否清除标志位 0-不清除，1-清除
uint8_t wifiState;//记录循迹位置值的变量
int trackingVal;//记录循迹位置值的变量
int distance;//离障碍物距离
char temp;//温度
char humi;//湿度
char str[16]; // 定义一个长度为16的字符数组作为字符串

/****** 宏定义 ******/
/* 当我们在写应用程序的时候，可能会用到一些宏定义 */


/* 
***********************************************************
						函数声明 
***********************************************************
*/
void AppTaskCreate(void);/* 用于创建任务 */

// WIFI
void WIFI_Task(void * pvParameters);
// 红外遥控
void IR_Task(void * pvParameters);
// 蓝牙
void Bluetooth_Task(void * pvParameters);
// 语音识别
void VoiceIdentify_Task(void * pvParameters);
//超声波测距
void Ultrasonic_Distance_Task(void * pvParameters);
// 接收数据并执行操作
void Receive_Task(void * pvParameters);

static void BSP_Init(void); /* 用于初始化板子相关资源 */

//根据参数，执行对应功能
void Exec_Function(uint8_t type);
//根据参数，播报语音
void Voice_broadcast(uint8_t type);
void WIFI_Run(uint8_t* wifiState);
void WIFI_Receive_Task(uint8_t* RxData);
void WIFI_Send_DHT(char *temp, char *humi);

/*业务操作，函数声明 */

// 舵机
void Servo_Task(uint8_t angle);
//小车电机驱动
void SmartCar_Task(uint8_t state);
// 自动循迹
void Tracking_Task(void);
//读取温度并上传到IOT服务器
void DHT11_Task(void);
// 超声波避障
void Ultrasonic_Task(void);

/*业务操作，函数声明 */

/**
*
*@brief 主函数
*@param 无
*@retval 无
*@note 第一步：开发板硬件初始化
*	   第2步：创建App应用任务
*	   第3步：启东FreeRTOS，开启多任务调度
*/
int main(void)
{
	BaseType_t xReturn = pdPASS;/* 定义1个创建信息返回值，默认为pdPASS */

	/* 开发板硬件初始化 */
	BSP_Init();

	printf("这是1个STM32F103C8T6开发板-FreeCTOS-智能小车项目实验！\r\n");
	printf("包含功能:红外遥控、蓝牙控制、wifi远程控制、语音识别、测量温度并上传到IOT、自动循迹、超声波测距、超声波避障、舵机转动、电机驱动\r\n");
	
	/* 创建 AppTaskCreate 任务 */
	xReturn = xTaskCreate((TaskFunction_t)AppTaskCreate,/* 任务函数 */
	(const char*)"AppTaskCreate",/* 任务名称 */
	(uint16_t)512,/* 任务堆栈大小 */
	(void*)NULL,/* 传递给任务函数的参数 */
	(UBaseType_t)1,/* 任务优先级 */
	(TaskHandle_t*)&AppTaskCreate_Handle);/* 任务控制块指针 */

	if(pdPASS == xReturn)/* 创建成功 */
	{
		vTaskStartScheduler();/* 启动任务，开启调度 */
	}
	else
	{
		printf("Task creation failed!\r\n");
		return -1;
	}

	while(1);/* 正常不会执行到这里 */
}


/*
@函数名：AppTaskCreate
@功能说明：为了方便管理，所有的任务创建函数都放在这个函数里面
@参数：无
@返回值：无
*/
void AppTaskCreate(void)
{
	BaseType_t xReturn = pdPASS;/* 定义1个创建信息返回值，默认为pdPASS */
	taskENTER_CRITICAL();//进入临界区
	
	/* 创建WIFI_Task任务 */
	xReturn = xTaskCreate((TaskFunction_t)WIFI_Task,//任务函数
	(const char*)"WIFI_Task",//任务名称
	(uint16_t)STACK_SIZE,//任务堆栈大小
	(void*)NULL,//传递给任务函数的参数
	(UBaseType_t)3,//任务优先级
	(TaskHandle_t*)&WIFI_Task_Handle);//任务控制块指针
	if(pdPASS == xReturn)
	{
		printf("WIFI_Task任务创建成功！\r\n");
	}
	else
	{
		printf("WIFI_Task任务创建失败！\r\n");
	}
	
	xReturn = xTaskCreate((TaskFunction_t)IR_Task,//任务函数
	(const char*)"IR_Task",//任务名称
	(uint16_t)STACK_SIZE,//任务堆栈大小
	(void*)NULL,//传递给任务函数的参数
	(UBaseType_t)2,//任务优先级
	(TaskHandle_t*)&IR_Task_Handle);//任务控制块指针
	if(pdPASS == xReturn)
	{
		printf("IR_Task任务创建成功！\r\n");
	}
	else
	{
		printf("IR_Task任务创建失败！\r\n");
	}

	xReturn = xTaskCreate((TaskFunction_t)Bluetooth_Task,//任务函数
	(const char*)"Bluetooth_Task",//任务名称
	(uint16_t)STACK_SIZE,//任务堆栈大小
	(void*)NULL,//传递给任务函数的参数
	(UBaseType_t)4,//任务优先级
	(TaskHandle_t*)&Bluetooth_Task_Handle);//任务控制块指针
	if(pdPASS == xReturn)
	{
		printf("Bluetooth_Task任务创建成功！\r\n");
	}
	else
	{
		printf("Bluetooth_Task任务创建失败！\r\n");
	}

	xReturn = xTaskCreate((TaskFunction_t)VoiceIdentify_Task,//任务函数
	(const char*)"VoiceIdentify_Task",//任务名称
	(uint16_t)STACK_SIZE,//任务堆栈大小
	(void*)NULL,//传递给任务函数的参数
	(UBaseType_t)5,//任务优先级
	(TaskHandle_t*)&VoiceIdentify_Task_Handle);//任务控制块指针
	if(pdPASS == xReturn)
	{
		printf("VoiceIdentify_Task任务创建成功！\r\n");
	}
	else
	{
		printf("VoiceIdentify_Task任务创建失败！\r\n");
	}

	xReturn = xTaskCreate((TaskFunction_t)Ultrasonic_Distance_Task,//任务函数
	(const char*)"Ultrasonic_Distance_Task",//任务名称
	(uint16_t)STACK_SIZE,//任务堆栈大小
	(void*)NULL,//传递给任务函数的参数
	(UBaseType_t)6,//任务优先级
	(TaskHandle_t*)&Ultrasonic_Distance_Task_Handle);//任务控制块指针
	if(pdPASS == xReturn)
	{
		printf("Ultrasonic_Distance_Task任务创建成功！\r\n");
	}
	else
	{
		printf("Ultrasonic_Distance_Task任务创建失败！\r\n");
	}

	xReturn = xTaskCreate((TaskFunction_t)Receive_Task,//任务函数
	(const char*)"Receive_Task",//任务名称
	(uint16_t)512,//任务堆栈大小
	(void*)NULL,//传递给任务函数的参数
	(UBaseType_t)7,//任务优先级
	(TaskHandle_t*)&Receive_Task_Handle);//任务控制块指针
	if(pdPASS == xReturn)
	{
		printf("Receive_Task任务创建成功！\r\n");
	}
	else
	{
		printf("Receive_Task任务创建失败！\r\n");
	}

	vTaskDelete(AppTaskCreate_Handle);//删除AppTaskCreate任务

	taskEXIT_CRITICAL();//推出临界区
}

/* 所有板子上的初始化均可放在这个函数里 */
void BSP_Init(void)
{
	/* 中断优先级分组为4，即4bit都用来表示抢占优先级别，范围为：0-15
	优先级分组只需要分组1次即可，以后如果有其他的任务都需要用到中断，都统一用这个优先级分组，
	切忌，千万不要再分组 */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	TIM1_Init();
	OLED_Init();//显示屏初始化
	SmartCar_Init();//电机驱动初始化
	Bluetooth_Init();//蓝牙初始化
	Ultrasonic_Init();//超声波初始化
	Tracking_Init();//循迹初始化
	//Buzzer_Init();//蜂鸣器初始化
	DHT11_Init();
	LED_Init();//LED初始化
	WIFI_Init();
	Servo_Init();
	IR_Nec_Init();
	VoiceIdentify_Init();//语音识别初始化
}

/*
*根据参数，执行对应功能。
具体原理：蓝牙模块或语音识别模块（发送方） 通过串口传输数据 stm32（接收方），根据接收的数据，执行对应功能。
*/
void Exec_Function(uint8_t type)
{
	//默认清除
	if(RxData > 0){
		RxDataClearFlag = 1;
	}
	switch (RxData) {
        case TYPE_FORWARD:
        case TYPE_BACKWORD:
        case TYPE_STOP:
        case TYPE_LEFT:
        case TYPE_RIGHT:
        case TYPE_CLOCKWISE_ROTATION:
        case TYPE_COUNTERCLOCKWISE_ROTATION:
            SmartCar_Task(RxData);
            break;
        case TYPE_ULTRASONIC_DISTANCE:
			RxDataClearFlag = 0;
			strcpy(str, " distance");
            break;
        case TYPE_TRACKING:
			RxDataClearFlag = 0;
            Tracking_Task();
            break;
        case TYPE_LED_ON:
            LED1_ON;
			strcpy(str, " led on ");
            break;
        case TYPE_LED_OFF:
            LED1_OFF;
			strcpy(str, " led off");
            break;
        case TYPE_READ_DHT11:
            DHT11_Task();
            break;
        case TYPE_SERVO_0:
        case TYPE_SERVO_45:
        case TYPE_SERVO_90:
        case TYPE_SERVO_135:
        case TYPE_SERVO_180:
            Servo_Task(RxData);
            break;
        case TYPE_ULTRASONIC_OBSTACLE:
			RxDataClearFlag = 0;
            Ultrasonic_Task();
            break;
        default:
            //printf("Unknown command\n");
            break;
    }
}

/*
*根据参数，播报语音。
具体原理：stm32（发送方）通过串口传输数据 语音识别模块ASRPRO（接收方），根据接收的数据，执行语音播报。
*/
void Voice_broadcast(uint8_t type)
{
	switch(type)
	{
		case TYPE_LED_ON://LED ON
		case TYPE_LED_OFF://LED OFF
			VoiceIdentify_SendByte(type);
			break;
	}
}

/*
@函数名：WIFI_Run
@功能说明：WIFI运行并心跳检测，断开自动重连
@参数：
@返回值：无
*/
void WIFI_Run(uint8_t* wifiState)
{
	//服务器或者wifi已断开，清除事件标志，继续执行本任务，重新连接
	if(WIFI_CONNECT != 1)
	{
		printf("wifi connecting...\r\n");                 
		TIM_Cmd(WIFI_TIM, DISABLE);                       //关闭TIM3
		PING_MODE = 0;//关闭发送PING包的定时器3，清除事件标志位
		ESP8266_Buf_Clear();//清空接收缓存区
		*wifiState = ESP8266_WiFi_MQTT_Connect_IoTServer();
		if(*wifiState == 0)			  //如果WiFi连接云服务器函数返回0，表示正确，进入if
		{   			     
			printf("wifi connect success and mqtt sub success\r\n");
			OLED_ShowString(1,4,"wifi OK ");      
			ESP8266_Buf_Clear();//清空接收缓存区

			WIFI_CONNECT = 1;  //服务器已连接，抛出事件标志 

			//启动定时器30s模式
			TIM_WIFI_ENABLE_30S();
			pingFlag = 0;
			PING_MODE = 1; //30s的PING定时器，设置事件标志位
		}
	}
}

/*
@函数名：WIFI发送温湿度给服务器
@功能说明：Send_Task任务主体
@参数：
@返回值：无
*/
void WIFI_Send_DHT(char *temp, char *humi)
{
	//服务器连接以及ping心跳包30S发送模式事件发生时执行此任务，否则挂起任务
	if(PING_MODE != 1)
	{
		return;
	}
	//读取DHT11温度模块
	char message[CMD_BUFFER_SIZE] = {0};
	snprintf(message,sizeof(message),"{\\\"temperature\\\": %d}",*temp);
	printf("wifi send dht:%s\n",message);
	ESP8266_MQTT_Publish(message);//添加数据，发布给服务器
	Delay_s(30);//30s才能上传1次。
}

void WIFI_Receive_Task(uint8_t* RxData)
{
	//服务器连接事件发生执行此任务，否则挂起
	if(WIFI_CONNECT != 1)
	{
		return;
	}

	//等待接收数据通知
	if(WIFI_Receive_Flag != 1)
	{
		return;
	}
	WIFI_Receive_Flag = 0;

	int len;
	//printf("KEY_Task Running\r\n");
	len = RingBuff_GetLen(&encoeanBuff);
	if (len) {
		uint8_t received_str[len+1];
		RingBuff_ReadNByte(&encoeanBuff,received_str,len);
		received_str[len] = '\0';
		// 输出接收到的字符串
		printf("Received: %s\n", received_str);

		// ping状态，mqtt连接成功
		//+MQTTCONN:0,6,1,"gz-3-mqtt.iot-api.com","1883","",1\r\n\r\nOK
		if (strstr((const char*)received_str, "+MQTTCONN:0,6") != NULL && strstr((const char*)received_str, "OK") != NULL) {
			printf("PING success\r\n");                       
			if(pingFlag == 1)
			{                   						     //如果pingFlag=1，表示第一次发送
				pingFlag = 0;    				       		 //要清除pingFlag标志
			}
			else if(pingFlag > 1)	
			{ 				 								 //如果pingFlag>1，表示是多次发送了，而且是2s间隔的快速发送
				pingFlag = 0;     				      		 //要清除pingFlag标志
				TIM_WIFI_ENABLE_30S(); 				      		 //PING定时器重回30s的时间
				PING_MODE = 1; //30s的PING定时器，设置事件标志位
			}
		}
		
		// 获取远程命令(TODO:待完善，接收的数据不完整。或调整设备传输数据类型为hex)
		if(strstr((const char*)received_str, MQTT_ATTR_PUSH_SUB) != NULL && strstr((const char*)received_str, "temp") != NULL){
			printf("IOT push data:%s \r\n",received_str); 		   	 //串口输出信息
			char json[256] = {0};//初始化缓冲区
			if(extract_json((const char*)received_str, json))
			{
				cjson_test = cJSON_Parse((const char*)json);
				if(cjson_test) {
					cjson_params = cJSON_GetObjectItem(cjson_test, "temp");
					if(cjson_params && cJSON_IsNumber(cjson_params)) {
						*RxData = (uint8_t)cjson_params->valueint;
					}
					cJSON_Delete(cjson_test); // 必须添加
				} else {
					printf("JSON parse fail: %s\n", json);
				}
			}
		}
	}
}

// WIFI
void WIFI_Task(void * pvParameters)
{
	while(1)
	{
		if(WIFI_CONNECT == 0)
		{
			OLED_ShowString(1,4,"wifi CON.");
		}
		WIFI_Run(&wifiState);//WIFI运行
		OLED_ShowNum(1,1,wifiState,2);//显示wifi连接状态值
		
		WIFI_Receive_Task(&RxData);//WIFI接收数据，并ping连接状态
		//服务器连接以及ping心跳包30S发送模式事件发生时执行此任务，否则挂起任务
		if(PING_MODE == 0)
		{
			printf("WIFI connect error\r\n");
			OLED_ShowString(1,4,"wifi ERR");
			continue;
		}

		Voice_broadcast(RxData);

		vTaskDelay(20);//延时20个tick
	}
}

// 红外遥控
void IR_Task(void * pvParameters)
{
	while(1)
	{
		if (IR_GetDataFlag())          // 收到红外遥控的完整数据帧
		{
			RxData = IR_GetData();
			Voice_broadcast(RxData);
		}

		vTaskDelay(20);//延时20个tick
	}
}

// 蓝牙
void Bluetooth_Task(void * pvParameters)
{
	while(1)
	{
		if (Bluetooth_Serial_GetRxFlag())          // 收到数据标志
		{
			RxData = Bluetooth_Serial_GetRxData();

			Voice_broadcast(RxData);
		}
		
		vTaskDelay(20);//延时20个tick
	}
}

// 语音识别
void VoiceIdentify_Task(void * pvParameters)
{
	while (1)
	{
		if (VoiceIdentify_Serial_GetRxFlag())          // 收到数据标志
		{
			RxData = VoiceIdentify_Serial_GetRxData();
		}
	}
}

// 舵机
void Servo_Task(uint8_t type)
{
	uint8_t angle = 0;
	switch(type)
	{
		case TYPE_SERVO_0://Servo 0
			angle = 0;
			strcpy(str, "servo 0 ");
			break;
		case TYPE_SERVO_45://Servo 45
			angle = 45;
			strcpy(str, "servo 45");
			break;
		case TYPE_SERVO_90://Servo 90
			angle = 90;
			strcpy(str, "servo 90");
			break;
		case TYPE_SERVO_135://Servo 135
			angle = 135;
			strcpy(str, "servo135");
			break;
		case TYPE_SERVO_180://Servo 180
			angle = 180;
			strcpy(str, "servo180");
			break;
	}
	Servo_SetAngle(angle);
}

//小车电机驱动
void SmartCar_Task(uint8_t state)
{
	switch(state)
	{
		case TYPE_FORWARD:
			Move_Forward();
			strcpy(str, "forword ");
			break;
		case TYPE_BACKWORD:
			Move_Backward();
			strcpy(str, "backword");
			break;
		case TYPE_STOP:
			Car_Stop();
			strcpy(str, "  stop  ");
			break;
		case TYPE_LEFT:
			Turn_Left();
			strcpy(str, "  left  ");
			break;
		case TYPE_RIGHT:
			Turn_Right();
			strcpy(str, " right  ");
			break;
		case TYPE_CLOCKWISE_ROTATION://顺时针旋转
			Clockwise_Rotation();
			strcpy(str, " cycle  ");
			break;
		case TYPE_COUNTERCLOCKWISE_ROTATION://逆时针旋转
			CounterClockwise_Rotation();
			strcpy(str, " Ncycle ");
			break;
	}
}

// 自动循迹
void Tracking_Task(void)
{
	trackingVal = (L * 100)+ (M * 10) + (R * 1);
	OLED_ShowNum(3,4,trackingVal,3);//显示循迹模块的值
	
	Tracking_Run();
	strcpy(str, "tracking");
}

//读取温度并上传到IOT服务器
void DHT11_Task(void)
{
	DHT11_Read_Data(&temp, &humi);
	sprintf(str,"%d oC H:%d",temp,humi);
	OLED_ShowString(4,4,str);
	WIFI_Send_DHT(&temp,&humi);
	strcpy(str, " dht11  ");
}

//超声波测距
void Ultrasonic_Distance_Task(void * pvParameters)
{
	while (1)
	{
		distance = Ultrasonic_Distance();
		OLED_ShowNum(2,4,distance,3);//显示超声波距离	

		//距离太近时
		if(distance < 10)
		{
			//Buzzer_ON;
			LED1_ON;
		}
		else
		{
			//Buzzer_OFF;
			LED1_OFF;
		}

		vTaskDelay(20);
	}
}

// 超声波避障
void Ultrasonic_Task(void)
{
	Ultrasonic_Run();			
	strcpy(str, " sonic  ");
}

// 接收数据并执行操作
void Receive_Task(void * pvParameters)
{
	while (1)
	{
		if(RxData == 0)
		{
			continue;
		}

		if(RxData){
			OLED_ShowNum(1,14,RxData,2);//显示接收的参数
		}

		Exec_Function(RxData);
		OLED_ShowString(1,4,str);//显示执行的动作
		
		//清除上次接收的数据
		if(RxDataClearFlag == 1){
			RxData = 0;
		}

		vTaskDelay(20);
	}
}
