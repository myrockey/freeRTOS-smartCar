#include "FreeRTOS.h"
#include "task.h"
#include "queue.h" // 队列
#include "event_groups.h" // 事件标志组
#include "semphr.h" //信号量

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
TaskHandle_t WIFI_Receive_Task_Handle = NULL;
TaskHandle_t WIFI_Send_Task_Handle = NULL;
TaskHandle_t Serial_Task_Handle = NULL;
TaskHandle_t CarCtrl_Task_Handle = NULL;
TaskHandle_t TrackSample_task_Handle = NULL;
TaskHandle_t Ultrasonic_Distance_Task_Handle = NULL;

/* 内核对象句柄 */
/* 信号量、消息队列、事件标志组、软件定时器这些都属于内核的对象，要想使用这些内核对象
必须先创建，创建成功之后会返回1个相应的句柄。实际上是1个指针，后续我们就可以通过这个
句柄操作这些内核对象

内核对象说白了就是一种全局的数据结构，通过这些数据结构我们可以实现任务间的通信，任务间
的事件同步等各种功能。至于这些功能的实现我们是通过调用这些内核对象的函数来完成的 */
QueueHandle_t  xCmdQueue;        // 串口/红外/语音 → 统一命令
QueueHandle_t  xUltraQueue;      // 超声波采样结果
QueueHandle_t  xTrackingQueue;   // 循迹线数据

/*	二值信号量句柄                         
 *	作用：用于控制MQTT命令缓冲处理任务，在MQTT数据接收发送缓冲处理任务中发出
 *		  当命令缓冲区收到命令数据时，发出信号量		 
 */
SemaphoreHandle_t BinarySemaphore;

/* 全局变量声明 */
/* 当我们再写应用程序的时候，可能需要用到一些全局变量。 */

cJSON* cjson_test = NULL;//json
cJSON* cjson_params = NULL;
char str[16]; // 定义一个长度为16的字符数组作为字符串

/****** 宏定义 ******/
/* 当我们在写应用程序的时候，可能会用到一些宏定义 */
#define STACK_SIZE 128

/* ---------- 调试开关 ---------- */
#define DEBUG_MODE_ENABLE   1       // 1 = 打开调试，0 = 关闭

/* ---------- 自动策略阈值 ---------- */
#define SAFE_DISTANCE_CM     30   // > 30 cm 全速
#define SLOW_DISTANCE_CM     15   // 15~30 cm 减速
#define BACK_DISTANCE_CM      7   // 7~15 cm  倒车
#define DANGER_DISTANCE_CM    5   // < 7 cm   危险区→转向

/* ---------- 速度表 ---------- */
#define SPD_CRUISE   70
#define SPD_SLOW     40
#define SPD_BACK     50
#define SPD_TURN     60

/* ---------- 转向延时 ---------- */
#define TURN_PULSE_MS 300

typedef enum
{
    CAR_RUN,        // 正常循迹
    CAR_AVOID_SLOW, // 减速前进
    CAR_AVOID_BACK, // 倒车
    CAR_AVOID_TURN  // 转向避障
} car_state_t;

static car_state_t car_state = CAR_RUN;
static TickType_t  state_enter_tick = 0;


/* 
***********************************************************
						函数声明 
***********************************************************
*/
void AppTaskCreate(void);/* 用于创建任务 */

// WIFI自动重连
void WIFI_Task(void * pvParameters);
// 接收数据并执行操作
void WIFI_Receive_Task(void * pvParameters);
// 发送数据
void WIFI_Send_Task(void * pvParameters);
// 串口收数据
void Serial_Task(void * pvParameters);
// 小车控制
void CarCtrl_Task(void * pvParameters);
// 自动循迹采样
void TrackSample_task(void * pvParameters);
// 超声波测距
void Ultrasonic_Distance_Task(void * pvParameters);

static void BSP_Init(void); /* 用于初始化板子相关资源 */
//根据参数，执行对应功能
static void Exec_Command(uint8_t cmd);
//根据参数，播报语音
static void Voice_broadcast(uint8_t type);

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

	OLED_ShowString(2,1,"D:");
	OLED_ShowString(3,1,"J:");
	OLED_ShowString(4,1,"T:");

	printf("这是1个STM32F103C8T6开发板-FreeCTOS-智能小车项目实验！\r\n");
	printf("包含功能:红外遥控、蓝牙控制、wifi远程控制、语音识别、测量温度并上传到IOT、自动循迹、超声波测距、超声波避障、舵机转动、电机驱动\r\n");
	
	//创建二值信号量
	BinarySemaphore = xSemaphoreCreateBinary();

	/* 创建内核对象 */
    xCmdQueue      = xQueueCreate(16, sizeof(uint8_t));
    xUltraQueue    = xQueueCreate(1, sizeof(uint16_t));
    xTrackingQueue = xQueueCreate(1, sizeof(uint8_t));

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
	(uint16_t)256,//任务堆栈大小
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
	
	xReturn = xTaskCreate((TaskFunction_t)Serial_Task,//任务函数
	(const char*)"Serial_Task",//任务名称
	(uint16_t)STACK_SIZE,//任务堆栈大小
	(void*)NULL,//传递给任务函数的参数
	(UBaseType_t)2,//任务优先级
	(TaskHandle_t*)&Serial_Task_Handle);//任务控制块指针
	if(pdPASS == xReturn)
	{
		printf("Serial_Task任务创建成功！\r\n");
	}
	else
	{
		printf("Serial_Task任务创建失败！\r\n");
	}

	xReturn = xTaskCreate((TaskFunction_t)CarCtrl_Task,//任务函数
	(const char*)"CarCtrl_Task",//任务名称
	(uint16_t)256,//任务堆栈大小
	(void*)NULL,//传递给任务函数的参数
	(UBaseType_t)4,//任务优先级
	(TaskHandle_t*)&CarCtrl_Task_Handle);//任务控制块指针
	if(pdPASS == xReturn)
	{
		printf("CarCtrl_Task任务创建成功！\r\n");
	}
	else
	{
		printf("CarCtrl_Task任务创建失败！\r\n");
	}

	xReturn = xTaskCreate((TaskFunction_t)TrackSample_task,//任务函数
	(const char*)"TrackSample_task",//任务名称
	(uint16_t)STACK_SIZE,//任务堆栈大小
	(void*)NULL,//传递给任务函数的参数
	(UBaseType_t)5,//任务优先级
	(TaskHandle_t*)&TrackSample_task_Handle);//任务控制块指针
	if(pdPASS == xReturn)
	{
		printf("TrackSample_task任务创建成功！\r\n");
	}
	else
	{
		printf("TrackSample_task任务创建失败！\r\n");
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

	xReturn = xTaskCreate((TaskFunction_t)WIFI_Receive_Task,//任务函数
	(const char*)"WIFI_Receive_Task",//任务名称
	(uint16_t)256,//任务堆栈大小
	(void*)NULL,//传递给任务函数的参数
	(UBaseType_t)7,//任务优先级
	(TaskHandle_t*)&WIFI_Receive_Task_Handle);//任务控制块指针
	if(pdPASS == xReturn)
	{
		printf("WIFI_Receive_Task任务创建成功！\r\n");
	}
	else
	{
		printf("WIFI_Receive_Task任务创建失败！\r\n");
	}

	xReturn = xTaskCreate((TaskFunction_t)WIFI_Send_Task,//任务函数
	(const char*)"WIFI_Send_Task",//任务名称
	(uint16_t)STACK_SIZE,//任务堆栈大小
	(void*)NULL,//传递给任务函数的参数
	(UBaseType_t)8,//任务优先级
	(TaskHandle_t*)&WIFI_Send_Task_Handle);//任务控制块指针
	if(pdPASS == xReturn)
	{
		printf("WIFI_Send_Task任务创建成功！\r\n");
	}
	else
	{
		printf("WIFI_Send_Task任务创建失败！\r\n");
	}

	vTaskDelete(AppTaskCreate_Handle);//删除AppTaskCreate任务

	taskEXIT_CRITICAL();//推出临界区
}

/* 所有板子上的初始化均可放在这个函数里 */
static void BSP_Init(void)
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
/* -------------- 本地命令解释器 -------------- */
static void Exec_Command(uint8_t cmd)
{
	uint16_t distance;
	uint8_t line;
    switch (cmd)
    {
		// 控制小车方向
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
		// 控制舵机转向
		case TYPE_SERVO_0://Servo 0
			strcpy(str, "servo 0 ");
			Servo_SetAngle(0);
			break;
		case TYPE_SERVO_45://Servo 45
			strcpy(str, "servo 45");
			Servo_SetAngle(45);
			break;
		case TYPE_SERVO_90://Servo 90
			strcpy(str, "servo 90");
			Servo_SetAngle(90);
			break;
		case TYPE_SERVO_135://Servo 135
			strcpy(str, "servo135");
			Servo_SetAngle(135);
			break;
		case TYPE_SERVO_180://Servo 180
			strcpy(str, "servo180");
			Servo_SetAngle(180);
			break;
		// LED ON/OFF
		case TYPE_LED_ON:
			LED1_ON;
			strcpy(str, " led on ");
			break;
		case TYPE_LED_OFF:
			LED1_OFF;
			strcpy(str, " led off");
			break;
		// 读取温度并上报
		case TYPE_READ_DHT11:
			xSemaphoreGive(BinarySemaphore);	     //给出二值信号量，控制发送任务执行
			strcpy(str, " dht11  ");
			break;
		case TYPE_ULTRASONIC_DISTANCE: // 超声波测距
			if (xQueuePeek(xUltraQueue, &distance, 0) == pdPASS)
			{
				OLED_ShowNum(2,4,distance,3);
			}
			strcpy(str, "distance");
			break;
		case TYPE_TRACKING: //自动循迹
			if (xQueuePeek(xTrackingQueue, &line, 0) == pdPASS)
			{
				OLED_ShowNum(3,4,line,3);
			}
			strcpy(str, "tracking");
			break;
		case TYPE_ULTRASONIC_OBSTACLE: // 自动避障
			strcpy(str, "obstacle");
			break;
		default:
			strcpy(str, " unknown");
			break;
    }

    /* 显示到 OLED */
    OLED_ShowNum(1, 14, cmd, 2);
    OLED_ShowString(1, 4, str);
}

/*
*根据参数，播报语音。
具体原理：stm32（发送方）通过串口传输数据 语音识别模块ASRPRO（接收方），根据接收的数据，执行语音播报。
*/
static void Voice_broadcast(uint8_t type)
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
void WIFI_Task(void * pvParameters)
{
	uint8_t wifiState;
	//服务器或者wifi已断开，清除事件标志，继续执行本任务，重新连接
	while(1)
	{
		OLED_ShowString(1,4,"wifi CON");
		printf("wifi connecting...\r\n");                 
		TIM_Cmd(WIFI_TIM, DISABLE);                       //关闭TIM3
		xEventGroupClearBits(Event_Handle, PING_MODE);//关闭发送PING包的定时器3，清除事件标志位
		ESP8266_Buf_Clear();//清空接收缓存区
		wifiState = ESP8266_WiFi_MQTT_Connect_IoTServer();
		OLED_ShowNum(1,1,wifiState,2);//显示wifi连接状态值
		if(wifiState == 0)			  //如果WiFi连接云服务器函数返回0，表示正确，进入if
		{   			     
			printf("wifi connect success and mqtt sub success\r\n");
			OLED_ShowString(1,4,"wifi OK ");      
			ESP8266_Buf_Clear();//清空接收缓存区

			xEventGroupSetBits(Event_Handle, WIFI_CONNECT);  //服务器已连接，抛出事件标志 
			
			//启动定时器30s模式
			TIM_WIFI_ENABLE_30S();
			pingFlag = 0;
			xEventGroupSetBits(Event_Handle, PING_MODE); //30s的PING定时器，设置事件标志位
			
			vTaskSuspend(NULL);	    						//服务器已连接，挂起自己，进入挂起态（任务由挂起转为就绪态时在这继续执行下去）
			xEventGroupClearBits(Event_Handle, WIFI_CONNECT);//服务器或者wifi已断开，清除事件标志，继续执行本任务，重新连接 
			xEventGroupClearBits(Event_Handle, PING_MODE);  //关闭发送PING包的定时器3，清除事件标志位
		}

		vTaskDelay(50);
	}
}

/*
@函数名：WIFI发送温湿度给服务器
@功能说明：Send_Task任务主体
@参数：
@返回值：无
*/
void WIFI_Send_Task(void * pvParameters)
{
	//服务器连接以及ping心跳包30S发送模式事件发生时执行此任务，否则挂起任务
	xEventGroupWaitBits((EventGroupHandle_t	)Event_Handle,		
						(EventBits_t		)PING_MODE,
						(BaseType_t			)pdFALSE,				
						(BaseType_t			)pdTRUE,
						(TickType_t			)portMAX_DELAY);

	char temp;//温度
	char humi;//湿度
	while (1)
	{
		xSemaphoreTake(BinarySemaphore, portMAX_DELAY);	//获取信号量，获取到信号量，继续执行，否则进入阻塞态，等待执行

		DHT11_Read_Data(&temp, &humi);//读取DHT11温度模块
		sprintf(str,"%d oC H:%d",temp,humi);
		OLED_ShowString(4,4,str);
		
		char message[CMD_BUFFER_SIZE] = {0};
		snprintf(message,sizeof(message),"{\\\"temperature\\\": %d}",temp);
		printf("wifi send dht:%s\n",message);
		ESP8266_MQTT_Publish(message);//添加数据，发布给服务器

		Delay_s(30);//30s才能上传1次。
	}
}

/*
@函数名：WIFI接收数据
@功能说明：Send_Task任务主体
@参数：
@返回值：无
*/
void WIFI_Receive_Task(void * pvParameters)
{
	int len;

	while (1)
	{

		//服务器连接事件发生执行此任务，否则挂起
		xEventGroupWaitBits((EventGroupHandle_t	)Event_Handle,		
							(EventBits_t		)WIFI_CONNECT,
							(BaseType_t			)pdFALSE,				
							(BaseType_t			)pdTRUE,
							(TickType_t			)portMAX_DELAY);
		
		// 等待IDLE中断的通知
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);//等待通知

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
					xEventGroupSetBits(Event_Handle, PING_MODE); //30s的PING定时器，设置事件标志位
				}
			}
			
			// 获取远程命令(TODO:待完善，接收的数据不完整。或调整设备传输数据类型为hex)
			if(strstr((const char*)received_str, MQTT_ATTR_PUSH_SUB) != NULL && strstr((const char*)received_str, "temp") != NULL){
				printf("IOT push data:%s \r\n",received_str); 		   	 //串口输出信息
				char json[128] = {0};//初始化缓冲区
				if(extract_json((const char*)received_str, json))
				{
					cjson_test = cJSON_Parse((const char*)json);
					if(cjson_test) {
						cjson_params = cJSON_GetObjectItem(cjson_test, "temp");
						if(cjson_params && cJSON_IsNumber(cjson_params)) {
							uint8_t cmd  = (uint8_t)cjson_params->valueint;
							xQueueSend(xCmdQueue, &cmd, 0);
							Voice_broadcast(cmd);
						}
						cJSON_Delete(cjson_test); // 必须添加
					} else {
						printf("JSON parse fail: %s\n", json);
					}
				}
			}
		}

		vTaskDelay(50);
	}
}

// 串口收到数据任务
void Serial_Task(void * pvParameters)
{
	uint8_t cmd;

	while(1)
	{
		if (IR_GetDataFlag())          // 收到红外遥控的完整数据帧
		{
			cmd = IR_GetData();
			xQueueSend(xCmdQueue, &cmd, 0);
			Voice_broadcast(cmd);
		}

		if (Bluetooth_Serial_GetRxFlag())          // 收到蓝牙数据标志
		{
			cmd = Bluetooth_Serial_GetRxData();
			xQueueSend(xCmdQueue, &cmd, 0);
			Voice_broadcast(cmd);
		}

		if (VoiceIdentify_Serial_GetRxFlag())          // 收到语音数据标志
		{
			cmd = VoiceIdentify_Serial_GetRxData();
			xQueueSend(xCmdQueue, &cmd, 0);
		}

		vTaskDelay(10);//延时10个tick
	}
}

// 小车控制
void CarCtrl_Task(void * pvParameters)
{
	uint16_t distance;
    uint8_t  line;
    uint8_t  cmd;        // 来自队列的远程指令
	
	/* 最近一次保持某状态的最短时间，防止抖动 */
    const TickType_t state_hold = pdMS_TO_TICKS(200);

	while(1)
	{
	// 调试输出
#if DEBUG_MODE_ENABLE
    static uint32_t dbg_cnt = 0;
    if (++dbg_cnt % 25 == 0)   // 每 25*20 ms ≈ 0.5 s 打印一次
    {
        printf("[DBG] st=%d  d=%u mm  ln=%d\n",
               car_state, distance, line);
    }
#endif
		 /* 0. 遥控器/语音/蓝牙 指令优先 */
		if (xQueueReceive(xCmdQueue, &cmd, 0) == pdPASS)
        {
            Exec_Command(cmd);
            car_state = CAR_RUN;           // 退出自动策略
            continue;
        }

		/* 1. 更新传感器数据 */
        xQueuePeek(xUltraQueue,    &distance, 0);
        xQueuePeek(xTrackingQueue, &line,     0);

        /* 2. 状态机主逻辑 */
        TickType_t now = xTaskGetTickCount();
		static uint8_t last_dir = 0;
		switch (car_state)
        {
			case CAR_RUN:
				if (distance < DANGER_DISTANCE_CM) // < 7
				{
					/* 直接进入危险区：先停车，再决定转向 */
					Car_Stop();
					car_state      = CAR_AVOID_TURN;
					state_enter_tick = now;
				}
				else if (distance < BACK_DISTANCE_CM) // 7 - 15
				{
					Car_SetSpeed(SPD_BACK);
					Move_Backward();
					car_state      = CAR_AVOID_BACK;
					state_enter_tick = now;
				}
				else if (distance < SLOW_DISTANCE_CM) // 15 - 30
				{
					Car_SetSpeed(SPD_SLOW);
					Move_Forward();
					car_state      = CAR_AVOID_SLOW;
					state_enter_tick = now;
				}
				else // > 30
				{
					/* 正常循迹 */
					Car_SetSpeed(SPD_CRUISE);  // 补上这句
					Tracking_Run(line);
				}
				break;

			case CAR_AVOID_SLOW:
				Car_SetSpeed(SPD_SLOW);
    			Move_Forward();               // ← 加上这句
				if (distance >= SLOW_DISTANCE_CM &&
					(now - state_enter_tick) >= state_hold)
				{
					car_state = CAR_RUN;
				}
				else if (distance < BACK_DISTANCE_CM)
				{
					car_state = CAR_AVOID_BACK;
					state_enter_tick = now;
				}
				break;

			case CAR_AVOID_BACK:
			    Car_SetSpeed(SPD_BACK);
    			Move_Backward();              // ← 加上这句
				if (distance >= BACK_DISTANCE_CM &&
					(now - state_enter_tick) >= state_hold)
				{
					car_state = CAR_AVOID_SLOW;
					state_enter_tick = now;
				}
				else if (distance >= SAFE_DISTANCE_CM)
				{
					car_state = CAR_RUN;
				}
				break;

			case CAR_AVOID_TURN:
				/* 危险区：左右交替转向 */
				if ((now - state_enter_tick) >= pdMS_TO_TICKS(TURN_PULSE_MS))
				{
					last_dir ^= 1;
					Car_SetSpeed(SPD_TURN);
					if (last_dir)
					{
						//Turn_Right();
						Clockwise_Rotation();//顺时针旋转
					}
					else
					{
						//Turn_Left();
						CounterClockwise_Rotation();//逆时针旋转
					}
					state_enter_tick = now;

					/* 如果已经远离障碍，回到 RUN */
					if (distance > SAFE_DISTANCE_CM)
					{
						car_state = CAR_RUN;
					}
				}
				break;
        }

		vTaskDelay(20);//延时20个tick
	}
}

// 循迹线值采样
void TrackSample_task(void * pvParameters)
{
	uint8_t line;
    TickType_t xLast = xTaskGetTickCount();
    while (1)
    {
        line = Tracking_Read();   // 返回 0b000~0b111
        xQueueOverwrite(xTrackingQueue, &line);
        vTaskDelayUntil(&xLast, pdMS_TO_TICKS(10));
    }
}

//超声波测距
void Ultrasonic_Distance_Task(void * pvParameters)
{
	uint16_t distance;
    TickType_t xLast = xTaskGetTickCount();
 	while (1)
    {
        distance = Ultrasonic_Distance();
        xQueueOverwrite(xUltraQueue, &distance);
        vTaskDelayUntil(&xLast, pdMS_TO_TICKS(20));
    }
}
