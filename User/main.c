#include "FreeRTOS.h"
#include "task.h"

//全局变量头文件
// #include "globals.h"

/* 开发版硬件bsp头文件 */
//#include "bsp_led.h"
// #include "bsp_usart.h"
// #include "bsp_led.h"


/* 任务句柄 */
/* 任务句柄是1个指针，用于指向1个任务，当任务创建好之后，它就具有了1个任务句柄
以后如果我们想要操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么这个句柄
可以为NULL */

/* 创建任务句柄 */
static TaskHandle_t AppTaskCreate_Handle = NULL;
/* LED任务句柄 */
static TaskHandle_t LED_Task_Handle = NULL;

/* 内核对象句柄 */
/* 信号量、消息队列、事件标志组、软件定时器这些都属于内核的对象，要想使用这些内核对象
必须先创建，创建成功之后会返回1个相应的句柄。实际上是1个指针，后续我们就可以通过这个
句柄操作这些内核对象

内核对象说白了就是一种全局的数据结构，通过这些数据结构我们可以实现任务间的通信，任务间
的事件同步等各种功能。至于这些功能的实现我们是通过调用这些内核对象的函数来完成的 */


/* 全局变量声明 */
/* 当我们再写应用程序的时候，可能需要用到一些全局变量。 */

/****** 宏定义 ******/
/* 当我们在写应用程序的时候，可能会用到一些宏定义 */


/* 
***********************************************************
						函数声明 
***********************************************************
*/
static void AppTaskCreate(void);/* 用于创建任务 */

static void LED_Task(void * pvParameters);/* WIFI_Task任务实现 */

static void BSP_Init(void); /* 用于初始化板子相关资源 */

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
	printf("包含功能:红外遥控、蓝牙控制、wifi控制并上报温度数据、\r\n");
	
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
static void AppTaskCreate(void)
{
	BaseType_t xReturn = pdPASS;/* 定义1个创建信息返回值，默认为pdPASS */
	taskENTER_CRITICAL();//进入临界区
	
	/* 创建WIFI_Task任务 */
	xReturn = xTaskCreate((TaskFunction_t)LED_Task,//任务函数
	(const char*)"LED_Task",//任务名称
	(uint16_t)512,//任务堆栈大小
	(void*)NULL,//传递给任务函数的参数
	(UBaseType_t)2,//任务优先级
	(TaskHandle_t*)&LED_Task_Handle);//任务控制块指针

	if(pdPASS == xReturn)
	{
		printf("LED_Task任务创建成功！\r\n");
	}
	else
	{
		printf("LED_Task任务创建失败！\r\n");
	}
	
	vTaskDelete(AppTaskCreate_Handle);//删除AppTaskCreate任务

	taskEXIT_CRITICAL();//推出临界区
}

/*
@函数名：LED_Task
@功能说明：LED_Task任务主体
@参数：
@返回值：无
*/
static void LED_Task(void * pvParameters)
{
	while(1)
	{
		//LED1_ON;
		vTaskDelay(20);//延时20个tick
	}
}

/* 所有板子上的初始化均可放在这个函数里 */
void BSP_Init(void)
{
	/* 中断优先级分组为4，即4bit都用来表示抢占优先级别，范围为：0-15
	优先级分组只需要分组1次即可，以后如果有其他的任务都需要用到中断，都统一用这个优先级分组，
	切忌，千万不要再分组 */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	/*LED初始化*/
	//LED_GPIO_Config();
}
