#include "stm32f10x.h"                  // Device header
#include "WIFI.h"
#include "Timer.h"
#include "RingBuff.h"
#include "Delay.h"
#include "Serial.h"

volatile char WIFI_CONNECT = 0;//服务器连接模式，1-表示已连接，0表示未连接
volatile char PING_MODE = 0;//ping心跳包发送模式，1表示开启30s发送模式，0表示未开启发送或开启2s快速发送模式。
volatile char pingFlag = 0;       //ping报文状态       0：正常状态，等待计时时间到，发送Ping报文
                         //ping报文状态       1：Ping报文已发送，当收到 服务器回复报文的后 将1置为0
uint8_t WIFI_Receive_Flag = 0;// WIFI接收到数据标志
uint8_t g_rx_esp8266_buf[WIFI_RX_BUFFER_SIZE] = {0};
volatile uint32_t g_rx_esp8266_cnt = 0;// 当前接收的字节数

uint8_t g_rx_dma_buf[USART2_DMA_RX_BUFFER_SIZE] = {0};//DMA接收数据缓冲区
volatile uint32_t g_rx_dma_cnt = 0;// 当前接收的字节数

/*函数名：初始化WiFi的复位IO                       */
/*参  数：无                                       */
/*返回值：无                                       */
/*-------------------------------------------------*/
void WIFI_Reset_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;                    //定义一个设置IO端口参数的结构体
	ESP8266_RESET_GPIO_APBX(ESP8266_RESET_GPIO_CLK , ENABLE); //使能PA端口时钟
	
	GPIO_InitStructure.GPIO_Pin = ESP8266_RESET_GPIO_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       //速率50Mhz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   	    //推免输出方式
	GPIO_Init(ESP8266_RESET_GPIO_PORT, &GPIO_InitStructure);  	    
	RESET_IO(1);                                            //复位IO拉高电平
}

void WIFI_Init(void)
{
	//Serial_Init_ESP8266(g_rx_dma_buf);
	Serial_DMA_Init_ESP8266(g_rx_dma_buf);
	WIFI_Reset_IO_Init();
	RingBuff_Init(&encoeanBuff);//环形缓冲区初始化
}

//发送数据
void WIFI_SendString(char *String)
{
	//Serial_SendString(ESP8266_USARTX, String);
	USART2_DMA_SendData((uint8_t*)String, strlen(String));//使用DMA方式发送
}

//清空接收缓存区
void ESP8266_Buf_Clear(void)
{
	//WiFi接收数据量变量清零                        
	//清空WiFi接收缓冲区 	
	g_rx_esp8266_cnt = 0;
	memset(g_rx_esp8266_buf,'\0',sizeof(g_rx_esp8266_buf));
}

/**
  * @brief  发送命令
  * @param  cmd 命令字符串
  * @param  res 响应关键词字符串
  * @param  timeOut 超时时间（100ms的倍数）
  * @retval 0-表示响应成功，1-表示响应失败
  */
char ESP8266_WiFi_SendCmd(char *cmd, char *res, uint8_t timeout)
{
	ESP8266_Buf_Clear();
	WIFI_SendString(cmd);
	while(timeout--)
	{
		Delay_ms(100);
		if(strstr((const char *)g_rx_esp8266_buf, res) != NULL)		//如果检索到关键词
		{
			return 0;
		}
		printf("timeout:%d \r\n", timeout);		//输出串口的超时时间
	}
	
	return 1; 
}

/*-------------------------------------------------*/
/*函数名：WiFi复位                                 */
/*参  数：timeout：超时时间（100ms的倍数）         */
/*返回值：0：正确   其他：错误                     */
/*-------------------------------------------------*/
char ESP8266_WiFi_Reset(int timeout)
{
	RESET_IO(0);                                    //复位IO拉低电平
	Delay_ms(500);                                  //延时500ms
	RESET_IO(1);                                   	//复位IO拉高电平	
	while(timeout--)								//等待超时时间到0 
	{                              		  
		Delay_ms(100);                              //延时100ms
		if(strstr((const char*)g_rx_esp8266_buf, "ready") != NULL)            //如果接收到ready表示复位成功
		{
			return 0;		         				   	//反之，表示正确，说明收到ready，通过break主动跳出while
		}
		//printf("reset rx: size:%d %s\r\n", g_rx_esp8266_cnt,g_rx_esp8266_buf);
		printf("reset timeout:%d", timeout);                     //串口输出现在的超时时间
	}
	return 1;                      //如果timeout<=0，说明超时时间到了，也没能收到ready，返回1
}

/*-------------------------------------------------*/
/*函数名：WiFi加入路由器指令                       */
/*参  数：timeout：超时时间（1s的倍数）            */
/*返回值：0：正确   其他：错误                     */
/*-------------------------------------------------*/
char ESP8266_WiFi_JoinAP(int timeout)
{		
	ESP8266_Buf_Clear();
	
	char cmd_buffer[CMD_BUFFER_SIZE];
	// 连接到 WiFi
	snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASSWORD);
	WIFI_SendString(cmd_buffer);
	while(timeout--)									   //等待超时时间到0
	{                                   
		Delay_ms(1000);                             	   //延时1s
		if(strstr((const char*)g_rx_esp8266_buf, "WIFI GOT IP\r\n\r\nOK") != NULL)   //如果接收到WIFI GOT IP表示成功
		{
			return 0;		
		}
		printf("joinAp timeout:%d \r\n", timeout);                            //串口输出现在的超时时间
	}
	return 1;                              //如果timeout<=0，说明超时时间到了，也没能收到WIFI GOT IP，返回1                                              //正确，返回0
}

/*-------------------------------------------------*/
/*函数名：WiFi_Smartconfig                         */
/*参  数：timeout：超时时间（1s的倍数）            */
/*返回值：0：正确   其他：错误                     */
/*-------------------------------------------------*/
char ESP8266_WiFi_Smartconfig(int timeout)
{
	ESP8266_Buf_Clear();
	while(timeout--)									//等待超时时间到0
	{                           		
		Delay_ms(1000);                         		//延时1s
		if(strstr((const char*)g_rx_esp8266_buf, "connected"))    	 	//如果串口接受到connected表示成功
		{
			break;                                  	//跳出while循环  
		}
		printf("Smartconfig timeout:%d \r\n", timeout);                 		//串口输出现在的超时时间  
	}	  			
	if(timeout <= 0)return 1;                     		//超时错误，返回1
	return 0;                                   		//正确返回0
}

/*-------------------------------------------------*/
/*函数名：等待加入路由器                           */
/*参  数：timeout：超时时间（1s的倍数）            */
/*返回值：0：正确   其他：错误                     */
/*-------------------------------------------------*/
char ESP8266_WiFi_WaitAP(int timeout)
{		
	while(timeout--){                               //等待超时时间到0
		Delay_ms(1000);                             //延时1s
		if(strstr((const char*)g_rx_esp8266_buf, "WIFI GOT IP"))      //如果接收到WIFI GOT IP表示成功
		{
			break;       						 
		}
		printf("waitAp timeout:%d \r\n", timeout);                     //串口输出现在的超时时间
	}
	printf("\r\n");                             	//串口输出信息
	if(timeout <= 0)return 1;                       //如果timeout<=0，说明超时时间到了，也没能收到WIFI GOT IP，返回1
	return 0;                                       //正确，返回0
}

//连接到MQTT服务器
char ESP8266_Connect_MQTT_Server(void)
{
	char cmd_buffer[CMD_BUFFER_SIZE];
	printf("准备清除MQTT连接\r\n");                    
	WIFI_SendString("AT+MQTTCLEAN=0\r\n");
	printf("清除MQTT连接成功\r\n");

	// 配置 MQTT 用户信息
	snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n", MQTT_CLIENT_ID, MQTT_CLIENT_USER, MQTT_CLIENT_PASSWORD);
	printf("配置 MQTT 用户属性\r\n");                    
	if(ESP8266_WiFi_SendCmd(cmd_buffer,"OK",100)) 		  //配置 MQTT 用户属性，100ms超时单位，总计5s超时时间
	{           
		printf("配置 MQTT 用户属性失败，准备重启\r\n");     //返回非0值，进入if
		return 11;                                 
	}
	printf("配置 MQTT 用户属性成功\r\n");

	// 连接到 ThingsCloud MQTT 服务器
	snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+MQTTCONN=0,\"%s\",%d,0\r\n", MQTT_HOST, MQTT_PORT);//最后面的0代表不自动重连
	printf("连接指定 MQTT broker\r\n");  
	// +MQTTCONNECTED:0,1,"gz-3-mqtt.iot-api.com","1883","",0 且 OK
	if(ESP8266_WiFi_SendCmd(cmd_buffer,"OK",100)) 		  //连接指定 MQTT broker，100ms超时单位，总计5s超时时间
	{           
		printf("连接指定 MQTT broker失败，准备重启\r\n");     //返回非0值，进入if
		return 12;                                 
	}
	printf("连接指定 MQTT broker成功\r\n");
	
	return 0;                                     //正确返回0
}

 char ESP8266_WiFi_MQTT_Connect_IoTServer(void) {	
	printf("准备设置STA模式\r\n");                
	if(ESP8266_WiFi_SendCmd("AT+CWMODE=1\r\n","OK",100))			  //设置STA模式，100ms超时单位，总计5s超时时间
	{             
		printf("设置STA模式失败，准备重启\r\n");  //返回非0值，进入if
		return 1;                                 //返回2
	}
	printf("设置STA模式成功\r\n");     

	printf("准备复位模块\r\n");//设置模式后，需重启才能生效                   
	if(ESP8266_WiFi_Reset(100))							  //复位，100ms超时单位，总计5s超时时间
	//if(ESP8266_WiFi_SendCmd("AT+RST\r\n","OK",100))							  //复位，100ms超时单位，总计5s超时时间
	{                             
		printf("复位失败，准备重启\r\n");	      //返回非0值，进入if
		return 2;                                 //返回1
	} 
	printf("复位成功\r\n"); 
	
	printf("准备取消自动连接\r\n");            	  
	if(ESP8266_WiFi_SendCmd("AT+CWAUTOCONN=0\r\n","OK",50))		  //取消自动连接，100ms超时单位，总计5s超时时间
	{       
		printf("取消自动连接失败，准备重启\r\n"); //返回非0值，进入if
		return 3;                                 //返回3
	}
	printf("取消自动连接成功\r\n");         
			
	printf("准备连接路由器\r\n");                 	
	if(ESP8266_WiFi_JoinAP(30))							  //连接路由器,1s超时单位，总计30s超时时间
	{                          
		printf("连接路由器失败，准备重启\r\n");   //返回非0值，进入if
		return 4;                                 //返回4	
	}
	printf("连接路由器成功\r\n");   
	
	printf("准备MQTT连接服务器\r\n");                 	
	if(ESP8266_Connect_MQTT_Server())
	{                          
		printf("连接MQTT服务器失败，准备重启\r\n");   //返回非0值，进入if
		return 5;                                 //返回5
	}
	printf("连接MQTT服务器成功\r\n");       		
	
	printf("准备MQTT订阅主题\r\n");                 	
	if(ESP8266_MQTT_Subscribe())
	{                          
		printf("MQTT订阅主题失败，准备重启\r\n");   //返回非0值，进入if
		return 6;                                 //返回5
	}
	printf("MQTT订阅主题成功\r\n");
    	
	return 0;
 }

// 查询当前WIFI连接状态 返回： +CWJAP: 且 OK
void ESP8266_CheckWiFiStatus(void)
{
	WIFI_SendString("AT+CWJAP?\r\n");
	//WIFI_SendString("AT+CIFSR\r\n");//连接成功，才能查到ip，所以也可以判断是否连接状态，返回数据：
	/*
	+CIFSR:STAIP,"192.168.14.220"
	+CIFSR:STAMAC,"ec:fa:bc:97:02:a1"

	OK
	*/
	//Delay_ms(2000);
}

// 查询当前MQTT连接状态 返回：+MQTTCONN:<LinkID>,<state>,<scheme><"host">,<port>,<"path">,<reconnect> 且 OK
// +MQTTCONN:0,6,1,"gz-3-mqtt.iot-api.com","1883","",1\r\n\r\nOK
void ESP8266_CheckMQTTStatus(void)
{
	WIFI_SendString("AT+MQTTCONN?\r\n");
	//Delay_ms(500);
}

void ESP8266_MQTT_Publish(char* message) {
    char cmd_buffer[CMD_BUFFER_SIZE];
	//char message[] = "{\\\"temperature\\\":30}";
	snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+MQTTPUB=0,\"%s\",\"%s\",0,0\r\n",MQTT_ATTR_PUB,message);
	WIFI_SendString(cmd_buffer);
	//Delay_ms(2000);
}

char ESP8266_MQTT_Subscribe(void) {
    char cmd_buffer[CMD_BUFFER_SIZE];
    snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+MQTTSUB=0,\"%s\",0\r\n", MQTT_ATTR_PUSH_SUB);
    return ESP8266_WiFi_SendCmd(cmd_buffer,"OK",20);
}

/* USART2中断函数 */
void ESP8266_IRQHandler(void)
{
    // 处理空闲中断
     if(USART_GetITStatus(ESP8266_USARTX, USART_IT_IDLE) != RESET)
    {
        // 读取SR和DR寄存器以清除IDLE标志
        ESP8266_USARTX->SR;
        ESP8266_USARTX->DR;
        
        // 停止DMA传输
        DMA_Cmd(USART2_RX_DMA_CHANNEL, DISABLE);
        
        // 获取接收到的数据长度
        g_rx_dma_cnt = USART2_DMA_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(USART2_RX_DMA_CHANNEL);
        
        if(g_rx_dma_cnt > 0)
        {
            if (WIFI_CONNECT == 0)
            {
                // 未连接服务器时的数据处理
                if(g_rx_dma_cnt < USART2_DMA_RX_BUFFER_SIZE)
                {
                    Filter_memcpy(g_rx_esp8266_buf, g_rx_dma_buf, g_rx_dma_cnt);
                    g_rx_esp8266_cnt = g_rx_dma_cnt;
                }
                else
                {
                  Filter_memcpy(g_rx_esp8266_buf, g_rx_dma_buf, USART2_DMA_RX_BUFFER_SIZE);
                  g_rx_esp8266_cnt = USART2_DMA_RX_BUFFER_SIZE;
                  // 可以添加一个标志位表示数据溢出
                  // uint8_t overflow_flag = 1;

                  // 可以通过LED或其他方式提示用户数据溢出
                  // LED_RED_ON();
                }
            }
            else
            {
                // 已连接服务器时的数据处理
                RingBuff_WriteNByte(&encoeanBuff, g_rx_dma_buf, g_rx_dma_cnt);
                
                // 重置定时器3计数器（ping包计时器）
                TIM_SetCounter(WIFI_TIM, 0);

                // 通知接收任务处理数据
                WIFI_Receive_Flag = 1;
            }
        }
        
        // 重新设置DMA传输数量并启动DMA
        USART2_RX_DMA_CHANNEL->CNDTR = USART2_DMA_RX_BUFFER_SIZE;
        DMA_Cmd(USART2_RX_DMA_CHANNEL, ENABLE);
    }
}

/*---------------------------------------------------------------*/
/*函数名：void TIM3_IRQHandler(void)				      			 */
/*功  能：定时器3中断处理函数									 */
/*		  1.控制ping心跳包的发送									 */
/*参  数：无                                       				 */
/*返回值：无                                     				 */
/*其  他：多次快速发送（2s，5次）没有反应，wifi任务由挂起态->就绪态*/
/*---------------------------------------------------------------*/
void WIFI_TIM_IRQHandler(void)
{
	if(TIM_GetITStatus(WIFI_TIM, TIM_IT_Update) != RESET)//如果TIM_IT_Update置位，表示TIM3溢出中断，进入if	
	{  
		printf("pingFlag=%d\r\n",pingFlag);
		switch(pingFlag) 					//判断pingFlag的状态
		{                               
			case 0:							//如果pingFlag等于0，表示正常状态，发送Ping报文  
					ESP8266_CheckMQTTStatus(); 		//添加Ping报文到发送缓冲区  
					break;
			case 1:							//如果pingFlag等于1，说明上一次发送到的ping报文，没有收到服务器回复，所以1没有被清除为0，可能是连接异常，我们要启动快速ping模式
					TIM_WIFI_ENABLE_2S(); 	//我们将定时器6设置为2s定时,快速发送Ping报文
					PING_MODE = 0;//关闭发送PING包的定时器3，设置事件标志位
					ESP8266_CheckMQTTStatus();			//添加Ping报文到发送缓冲区  
					break;
			case 2:							//如果pingFlag等于2，说明还没有收到服务器回复
			case 3:				            //如果pingFlag等于3，说明还没有收到服务器回复
			case 4:				            //如果pingFlag等于4，说明还没有收到服务器回复	
					ESP8266_CheckMQTTStatus();  		//添加Ping报文到发送缓冲区 
					break;
			case 5:							//如果pingFlag等于5，说明我们发送了多次ping，均无回复，应该是连接有问题，我们重启连接
					WIFI_CONNECT = 0;       //连接状态置0，表示断开，没连上服务器
					TIM_Cmd(WIFI_TIM, DISABLE); //关TIM3 				
					PING_MODE = 0;			//关闭发送PING包的定时器3，清除事件标志位
					break;			
		}
		pingFlag++;           		   		//pingFlag自增1，表示又发送了一次ping，期待服务器的回复
		TIM_ClearITPendingBit(WIFI_TIM, TIM_IT_Update); //清除TIM3溢出中断标志 	
	}
}
