
/*-------------------------------------------------*/
/*                                                 */
/*   		     DHT11采集温湿度                   */
/*                                                 */
/*-------------------------------------------------*/

// 硬件连接：
// DATA：PA15

#include "stm32f10x.h"  //包含需要的头文件
#include "DHT11.h"      //包含需要的头文件 
#include "Delay.h"      //包含需要的头文件


/*函数名：初始化DHT11                              */	 
void DHT11_Init(void)
{
	DHT11_GPIO_APBX(DHT11_GPIO_CLK, ENABLE); //使能PA端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//开启AFIO复用时钟
	 
	GPIO_InitTypeDef GPIO_InitStructure;                  //定义一个IO端口参数结构体
	GPIO_InitStructure.GPIO_Pin =  DHT11_GPIO_PIN;            //准备设置引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //速率50Mhz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   	  //推免输出方式
	GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStructure);            	  //设置引脚
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //关闭JTAG
}

void DHT11_IO_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;                  //定义一个IO端口参数结构体
	GPIO_InitStructure.GPIO_Pin =  DHT11_GPIO_PIN;            //准备设置引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //速率50Mhz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   	  //上拉输入模式
	GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStructure);            	  //设置引脚
}

void DHT11_IO_OUT(void) 
{
	GPIO_InitTypeDef GPIO_InitStructure;                  //定义一个IO端口参数结构体
	GPIO_InitStructure.GPIO_Pin =  DHT11_GPIO_PIN;            //准备设置引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //速率50Mhz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   	  //推免输出方式
	GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStructure);            	  //设置引脚
}			

/*-------------------------------------------------*/
/*函数名：发送起始信号                               */
/*参  数：无                                       */
/*返回值：无                                       */
/*-------------------------------------------------*/  
void DHT11_Run(void)	   
{                 
	DHT11_IO_OUT(); 	//设置IO输出模式
	DHT11_OUT(0); 	//拉低IO
	Delay_ms(30);     //拉低至少18ms，我们拉低30
	DHT11_OUT(1); 	//拉高IO
	Delay_us(30);     //主机拉高20~40us，我们拉高30us
}

/*-------------------------------------------------*/
/*函数名：等待DHT11的回应                          */
/*参  数：无                                       */
/*返回值：1错误 0正确                              */
/*-------------------------------------------------*/ 
char DHT11_Check(void) 	   
{   
	char timeout;                            //定义一个变量用于超时判断  
	
	timeout = 0;                             //超时变量清零    
	DHT11_IO_IN();                           //IO设置输入模式
    while((DHT11_DQ_IN == 1) && (timeout < 100))//DHT11会拉低40~50us,我们等待70us超时时间	
	{	 
		timeout++;                           //超时变量+1
		Delay_us(1);                       	 //延时1us
	} 
	if(timeout >= 100)return 1;               //如果timeout>=70,说明是因为超时退出的while循环，返回1表示错误
	else timeout = 0;                        //反之，说明是因为等到了DHT11拉低IO，退出的while循环，正确并清零timeout
    while((DHT11_DQ_IN == 0) && (timeout < 100))//DHT11拉低后会再次拉高40~50us,,我们等待70us超时时间	
	{ 		
		timeout++;                           //超时变量+1
		Delay_us(1);                          
	}
	if(timeout >= 100)return 2;               //如果timeout>=70,说明是因为超时退出的while循环，返回2表示错误  
	return 0;                                //反之正确，返回0
}
/*-------------------------------------------------*/
/*函数名：读取一个位                               */
/*参  数：无                                       */
/*返回值：1或0                                     */
/*-------------------------------------------------*/ 
char DHT11_Read_Bit(void) 			 
{
 	char timeout;                          	   //定义一个变量用于超时判断  
	
	timeout = 0;                               //清零timeout	
	while((DHT11_DQ_IN == 1) && (timeout < 40))//每一位数据开始，是12~14us的低电平，我们等40us
	{   
		timeout++;                             //超时变量+1
		Delay_us(1);                            
	}
	timeout = 0;                               //清零timeout	
	while((DHT11_DQ_IN == 0) && (timeout < 60))//接下来，DHT11会拉高IO，根据拉高的时间判断是0或1，我们等60us
	{  
		timeout++;                             //超时变量+1
		Delay_us(1);                            
	}
	Delay_us(35);                               
	if(DHT11_DQ_IN)return 1;                   //如果延时后，是高电平，那么本位接收的是1，返回1
	else return 0;		                       //反之延时后，是低电平，那么本位接收的是0，返回0
}

/*-------------------------------------------------*/
/*函数名：读取一个字节                              */
/*参  数：无                                       */
/*返回值：数据                                      */
/*-------------------------------------------------*/ 
char DHT11_Read_Byte(void)    
{        
    char i;                       			  //定义一个变量用于for循环  
	char dat;                              	  //定义一个变量用于保存数据 
	dat = 0;	                        	  //清除保存数据的变量
	for (i = 0; i < 8; i++){              	  //一个字节8位，循环8次	
   		dat <<= 1;                    		  //左移一位，腾出空位    
	    dat |= DHT11_Read_Bit();      		  //读取一位数据
    }						    
    return dat;                   			  //返回一个字节的数据
}

/*-------------------------------------------------*/
/*函数名：读取一次数据温湿度                       */
/*参  数：temp:温度值                              */
/*参  数：humi:湿度值                              */
/*返回值：1错误 0正确                              */
/*-------------------------------------------------*/ 
char DHT11_Read_Data(char *temp, char *humi)    
{        
 	char buf[5];                              //一次完整的数据有5个字节，定义一个缓冲区
	
	char i;                                   //定义一个变量用于for循环  
	DHT11_Run();                              //复位DHT11	
	if(DHT11_Check() == 0)				      //判断DHT11回复状态=0的话，表示正确，进入if
	{
		for(i = 0; i < 5; i++){               //一次完整的数据有5个字节，循环5次		
			buf[i] = DHT11_Read_Byte();       //每次读取一个字节
		}
		/*
		[0] 湿度整数
		[1] 湿度小数（始终 0）
		[2] 温度整数
		[3] 温度小数（始终 0）
		[4] 校验和
		*/
		if((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])//判断数据校验，前4个字节相加应该等于第5个字节，正确的话，进入if	
		{     
			//printf("%d\r\n",buf[0]); 	//湿度数据
			//printf("%d\r\n",buf[1]);
			//printf("%d\r\n",buf[2]);	//温度数据
			//printf("%d\r\n",buf[3]);
			//printf("%d\r\n",buf[4]);
			*humi = buf[0];                   //湿度数据，保存在humi指针指向的地址变量中
			*temp = buf[2];					  //温度数据，保存在temp指针指向的地址变量中
		}else return 1;                       //反之，数据校验错误，直接返回1
	}else return 2;                           //反之，如果DHT11回复状态=1的话，表示错误，进入else，直接返回2
	
	return 0;	                              //读取正确返回0    
}  





