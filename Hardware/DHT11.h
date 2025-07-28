#ifndef __DHT11_H
#define __DHT11_H

#define DHT11_GPIO_APBX RCC_APB2PeriphClockCmd
#define DHT11_GPIO_CLK  RCC_APB2Periph_GPIOA
#define DHT11_GPIO_PORT GPIOA
#define DHT11_GPIO_PIN  GPIO_Pin_15
						   
#define DHT11_OUT(x)    GPIO_WriteBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN, (BitAction)x)  //PA8控制DHT11
#define DHT11_DQ_IN     GPIO_ReadInputDataBit(DHT11_GPIO_PORT, DHT11_GPIO_PIN)        //PA8控制DHT11	

void DHT11_Init(void);
void DHT11_Run(void);
char DHT11_Read_Data(char *temp, char *humi);

void DHT11_IO_IN(void);
void DHT11_IO_OUT(void);
char DHT11_Check(void);
char DHT11_Read_Bit(void);
char DHT11_Read_Byte(void);

#endif
















