#include "stm32f10x.h"                  // Device header
#include "Bluetooth.h"
#include "Serial.h"

void Bluetooth_Init(void)
{
	Serial_Init_HC_05();
}

//发送数据
void Bluetooth_SendString(char *String)
{
	Serial_SendString(HC_05_USARTX, String);
}

uint8_t Bluetooth_Serial_GetRxFlag(void)
{
	return Serial_GetRxFlag_HC_05();
}

uint8_t Bluetooth_Serial_GetRxData(void)
{
	return Serial_GetRxData_HC_05();
}
