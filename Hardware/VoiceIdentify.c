#include "stm32f10x.h"                  // Device header
#include "VoiceIdentify.h"
#include "Serial.h"

// 示例参考：https://www.haohaodada.com/new/bbs/forum.php?mod=viewthread&tid=802
// 语音识别模块 ASRPRO的PB5（TX）引脚接STM32的A10（RX）引脚，PB6（RX）引脚接STM32的A9（TX）引脚。
void VoiceIdentify_Init(void)
{
	Serial_Init_ASRPRO();
}

//发送数据
void VoiceIdentify_SendString(char *String)
{
	Serial_SendString(ASRPRO_USARTX, String);
}

void VoiceIdentify_SendByte(uint8_t Byte)
{
	Serial_SendByte(ASRPRO_USARTX, Byte);
}

uint8_t VoiceIdentify_Serial_GetRxFlag(void)
{
	return Serial_GetRxFlag_ASRPRO();
}

uint8_t VoiceIdentify_Serial_GetRxData(void)
{
	return Serial_GetRxData_ASRPRO();
}
