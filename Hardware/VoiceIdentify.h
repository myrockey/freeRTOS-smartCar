#ifndef __ASRPRO_H
#define __ASRPRO_H

void VoiceIdentify_Init(void);
void VoiceIdentify_SendString(char *String);
void VoiceIdentify_SendByte(uint8_t Byte);
uint8_t VoiceIdentify_Serial_GetRxFlag(void);
uint8_t VoiceIdentify_Serial_GetRxData(void);

#endif


