#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

void Bluetooth_Init(void);
void Bluetooth_SendString(char *String);
uint8_t Bluetooth_Serial_GetRxFlag(void);
uint8_t Bluetooth_Serial_GetRxData(void);

#endif
