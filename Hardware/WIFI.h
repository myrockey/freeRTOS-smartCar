#ifndef __WIFI_H
#define __WIFI_H

#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

#include "globals.h"
#include "Serial.h"


#define ESP8266_RESET_GPIO_APBX RCC_APB2PeriphClockCmd
#define ESP8266_RESET_GPIO_CLK  RCC_APB2Periph_GPIOA
#define ESP8266_RESET_GPIO_PORT GPIOA
#define ESP8266_RESET_GPIO_PIN	GPIO_Pin_12 //硬件复位引脚
#define RESET_IO(x)    GPIO_WriteBit(GPIOA, ESP8266_RESET_GPIO_PIN, (BitAction)x)  //PA4控制WiFi的复位

#define WIFI_SSID "test" //wifi名称
#define WIFI_PASSWORD "12345678" //wifi密码
#define MQTT_CLIENT_ID "12"
#define MQTT_CLIENT_USER "ij6kv7tstqsrpzlx"
#define MQTT_CLIENT_PASSWORD "qDsJd0CqfV"
#define MQTT_HOST "gz-3-mqtt.iot-api.com"
#define MQTT_PORT 1883

//mqtt发布订阅相关
#define MQTT_ATTR_PUB      "attributes" //设备上报属性
#define MQTT_ATTR_SUB      "attributes/response" //设备上报属性后，设备等待接收平台响应
#define MQTT_ATTR_GET_PUB  "attributes/get/1000" //设备获取平台属性
#define MQTT_ATTR_GET_SUB  "attributes/get/response/1000" //设备发送获取平台属性后，设备等待接收平台响应
#define MQTT_ATTR_PUSH_SUB "attributes/push" //监听thingsCloud下发的属性，设备接收平台下发的属性
#define MQTT_COMMAND_SUB   "command/send/1000" //监听thingsCloud下发的命令
#define MQTT_COMMAND_PUB   "command/reply/1000" // 平台下发命令后，设备回复平台
#define MQTT_QoS 0
#define MQTT_RETAIN 0 //0-不保留信息

#define WIFI_RX_BUFFER_SIZE 1024 //wifi接收数据缓存
#define CMD_BUFFER_SIZE 128

extern EventGroupHandle_t Event_Handle;
extern const int WIFI_CONNECT;//服务器连接模式，1-表示已连接，0表示未连接
extern const int PING_MODE;//ping心跳包发送模式，1表示开启30s发送模式，0表示未开启发送或开启2s快速发送模式。
extern volatile char pingFlag;       //ping报文状态       0：正常状态，等待计时时间到，发送Ping报文
                         //ping报文状态       1：Ping报文已发送，当收到 服务器回复报文的后 将1置为0

extern uint8_t g_rx_esp8266_buf[WIFI_RX_BUFFER_SIZE];
extern volatile uint32_t g_rx_esp8266_cnt;// 当前接收的字节数

void WIFI_Init(void);
void WIFI_SendString(char *String);
//void WIFI_Run(void);

// ESP8266相关函数
//清空接收缓存区
void ESP8266_Buf_Clear(void);
//发送命令
char ESP8266_WiFi_SendCmd(char *cmd, char *res, uint8_t timeout);
/*函数名：WiFi复位                                 */
char ESP8266_WiFi_Reset(int timeout);
/*函数名：WiFi加入路由器指令                       */
char ESP8266_WiFi_JoinAP(int timeout);
/*函数名：WiFi_Smartconfig                         */
char ESP8266_WiFi_Smartconfig(int timeout);
/*函数名：等待加入路由器                           */
char ESP8266_WiFi_WaitAP(int timeout);
//连接到MQTT服务器
char ESP8266_Connect_MQTT_Server(void);
//WIFI通过MQTT通信连接IOT服务器
char ESP8266_WiFi_MQTT_Connect_IoTServer(void);
// 查询当前WIFI连接状态 返回： +CWJAP_DEF: 且 OK
void ESP8266_CheckWiFiStatus(void);
// 查询当前MQTT连接状态 返回：+MQTTCONN:<LinkID>,<state>,<scheme><"host">,<port>,<"path">,<reconnect> 且 OK
void ESP8266_CheckMQTTStatus(void);
void ESP8266_MQTT_Publish(char* message);
char ESP8266_MQTT_Subscribe(void);

#endif

