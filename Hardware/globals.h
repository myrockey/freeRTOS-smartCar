#ifndef __GLOBALS_H
#define __GLOBALS_H

// 定义枚举
typedef enum {
    TYPE_FORWARD = 0x01, // 向前
    TYPE_BACKWORD = 0x02, // 向后
    TYPE_STOP = 0x03, // 停车
    TYPE_LEFT = 0x04, // 向左
    TYPE_RIGHT = 0x05, // 向右
    TYPE_CLOCKWISE_ROTATION = 0x06, // 顺时针旋转
    TYPE_COUNTERCLOCKWISE_ROTATION = 0x07, // 逆时针旋转
    TYPE_ULTRASONIC_DISTANCE = 0x08, // 超声波测距
    TYPE_TRACKING = 0x09, // 循迹
    TYPE_LED_ON = 0x0A, // LED ON
    TYPE_LED_OFF = 0x0B, // LED OFF
    TYPE_READ_DHT11 = 0x0C, // 读取温湿度
    TYPE_SERVO_0 = 0x0D, // Servo 0
    TYPE_SERVO_45 = 0x0E, // Servo 45
    TYPE_SERVO_90 = 0x0F, // Servo 90
    TYPE_SERVO_135 = 0x10, // Servo 135
    TYPE_SERVO_180 = 0x11, // Servo 180
    TYPE_ULTRASONIC_OBSTACLE = 0x12 // 超声波避障
} CommandType;


void Filter_memcpy(uint8_t *dst, uint8_t *src, int size);//拷贝数据，并去除空字符
uint8_t extract_json(const char *raw, char *json_out);//从字符串中提取json

#endif /*__GLOBALS_H*/
