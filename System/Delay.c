#include "FreeRTOS.h"
#include "task.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	/* 微秒级延时：使用忙等待，不受 FreeRTOS 抢占影响 */
    /* 假设主频 72 MHz，则 1 us = 72 个时钟周期 */
    const uint32_t loops_per_us = 72 / 5;  /* 每条空指令大约 5 个周期（Cortex-M3） */
    uint32_t total = xus * loops_per_us;
    while (total--) {
        __NOP();
    }
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
    vTaskDelay(pdMS_TO_TICKS(xms));
}

/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
    Delay_ms(xs * 1000);
}
