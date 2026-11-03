/**
 * @file    bsp_platform.h
 * @brief   板级公共服务：DWT 微秒计时和 DMA 非缓存区配置。
 */
#ifndef BSP_PLATFORM_H
#define BSP_PLATFORM_H

#include <stdint.h>


/** @brief 初始化 DWT 周期计数器(供微秒计时/延时)。计时相关 API 前调用一次。 */
void bsp_time_init(void);

/** @brief 给 .dma_buffer 所在 RAM_D2 起始区配 MPU 非缓存属性。
 *         在 main 的 USER CODE SysInit 中、启动任何 DMA/任务之前调用一次。 */
void bsp_dma_region_init(void);

/** @brief 返回启动计时器以来的单调微秒时间。 */
uint64_t bsp_time_us(void);

/** @brief DWT 忙等指定微秒，仅用于短暂的芯片初始化时序。 */
void bsp_delay_us(uint32_t us);


#endif /* BSP_PLATFORM_H */
