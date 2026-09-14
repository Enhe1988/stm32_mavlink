/**
 * @file    icm_platform.h
 * @brief   ICM 驱动平台服务：DWT 微秒计时和 DMA 非缓存区配置。
 */
#ifndef ICM_PLATFORM_H
#define ICM_PLATFORM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化 DWT 周期计数器(供微秒计时/延时)。计时相关 API 前调用一次。 */
void icm_platform_timer_init(void);

/** @brief 给 .dma_buffer 所在 RAM_D2 起始区配 MPU 非缓存属性。
 *         在用 SPI4 DMA 前调用一次(Icm42688p::init 内)。 */
void icm_platform_dma_region_init(void);

/** @brief 返回启动计时器以来的单调微秒时间。 */
uint64_t icm_platform_get_time_us(void);

/** @brief DWT 忙等指定微秒，仅用于短暂的芯片初始化时序。 */
void icm_platform_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* ICM_PLATFORM_H */
