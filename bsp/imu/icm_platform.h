/**
 * @file    icm_platform.h
 * @brief   ICM 官方驱动平台服务声明(ExtFunc 计时 + 初始化)。
 *          实现见 icm_platform.c。供 C/C++ 共用。
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

/* 以下两个为官方驱动 ExtFunc 钩子,定义在 icm_platform.c。
 * 声明见 Invn/Drivers/Icm426xx/Icm426xxExtFunc.h,这里不重复声明,
 * 仅暴露初始化入口。 */

#ifdef __cplusplus
}
#endif

#endif /* ICM_PLATFORM_H */
