/**
 * @file    icm_platform.c
 * @brief   ICM 驱动平台服务：基于 Cortex-M7 DWT 的微秒计时/延时，
 *          以及 RAM_D2 DMA 区域的 MPU 配置。
 *
 *  DWT->CYCCNT 是 32 位、按 SystemCoreClock 递增。这里维护一个 64 位累加器,
 *  每次读取补上自上次以来的增量(含整 32 位回绕),换算成 us 返回。
 */
#include "icm_platform.h"
#include "stm32h7xx_hal.h"

static uint32_t s_cyc_per_us  = 1;   /* 每微秒周期数,init 时刷新 */
static uint64_t s_accum_cyc   = 0;   /* 64 位累计周期 */
static uint32_t s_last_cyc    = 0;   /* 上次读到的 CYCCNT */

void icm_platform_timer_init(void)
{
    /* 使能 trace + DWT 周期计数器 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#ifdef DWT_LAR_KEY  /* 某些核需要解锁 */
    DWT->LAR = 0xC5ACCE55;
#endif
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    s_cyc_per_us = SystemCoreClock / 1000000U;
    if (s_cyc_per_us == 0U) {
        s_cyc_per_us = 1U;
    }
    s_last_cyc  = DWT->CYCCNT;
    s_accum_cyc = 0;
}

uint64_t icm_platform_get_time_us(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t now   = DWT->CYCCNT;
    s_accum_cyc   += (uint32_t)(now - s_last_cyc);   /* 自然回绕 */
    s_last_cyc     = now;
    uint64_t us    = s_accum_cyc / s_cyc_per_us;

    if (!primask) {
        __enable_irq();
    }
    return us;
}

void icm_platform_delay_us(uint32_t us)
{
    /* DWT 忙等。init 序列里最长约 60ms;在 480MHz 下 ~2.9e7 周期 < 32 位范围,安全。 */
    uint32_t start  = DWT->CYCCNT;
    uint32_t target = us * s_cyc_per_us;
    while ((uint32_t)(DWT->CYCCNT - start) < target) {
        /* busy wait */
    }
}

void icm_platform_dma_region_init(void)
{
    /* 把 .dma_buffer 所在的 RAM_D2 起始 2KB 标记为 Normal 非缓存,
     * 解决 H743 "DMA 写内存 / CPU 读旧缓存" 一致性问题。
     * 只新增 region1,不动生成代码里 MPU_Config 的 region0(重生成安全)。
     * 容量需覆盖该段所有 DMA 缓冲:两款 ICM SPI + USART3 RX(1024B) +
     * USART2 RX(256B,光流),当前链接结果为 1408B。新增缓冲需确保不超 2KB。 */
    MPU_Region_InitTypeDef cfg = {0};

    HAL_MPU_Disable();

    cfg.Enable           = MPU_REGION_ENABLE;
    cfg.Number           = MPU_REGION_NUMBER1;
    cfg.BaseAddress      = 0x30000000U;          /* RAM_D2 起始 = .dma_buffer 基址 */
    cfg.Size             = MPU_REGION_SIZE_2KB;   /* 覆盖 ICM + 两个 USART DMA 缓冲 */
    cfg.SubRegionDisable = 0x00;
    cfg.TypeExtField     = MPU_TEX_LEVEL1;        /* TEX=1,C=0,B=0,S=1 → Normal 非缓存 */
    cfg.AccessPermission = MPU_REGION_FULL_ACCESS;
    cfg.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    cfg.IsShareable      = MPU_ACCESS_SHAREABLE;
    cfg.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    cfg.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&cfg);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
