/**
 * @file    Icm42688p.hpp
 * @brief   ICM-42688-P 驱动(包官方 Icm426xx 驱动)。
 *          SPI4 + PE11(CS) + PE15(DRDY EXTI),read_reg 走 DMA 后端。
 *          骨架阶段:成员就位,方法桩实现,后续按设计文档 §3/§5 填充。
 */
#ifndef ICM42688P_HPP
#define ICM42688P_HPP

#include "imu.hpp"

extern "C" {
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h"
#include "Invn/Drivers/Icm426xx/Icm426xxExtFunc.h"   /* sleep_us / get_time_us 声明 */
}

class Icm42688p : public ImuDriver {
public:
    bool init() override;
    void start() override;
    const ImuData &getData() const override { return data_; }

    /* 同步触发一次寄存器读(内部经 serif 读 → sensorEventCb 填 data_)。
     * 阶段4 用轮询验证数据;返回官方驱动 rc(0=ok)。 */
    int poll();

    /* 自检诊断(供调试器/上层查看) */
    uint8_t  whoAmI() const { return who_am_i_; }      /* 期望 0x47 */
    int      initRc() const { return init_rc_; }       /* inv_icm426xx_init 返回码,0=ok */
    uint32_t sampleCount() const { return sample_count_; } /* 已采集帧数(中断驱动自增) */

    /* ── ISR 跳板(由 main 的 HAL 回调按 pin/Instance 分发进来)── */
    static void onDrdyISR();        /* PE15 上升沿 → 通知采集任务 */
    static void onSpiDmaDoneISR();  /* SPI4 DMA 完成 → 释放完成信号量 */

private:
    /* 官方驱动数据回调(static,经实例访问) */
    static void sensorEventCb(inv_icm426xx_sensor_event_t *event);

    /* 采集任务:等 DRDY 通知 → 调官方驱动同步读 */
    static void acqTaskTramp(void *arg);
    void        acqTask();

    ImuData                    data_{};         /* 最新一帧标定后数据 */
    struct inv_icm426xx        drv_{};          /* 官方驱动实例 */
    struct inv_icm426xx_serif  serif_{};        /* 总线读写回调 */
    uint8_t                    who_am_i_ = 0;   /* 最近一次读到的 WHO_AM_I */
    int                        init_rc_  = 0;   /* 最近一次 init 返回码 */
    volatile uint32_t          sample_count_ = 0; /* DRDY 驱动累计帧计数 */
};

#endif /* ICM42688P_HPP */
