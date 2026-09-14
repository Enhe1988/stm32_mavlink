/**
 * @file    imu_selftest.h
 * @brief   两个 IMU 的通信自检(读 ID),结果存全局变量供调试器查看。
 *          与 bsp_config.h 的编译期选型无关:ICM 与 BMI088 都会测一遍。
 */
#ifndef IMU_SELFTEST_H
#define IMU_SELFTEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* ICM-42688-P (SPI4) */
    uint8_t  icm_who_am_i;       /* 期望 0x47 */
    int32_t  icm_init_rc;        /* 0=ok；-1 SPI；-2 复位；-3 ID；-4 配置；-5 任务 */
    uint8_t  icm_ok;             /* 1 = 完整初始化成功且 ID 命中 0x47 */

    /* BMI088 (SPI1) */
    uint8_t  bmi_accel_chip_id;  /* 期望 0x1E */
    uint8_t  bmi_gyro_chip_id;   /* 期望 0x0F */
    uint8_t  bmi_ok;             /* 1 = 两个 ID 都命中 */

    uint8_t  done;               /* 1 = 自检已执行完 */
} imu_selftest_t;

/** @brief 全局自检结果。调试器加到 watch 即可看到。 */
extern volatile imu_selftest_t g_imu_selftest;

/** @brief 执行一次自检(SPI 外设初始化后调用,如 INS_task 开头)。 */
void imu_selftest_run(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_SELFTEST_H */
