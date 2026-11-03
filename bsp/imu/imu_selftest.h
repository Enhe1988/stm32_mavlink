/**
 * @file    imu_selftest.h
 * @brief   三类 IMU 的上电探测结果，供调试器查看。
 */
#ifndef IMU_SELFTEST_H
#define IMU_SELFTEST_H

#include <stdint.h>


typedef struct {
    uint8_t  icm45686_who_am_i;  /* 期望 0xE9 */
    int32_t  icm45686_init_rc;
    uint8_t  icm45686_ok;

    uint8_t  icm42688_who_am_i;  /* 期望 0x47 */
    int32_t  icm42688_init_rc;
    uint8_t  icm42688_ok;

    /* BMI088 (SPI1) */
    uint8_t  bmi_accel_chip_id;  /* 期望 0x1E */
    uint8_t  bmi_gyro_chip_id;   /* 期望 0x0F */
    uint8_t  bmi_ok;             /* 1 = 两个 ID 都命中 */

    uint8_t  detected_mask;      /* IMU_DETECTED_* 位图 */
    uint8_t  active_type;        /* imu_type_t */
    uint8_t  init_ok;            /* 选中型号完整初始化成功 */
    uint8_t  done;               /* 1 = 探测已执行完 */
} imu_selftest_t;

/** @brief 全局自检结果。调试器加到 watch 即可看到。 */
extern volatile imu_selftest_t g_imu_selftest;

/** @brief 兼容旧调用：执行一次全型号身份探测，不启动采集。 */
void imu_selftest_run(void);


#endif /* IMU_SELFTEST_H */
