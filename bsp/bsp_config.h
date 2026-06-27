#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

/*
 * 编译期 IMU 选型:只定义其中一个。
 *   IMU_USE_ICM42688  —— ICM-42688-P,SPI4,中断+DMA
 *   IMU_USE_BMI088    —— BMI088,SPI1,轮询
 */
#define IMU_USE_ICM42688
 //#define IMU_USE_BMI088 

#if defined(IMU_USE_ICM42688) && defined(IMU_USE_BMI088)
#error "bsp_config.h: IMU_USE_ICM42688 与 IMU_USE_BMI088 只能二选一"
#endif
#if !defined(IMU_USE_ICM42688) && !defined(IMU_USE_BMI088)
#error "bsp_config.h: 必须定义 IMU_USE_ICM42688 或 IMU_USE_BMI088 之一"
#endif

/*
 * 编译期板子放置方式:只定义其中一个。对 gyro/accel 施加同一个旋转(见 imu_orientation.h)。
 *   BOARD_ORIENT_FLAT     —— 平放(BMI088 之前用的方式,恒等,不改变现有行为)
 *   BOARD_ORIENT_VERTICAL —— 竖放(相对平放转 90°,具体旋转待标定确认)
 */
#define BOARD_ORIENT_FLAT
/* #define BOARD_ORIENT_VERTICAL */

#if defined(BOARD_ORIENT_FLAT) && defined(BOARD_ORIENT_VERTICAL)
#error "bsp_config.h: BOARD_ORIENT_FLAT 与 BOARD_ORIENT_VERTICAL 只能二选一"
#endif
#if !defined(BOARD_ORIENT_FLAT) && !defined(BOARD_ORIENT_VERTICAL)
#error "bsp_config.h: 必须定义 BOARD_ORIENT_FLAT 或 BOARD_ORIENT_VERTICAL 之一"
#endif

/*
 * 上电零偏(零飘)校正开关:
 *   定义   → 本次启动跑一次静止采样实测标定,结果存 g_imu_bias(板子系,与放置无关)。
 *            ⚠️ 期间板子必须【静止且水平】约 1 秒。debug 读出 g_imu_bias 抄进
 *               imu_calib.c 的 s_stored_bias,之后就可关掉本开关用写死值。
 *   不定义 → 用 imu_calib.c 里按芯片写死的零偏(s_stored_bias)。
 */
#define IMU_CALIB_ON_BOOT

#define IMU_CALIB_SAMPLES   500U   /* 实测标定采样帧数(~2ms/帧 ≈ 1s) */

#endif /* BSP_CONFIG_H */
