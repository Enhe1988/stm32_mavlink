/**
 * @file    imu_calib.h
 * @brief   IMU 零偏(零飘)校正,统一在【板子系】处理(轴映射之后、放置旋转之前)。
 *          放在板子系的好处:零偏 = map(芯片原始零偏),只跟芯片有关、
 *          【与平放/竖放无关】—— 一套写死的值平放竖放都通用。
 *
 *  位置:各驱动 imu_axis_apply 之后、imu_orient_apply 之前调用 imu_calib_apply()。
 *  标定:imu_calib_run() 上电静止采样算机体系零偏,再用 imu_orient_unapply 转回板子系存。
 *        本次启动跑实测标定 还是 用写死值,由 bsp_config.h 的 IMU_CALIB_ON_BOOT 决定:
 *          定义   → imu_calib_run()(debug 时读 g_imu_bias 抄进 imu_calib.c 的写死值)
 *          不定义 → imu_calib_load_stored()(用写死值)
 */
#ifndef IMU_CALIB_H
#define IMU_CALIB_H

#include <stdint.h>
#include "imu_api.h"   /* ImuData / imu_get_data */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float   gyro[3];    /* 板子系陀螺零偏 (rad/s) */
    float   accel[3];   /* 板子系加速度零偏 (m/s²) */
    uint8_t valid;      /* 1 = 已加载/已标定,apply 才生效 */
} imu_bias_t;

/** @brief 全局零偏(调试器可观测;板子系,与放置无关)。 */
extern volatile imu_bias_t g_imu_bias;

/** @brief 就地从板子系 gyro/accel 减去零偏(valid=0 时不动)。 */
static inline void imu_calib_apply(float gyro[3], float accel[3])
{
    if (!g_imu_bias.valid) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        gyro[i]  -= g_imu_bias.gyro[i];
        accel[i] -= g_imu_bias.accel[i];
    }
}

/** @brief 上电静止+水平采样标定。采 samples 帧均值算机体系零偏,转回板子系写入
 *         g_imu_bias 并置 valid。必须在 imu_start() 之后调用;采样期间板子静止且水平。
 *         debug 时把 g_imu_bias 的值抄进 imu_calib.c 的写死表,即可下次用 stored。 */
void imu_calib_run(unsigned samples);

/** @brief 加载 imu_calib.c 里按芯片写死的零偏(板子系)到 g_imu_bias。
 *         不跑实测标定时用(IMU_CALIB_ON_BOOT 未定义)。 */
void imu_calib_load_stored(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CALIB_H */
