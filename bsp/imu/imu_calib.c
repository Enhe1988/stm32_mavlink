/**
 * @file    imu_calib.c
 * @brief   IMU 零偏标定实现。见 imu_calib.h。
 *          零偏存于【板子系】(与平放/竖放无关):
 *            - 实测标定:静止采样算机体系零偏 → imu_orient_unapply 转回板子系。
 *            - 写死值  :debug 时把 g_imu_bias 抄进下面的 s_stored_bias_*。
 */
#include "imu_calib.h"
#include "imu_orientation.h"   /* imu_orient_unapply */
#include "bsp_config.h"        /* 上电标定开关 */
#include "cmsis_os.h"          /* osDelay */

#define IMU_CALIB_GRAVITY 9.80665f   /* 水平静止时机体 +Z(下)应读到的重力 */

volatile imu_bias_t g_imu_bias = {0};

/* ── 按芯片写死的零偏(板子系,平放竖放通用)──
 * debug 跑 imu_calib_run 后,把 g_imu_bias 的 gyro/accel 抄到对应这里,
 * 再关掉 IMU_CALIB_ON_BOOT 即用这套值。初始为 0(=不校正)。 */
static const imu_bias_t s_stored_bias_icm45686 = {
    { 0.0f, 0.0f, 0.0f },   /* gyro  rad/s */
    { 0.0f, 0.0f, 0.0f },   /* accel m/s²  */
    1U
};
static const imu_bias_t s_stored_bias_icm42688 = {
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    1U
};
static const imu_bias_t s_stored_bias_bmi088 = {
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    1U
};

void imu_calib_load_stored(void)
{
    switch (imu_get_type()) {
    case IMU_TYPE_ICM45686:
        g_imu_bias = s_stored_bias_icm45686;
        break;
    case IMU_TYPE_ICM42688:
        g_imu_bias = s_stored_bias_icm42688;
        break;
    case IMU_TYPE_BMI088:
        g_imu_bias = s_stored_bias_bmi088;
        break;
    default:
        g_imu_bias = (imu_bias_t){0};
        break;
    }
}

void imu_calib_run(unsigned samples)
{
    if (samples == 0U) {
        return;
    }

    /* 标定期间 valid 保持 0,imu_calib_apply 不减偏 → 采到的是未校正数据 */
    double gsum[3] = {0.0, 0.0, 0.0};
    double asum[3] = {0.0, 0.0, 0.0};

    for (unsigned n = 0; n < samples; ++n) {
        ImuData sample;
        (void)imu_read(&sample);
        const ImuData *d = &sample;   /* 机体系完整快照 */
        for (int i = 0; i < 3; ++i) {
            gsum[i] += d->gyro[i];
            asum[i] += d->accel[i];
        }
        osDelay(2);   /* ~2ms 间隔,确保取到新帧 */
    }

    /* 机体系零偏:陀螺=均值;加速度=均值-(0,0,+9.8)(水平静止) */
    float gyro_body[3];
    float accel_body[3];
    for (int i = 0; i < 3; ++i) {
        gyro_body[i]  = (float)(gsum[i] / (double)samples);
        accel_body[i] = (float)(asum[i] / (double)samples);
    }
    accel_body[2] -= IMU_CALIB_GRAVITY;

    /* 转回板子系存储(与放置无关) */
    imu_orient_unapply(gyro_body);
    imu_orient_unapply(accel_body);
    for (int i = 0; i < 3; ++i) {
        g_imu_bias.gyro[i]  = gyro_body[i];
        g_imu_bias.accel[i] = accel_body[i];
    }
    g_imu_bias.valid = 1U;
}
