/**
 * @file    Bmi088.cpp
 * @brief   BMI088 驱动实现:包现有 Bosch C 驱动(SPI1)+ 1kHz 轮询任务,
 *          配置沿用原 INS_task 的 init_bmi08(加速度 800Hz/6G,陀螺 1000Hz/1000dps),
 *          换算成 rad/s、m/s² → 轴映射(imu_frame)→ 放置旋转(imu_orientation)
 *          → 零偏扣除(imu_calib),填统一 ImuData。
 */
#include "Bmi088.hpp"
#include "imu_orientation.h"
#include "imu_frame.h"
#include "imu_calib.h"

extern "C" {
#include "bmi088_mm.h"   // bmi088_mma_*
#include "bmi08.h"       // bmi08g_* / bmi08a_*
#include "bmi08x.h"      // BMI088_ACCEL_CHIP_ID
#include "common.h"      // bmi08_interface_init, BMI08_SPI_INTF
#include "main.h"        // HAL_GetTick
#include "FreeRTOS.h"
#include "task.h"
}

/* ── 量程标定常数(与下面 config 设的量程一致)──
 * 16bit half-scale = 2^15 = 32768;accel ±6g;gyro ±1000dps */
#define BMI_HALF_SCALE  32768.0f
#define BMI_ACCEL_RANGE_G    6.0f
#define BMI_GYRO_RANGE_DPS   1000.0f
#define BMI_GRAVITY      9.80665f
#define BMI_DEG2RAD      0.017453292519943295f   /* π/180 */

/* 零偏统一由 imu_calib 在机体系处理(见 imu_calib.h);这里输出原始芯片轴。 */
static inline float bmi_accel_to_mps2(float raw)
{
    return (BMI_GRAVITY * raw * BMI_ACCEL_RANGE_G) / BMI_HALF_SCALE;
}
static inline float bmi_gyro_to_rads(float raw)
{
    return ((BMI_GYRO_RANGE_DPS * raw) / BMI_HALF_SCALE) * BMI_DEG2RAD;
}

bool Bmi088::probe()
{
    dev_ = {};

    /* SPI1 接口 + delay api 绑定到 dev_ */
    if (bmi08_interface_init(&dev_, BMI08_SPI_INTF) != BMI08_OK) {
        return false;
    }

    /* 两颗 die 都读一遍，单颗缺失时调试信息仍保留另一颗的 ID。 */
    const int8_t accel_result = bmi088_mma_init(&dev_);
    const int8_t gyro_result = bmi08g_init(&dev_);
    return (accel_result == BMI08_OK) && (gyro_result == BMI08_OK) &&
           (dev_.accel_chip_id == BMI088_ACCEL_CHIP_ID) &&
           (dev_.gyro_chip_id == BMI08_GYRO_CHIP_ID);
}

bool Bmi088::init()
{
    const bool already_probed =
        (dev_.accel_chip_id == BMI088_ACCEL_CHIP_ID) &&
        (dev_.gyro_chip_id == BMI08_GYRO_CHIP_ID);
    if (!already_probed && !probe()) {
        return false;
    }

    if (bmi08a_soft_reset(&dev_) != BMI08_OK) {
        return false;
    }

    /* 加速度:Active / 800Hz / ±6g / Normal BW */
    dev_.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    if (bmi08a_set_power_mode(&dev_) != BMI08_OK) {
        return false;
    }
    dev_.accel_cfg.odr   = BMI08_ACCEL_ODR_800_HZ;
    dev_.accel_cfg.range = BMI088_MM_ACCEL_RANGE_6G;
    dev_.accel_cfg.bw    = BMI08_ACCEL_BW_NORMAL;
    if (bmi08a_set_power_mode(&dev_) != BMI08_OK ||
        bmi088_mma_set_meas_conf(&dev_) != BMI08_OK) {
        return false;
    }

    /* 陀螺:Normal / 1000Hz / ±1000dps / 32Hz BW */
    dev_.gyro_cfg.odr   = BMI08_GYRO_BW_116_ODR_1000_HZ;
    dev_.gyro_cfg.range = BMI08_GYRO_RANGE_1000_DPS;
    dev_.gyro_cfg.bw    = BMI08_GYRO_BW_32_ODR_100_HZ;
    dev_.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    if (bmi08g_set_power_mode(&dev_) != BMI08_OK ||
        bmi08g_set_meas_conf(&dev_) != BMI08_OK) {
        return false;
    }

    return true;
}

void Bmi088::start()
{
    xTaskCreate(&Bmi088::pollTaskTramp, "bmi_acq", 512, this,
                configMAX_PRIORITIES - 1, nullptr);
}

void Bmi088::pollTaskTramp(void *arg)
{
    static_cast<Bmi088 *>(arg)->pollTask();
}

void Bmi088::pollTask()
{
    struct bmi08_sensor_data a;
    struct bmi08_sensor_data g;

    for (;;) {
        if (bmi088_mma_get_data(&a, &dev_) == BMI08_OK &&
            bmi08g_get_data(&g, &dev_) == BMI08_OK) {

            /* 芯片原始轴物理量(零偏统一在机体系扣,见下方 imu_calib_apply) */
            float acc[3] = {
                bmi_accel_to_mps2((float)a.x),
                bmi_accel_to_mps2((float)a.y),
                bmi_accel_to_mps2((float)a.z),
            };
            float gyr[3] = {
                bmi_gyro_to_rads((float)g.x),
                bmi_gyro_to_rads((float)g.y),
                bmi_gyro_to_rads((float)g.z),
            };

            /* 传感器系 → 机体系(统一映射,accel/gyro 各一套;标定改 imu_frame.c) */
            imu_axis_apply(&IMU_MAP_BMI_ACCEL, acc);
            imu_axis_apply(&IMU_MAP_BMI_GYRO,  gyr);
            for (int i = 0; i < 3; ++i) {
                data_.accel[i] = acc[i];
                data_.gyro[i]  = gyr[i];
            }

            /* 板子系零偏扣除(放置旋转之前,与平放/竖放无关;未标定时不动) */
            imu_calib_apply(data_.gyro, data_.accel);

            /* 板子放置旋转(平放=恒等;竖放见 imu_orientation.h) */
            imu_orient_apply(data_.gyro);
            imu_orient_apply(data_.accel);

            data_.temp      = 0.0f;   /* TODO: bmi08a_get_sensor_temperature(可选) */
            data_.timestamp = HAL_GetTick() * 1000U;   /* us */
            sample_count_++;
        }
        vTaskDelay(1);   /* ~1kHz 轮询 */
    }
}
