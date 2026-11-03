/** BMI088: Bosch C driver on SPI1, independent polling task. */
#include "bmi088.h"
#include "bsp_platform.h"
#include "imu_internal.h"
#include "bmi088_mm.h"
#include "bmi08.h"
#include "bmi08x.h"
#include "common.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

static struct bmi08_dev s_dev;
static TaskHandle_t s_poll_task;
static void poll_task(void *arg);

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

bool bmi088_probe(void)
{
    s_dev = (struct bmi08_dev){0};

    /* SPI1 接口 + delay api 绑定到 s_dev */
    if (bmi08_interface_init(&s_dev, BMI08_SPI_INTF) != BMI08_OK) {
        return false;
    }

    /* 两颗 die 都读一遍，单颗缺失时调试信息仍保留另一颗的 ID。 */
    const int8_t accel_result = bmi088_mma_init(&s_dev);
    const int8_t gyro_result = bmi08g_init(&s_dev);
    return (accel_result == BMI08_OK) && (gyro_result == BMI08_OK) &&
           (s_dev.accel_chip_id == BMI088_ACCEL_CHIP_ID) &&
           (s_dev.gyro_chip_id == BMI08_GYRO_CHIP_ID);
}

bool bmi088_init(void)
{
    const bool already_probed =
        (s_dev.accel_chip_id == BMI088_ACCEL_CHIP_ID) &&
        (s_dev.gyro_chip_id == BMI08_GYRO_CHIP_ID);
    if (!already_probed && !bmi088_probe()) {
        return false;
    }

    if (bmi08a_soft_reset(&s_dev) != BMI08_OK) {
        return false;
    }

    /* 加速度:Active / 800Hz / ±6g / Normal BW */
    s_dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    if (bmi08a_set_power_mode(&s_dev) != BMI08_OK) {
        return false;
    }
    s_dev.accel_cfg.odr   = BMI08_ACCEL_ODR_800_HZ;
    s_dev.accel_cfg.range = BMI088_MM_ACCEL_RANGE_6G;
    s_dev.accel_cfg.bw    = BMI08_ACCEL_BW_NORMAL;
    if (bmi08a_set_power_mode(&s_dev) != BMI08_OK ||
        bmi088_mma_set_meas_conf(&s_dev) != BMI08_OK) {
        return false;
    }

    /* 陀螺:Normal / 1000Hz / ±1000dps / 32Hz BW */
    s_dev.gyro_cfg.odr   = BMI08_GYRO_BW_116_ODR_1000_HZ;
    s_dev.gyro_cfg.range = BMI08_GYRO_RANGE_1000_DPS;
    s_dev.gyro_cfg.bw    = BMI08_GYRO_BW_32_ODR_100_HZ;
    s_dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    if (bmi08g_set_power_mode(&s_dev) != BMI08_OK ||
        bmi08g_set_meas_conf(&s_dev) != BMI08_OK) {
        return false;
    }

    return true;
}

bool bmi088_start(void)
{
    if (s_poll_task != NULL) {
        return true;
    }
    return xTaskCreate(poll_task, "bmi_acq", 512U, NULL,
                       configMAX_PRIORITIES - 1U, &s_poll_task) == pdPASS;
}

uint8_t bmi088_accel_id(void) { return s_dev.accel_chip_id; }
uint8_t bmi088_gyro_id(void) { return s_dev.gyro_chip_id; }

static void poll_task(void *arg)
{
    (void)arg;
    struct bmi08_sensor_data a;
    struct bmi08_sensor_data g;
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1U);
    configASSERT(period > 0);

    for (;;) {
        if (bmi088_mma_get_data(&a, &s_dev) == BMI08_OK &&
            bmi08g_get_data(&g, &s_dev) == BMI08_OK) {

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

            ImuData sample = {0};
            for (int i = 0; i < 3; ++i) {
                sample.accel[i] = acc[i];
                sample.gyro[i] = gyr[i];
            }
            sample.timestamp = (uint32_t)bsp_time_us();
            imu_publish(&sample);

        }
        vTaskDelayUntil(&wake, period); /* 1ms 绝对周期，避免累加 SPI 读取耗时。 */
    }
}
