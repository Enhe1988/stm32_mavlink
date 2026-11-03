/** Unified IMU selection, board transforms and snapshot publication. */
#include "imu_api.h"
#include "imu_internal.h"
#include "imu_selftest.h"
#include "imu_frame.h"
#include "imu_calib.h"
#include "imu_orientation.h"
#include "icm45686.h"
#include "icm42688p.h"
#include "icm_acq.h"
#include "bmi088.h"
#include "main.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

volatile uint8_t g_imu_detected_mask;
volatile imu_type_t g_imu_active_type = IMU_TYPE_NONE;
volatile imu_selftest_t g_imu_selftest;
volatile imu_status_t g_imu_status;

static bool s_initialized;
static ImuData s_data[2];
static volatile uint8_t s_published;

/* 统一驱动接口与硬件适配
 * 样机 A/B/C 是构型配置，ICM/BMI 是传感器选择，两者独立。
 * 全部读取身份寄存器再按 45686 > 42688 > BMI088 选择，初始化失败还会向低优先级回退。
 * 上层不需要知道当前具体芯片；诊断结果保留在 g_imu_selftest。 */
imu_type_t imu_detect(void)
{
    if (s_initialized) {
        return g_imu_active_type;
    }
    bool has45686 = icm45686_probe();
    bool has42688 = icm42688p_probe();
    bool has_bmi = bmi088_probe();
    g_imu_detected_mask = (has45686 ? IMU_DETECTED_ICM45686 : 0U)
                        | (has42688 ? IMU_DETECTED_ICM42688 : 0U)
                        | (has_bmi ? IMU_DETECTED_BMI088 : 0U);
    g_imu_active_type = has45686 ? IMU_TYPE_ICM45686
                      : has42688 ? IMU_TYPE_ICM42688
                      : has_bmi ? IMU_TYPE_BMI088 : IMU_TYPE_NONE;
    g_imu_selftest.icm45686_who_am_i = icm45686_who_am_i();
    g_imu_selftest.icm45686_ok = has45686;
    g_imu_selftest.icm42688_who_am_i = icm42688p_who_am_i();
    g_imu_selftest.icm42688_ok = has42688;
    g_imu_selftest.bmi_accel_chip_id = bmi088_accel_id();
    g_imu_selftest.bmi_gyro_chip_id = bmi088_gyro_id();
    g_imu_selftest.bmi_ok = has_bmi;
    g_imu_selftest.detected_mask = g_imu_detected_mask;
    g_imu_selftest.active_type = (uint8_t)g_imu_active_type;
    g_imu_selftest.init_ok = 0U;
    g_imu_selftest.done = 1U;
    return g_imu_active_type;
}

bool imu_init(void)
{
    if (s_initialized) {
        return true;
    }
    (void)imu_detect();
    if ((g_imu_detected_mask & IMU_DETECTED_ICM45686) != 0U) {
        g_imu_active_type = IMU_TYPE_ICM45686;
        s_initialized = icm45686_init();
        g_imu_selftest.icm45686_init_rc = icm45686_init_result();
    }
    if (!s_initialized && (g_imu_detected_mask & IMU_DETECTED_ICM42688) != 0U) {
        g_imu_active_type = IMU_TYPE_ICM42688;
        s_initialized = icm42688p_init();
        g_imu_selftest.icm42688_init_rc = icm42688p_init_result();
    }
    if (!s_initialized && (g_imu_detected_mask & IMU_DETECTED_BMI088) != 0U) {
        g_imu_active_type = IMU_TYPE_BMI088;
        s_initialized = bmi088_init();
    }
    if (!s_initialized) {
        g_imu_active_type = IMU_TYPE_NONE;
    }
    g_imu_selftest.active_type = (uint8_t)g_imu_active_type;
    g_imu_selftest.init_ok = s_initialized;
    return s_initialized;
}

void imu_start(void)
{
    if (!s_initialized || g_imu_status.running) {
        return;
    }
    int result;
    switch (g_imu_active_type) {
    case IMU_TYPE_ICM45686:
    case IMU_TYPE_ICM42688:
        result = icm_acq_start(g_imu_active_type);
        if (g_imu_active_type == IMU_TYPE_ICM45686) {
            g_imu_selftest.icm45686_init_rc = result;
        } else {
            g_imu_selftest.icm42688_init_rc = result;
        }
        break;
    case IMU_TYPE_BMI088:
        result = bmi088_start() ? INIT_OK : INIT_TASK_ERROR;
        break;
    default:
        return;
    }
    g_imu_status.start_result = result;
    g_imu_status.running = result == INIT_OK;
}

/* 传感器与上层解耦
 * 三款驱动已把原始值换算为 rad/s、m/s²；此处统一进行轴映射、板子系零偏和安装旋转。
 * 顺序不能互换：零偏表存于板子系，先旋转成飞行器系再扣表会用错轴。
 * 只有选中的一个采集任务会发布；先填另一份数据缓冲，再切发布索引，读方复制完整快照。 */
void imu_publish(ImuData *sample)
{
    /* Exactly one selected driver publishes, always from task context. */
    bool bmi = g_imu_active_type == IMU_TYPE_BMI088;
    imu_axis_apply(bmi ? &IMU_MAP_BMI_ACCEL : &IMU_MAP_ICM_ACCEL, sample->accel);
    imu_axis_apply(bmi ? &IMU_MAP_BMI_GYRO : &IMU_MAP_ICM_GYRO, sample->gyro);
    imu_calib_apply(sample->gyro, sample->accel);
    imu_orient_apply(sample->gyro);
    imu_orient_apply(sample->accel);

    uint8_t next = s_published ^ 1U;
    s_data[next] = *sample;
    taskENTER_CRITICAL();
    __DMB();
    s_published = next;
    ++g_imu_status.sample_count;
    taskEXIT_CRITICAL();
}

bool imu_read(ImuData *out)
{
    if (out == NULL) {
        return false;
    }
    taskENTER_CRITICAL();
    *out = s_data[s_published];
    bool valid = g_imu_status.sample_count != 0U;
    taskEXIT_CRITICAL();
    return valid;
}

const ImuData *imu_get_data(void)
{
    uint8_t index = s_published;
    __DMB();
    return &s_data[index];
}

imu_type_t imu_get_type(void) { return g_imu_active_type; }
void imu_selftest_run(void) { (void)imu_detect(); }

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == ICM_INT_Pin) {
        icm_acq_drdy_isr();
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi->Instance == SPI4) {
        icm_acq_done_isr();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi)
{
    if (spi->Instance == SPI4) {
        icm_acq_error_isr();
    }
}
