/**
 * @file    imu_selftest.cpp
 * @brief   IMU 通信自检实现。ICM 走自研最小驱动(init+WHO_AM_I),
 *          BMI088 走现有 Bosch C 驱动读 chip id。结果写入 g_imu_selftest。
 */
#include "imu_selftest.h"
#include "Icm42688p.hpp"

extern "C" {
#include "bmi08x.h"       // BMI088_ACCEL_CHIP_ID
#include "bmi08.h"        // bmi08g_init
#include "bmi088_mm.h"    // bmi088_mma_init
#include "common.h"       // bmi08_interface_init, BMI08_SPI_INTF
#include "bmi08_defs.h"   // BMI08_OK, BMI08_GYRO_CHIP_ID
}

volatile imu_selftest_t g_imu_selftest = {0};

static void icm_selftest(void)
{
    /* 在真正 imu_init 之前用临时实例执行完整寄存器初始化与 WHO_AM_I 检查。
     * 不调用 start()，不会创建解析任务或开放 DATA_RDY 中断。 */
    static Icm42688p icm;
    bool ok = icm.init();
    g_imu_selftest.icm_who_am_i = icm.whoAmI();
    g_imu_selftest.icm_init_rc  = (int32_t)icm.initRc();
    g_imu_selftest.icm_ok       = ok ? 1U : 0U;
}

static void bmi088_selftest(void)
{
    struct bmi08_dev dev = {0};
    int8_t rslt = bmi08_interface_init(&dev, BMI08_SPI_INTF);

    if (rslt == BMI08_OK) {
        (void)bmi088_mma_init(&dev);   /* 读 accel chip id → dev.accel_chip_id */
        (void)bmi08g_init(&dev);       /* 读 gyro  chip id → dev.gyro_chip_id  */
    }

    g_imu_selftest.bmi_accel_chip_id = dev.accel_chip_id;
    g_imu_selftest.bmi_gyro_chip_id  = dev.gyro_chip_id;
    g_imu_selftest.bmi_ok =
        ((dev.accel_chip_id == BMI088_ACCEL_CHIP_ID) &&
         (dev.gyro_chip_id == BMI08_GYRO_CHIP_ID))
            ? 1U
            : 0U;
}

extern "C" void imu_selftest_run(void)
{
    icm_selftest();
    bmi088_selftest();
    g_imu_selftest.done = 1U;
}
