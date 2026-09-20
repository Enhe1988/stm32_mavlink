/**
 * @file    imu_api.cpp
 * @brief   IMU 抽象层对外 C 接口 + 上电自动探测和运行时选择。
 */
#include "imu_api.h"
#include "imu.hpp"
#include "Icm45686.hpp"
#include "Icm42688p.hpp"
#include "Bmi088.hpp"
#include "icm_platform.h"
#include "imu_selftest.h"

extern "C" {
#include "main.h"
#include "spi.h"
}

namespace {

Icm45686 g_icm45686;
Icm42688p g_icm42688;
Bmi088 g_bmi088;

ImuDriver *s_active = nullptr;
bool s_initialized = false;
const ImuData s_empty_data = {};

void selectActive(imu_type_t type)
{
    switch (type) {
    case IMU_TYPE_ICM45686:
        s_active = &g_icm45686;
        break;
    case IMU_TYPE_ICM42688:
        s_active = &g_icm42688;
        break;
    case IMU_TYPE_BMI088:
        s_active = &g_bmi088;
        break;
    default:
        s_active = nullptr;
        break;
    }
}

} /* namespace */

extern "C" {

volatile uint8_t g_imu_detected_mask = 0U;
volatile imu_type_t g_imu_active_type = IMU_TYPE_NONE;

imu_type_t imu_detect(void)
{
    if (s_initialized) {
        return g_imu_active_type;
    }

    /* DWT 供后续 ICM 初始化使用；MPU 配置同时覆盖同段内的 UART DMA 缓冲。 */
    icm_platform_timer_init();
    icm_platform_dma_region_init();

    const bool has_icm45686 = g_icm45686.probe();
    const bool has_icm42688 = g_icm42688.probe();
    const bool has_bmi088 = g_bmi088.probe();

    uint8_t mask = 0U;
    if (has_icm45686) {
        mask |= IMU_DETECTED_ICM45686;
    }
    if (has_icm42688) {
        mask |= IMU_DETECTED_ICM42688;
    }
    if (has_bmi088) {
        mask |= IMU_DETECTED_BMI088;
    }
    g_imu_detected_mask = mask;

    /* 固定优先级：ICM45686 > ICM42688 > BMI088。 */
    imu_type_t selected = IMU_TYPE_NONE;
    if (has_icm45686) {
        selected = IMU_TYPE_ICM45686;
    } else if (has_icm42688) {
        selected = IMU_TYPE_ICM42688;
    } else if (has_bmi088) {
        selected = IMU_TYPE_BMI088;
    }

    g_imu_active_type = selected;
    selectActive(selected);
    s_initialized = false;

    g_imu_selftest.icm45686_who_am_i = g_icm45686.whoAmI();
    g_imu_selftest.icm45686_ok = has_icm45686 ? 1U : 0U;
    g_imu_selftest.icm42688_who_am_i = g_icm42688.whoAmI();
    g_imu_selftest.icm42688_ok = has_icm42688 ? 1U : 0U;
    g_imu_selftest.bmi_accel_chip_id = g_bmi088.accelChipId();
    g_imu_selftest.bmi_gyro_chip_id = g_bmi088.gyroChipId();
    g_imu_selftest.bmi_ok = has_bmi088 ? 1U : 0U;
    g_imu_selftest.detected_mask = mask;
    g_imu_selftest.active_type = static_cast<uint8_t>(selected);
    g_imu_selftest.init_ok = 0U;
    g_imu_selftest.done = 1U;

    return selected;
}

bool imu_init(void)
{
    if (s_initialized) {
        return true;
    }

    if (imu_detect() == IMU_TYPE_NONE) {
        return false;
    }

    /* 按优先级选择第一个能完成完整初始化的设备，并保留高优先级失败诊断。 */
    if ((g_imu_detected_mask & IMU_DETECTED_ICM45686) != 0U) {
        g_imu_active_type = IMU_TYPE_ICM45686;
        selectActive(g_imu_active_type);
        s_initialized = s_active->init();
        g_imu_selftest.icm45686_init_rc = static_cast<int32_t>(g_icm45686.initRc());
    }

    if (!s_initialized &&
        ((g_imu_detected_mask & IMU_DETECTED_ICM42688) != 0U)) {
        g_imu_active_type = IMU_TYPE_ICM42688;
        selectActive(g_imu_active_type);
        s_initialized = s_active->init();
        g_imu_selftest.icm42688_init_rc = static_cast<int32_t>(g_icm42688.initRc());
    }

    if (!s_initialized &&
        ((g_imu_detected_mask & IMU_DETECTED_BMI088) != 0U)) {
        g_imu_active_type = IMU_TYPE_BMI088;
        selectActive(g_imu_active_type);
        s_initialized = s_active->init();
    }

    if (!s_initialized) {
        g_imu_active_type = IMU_TYPE_NONE;
        selectActive(IMU_TYPE_NONE);
    }

    g_imu_selftest.active_type = static_cast<uint8_t>(g_imu_active_type);
    g_imu_selftest.init_ok = s_initialized ? 1U : 0U;
    return s_initialized;
}

void imu_start(void)
{
    if (s_initialized && s_active != nullptr) {
        s_active->start();
    }
}

const ImuData *imu_get_data(void)
{
    return (s_active != nullptr) ? &s_active->getData() : &s_empty_data;
}

imu_type_t imu_get_type(void)
{
    return g_imu_active_type;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin != ICM_INT_Pin) {
        return;
    }

    if (g_imu_active_type == IMU_TYPE_ICM45686) {
        Icm45686::onDrdyISR();
    } else if (g_imu_active_type == IMU_TYPE_ICM42688) {
        Icm42688p::onDrdyISR();
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi->Instance != SPI4) {
        return;
    }

    if (g_imu_active_type == IMU_TYPE_ICM45686) {
        Icm45686::onSpiDmaDoneISR();
    } else if (g_imu_active_type == IMU_TYPE_ICM42688) {
        Icm42688p::onSpiDmaDoneISR();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi)
{
    if (spi->Instance != SPI4) {
        return;
    }

    if (g_imu_active_type == IMU_TYPE_ICM45686) {
        Icm45686::onSpiErrorISR();
    } else if (g_imu_active_type == IMU_TYPE_ICM42688) {
        Icm42688p::onSpiErrorISR();
    }
}

} /* extern "C" */
