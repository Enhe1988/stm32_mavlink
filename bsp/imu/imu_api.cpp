/**
 * @file    imu_api.cpp
 * @brief   IMU 抽象层对外 C 接口 + 编译期选型。
 *          驱动对象静态构造(-fno-threadsafe-statics 已设),调度器启动前 init。
 */
#include "imu_api.h"
#include "imu.hpp"
#include "bsp_config.h"

#if defined(IMU_USE_ICM42688)
#include "Icm42688p.hpp"
static Icm42688p g_imu;
#elif defined(IMU_USE_BMI088)
#include "Bmi088.hpp"
static Bmi088 g_imu;
#endif

static ImuDriver *active = &g_imu;

extern "C" {

bool imu_init(void)
{
    return active->init();
}

void imu_start(void)
{
    active->start();
}

const ImuData *imu_get_data(void)
{
    return &active->getData();
}

} /* extern "C" */
