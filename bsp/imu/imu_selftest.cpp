/**
 * @file    imu_selftest.cpp
 * @brief   保留旧自检入口；实际探测和优先级选择由 imu_api 统一完成。
 */
#include "imu_selftest.h"
#include "imu_api.h"

volatile imu_selftest_t g_imu_selftest = {};

extern "C" void imu_selftest_run(void)
{
    (void)imu_detect();
}
