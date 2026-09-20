/**
 * @file    imu_frame.c
 * @brief   各 IMU 的「传感器系 → 机体(板子)系」轴向映射常量,集中一处。
 *          标定坐标系时只改这个文件。映射语义见 imu_frame.h。
 */
#include "imu_frame.h"

/* ===== 标定中:全部恒等,g_imu_live 直接显示芯片原始轴,用于测量 =====
 * 测完后按实测把下面三组 axis/sign 改成目标机体系。 */

/* ICM-45686 / ICM-42688-P 共用贴片位置和方向。
 * 加速度(平放标定):body=(-raw_x, +raw_y, +raw_z)
 *   水平→(0,0,+9.8)、机头下→(+9.8,0,0)、右下→(0,+9.8,0) */
const imu_axis_map_t IMU_MAP_ICM_ACCEL = {
    { 0, 1, 2 },
    { -1.0f, 1.0f, 1.0f }
};

/* ICM 陀螺(平放标定):body=(+raw_x, -raw_y, -raw_z)
 *   右滚→+X、机头抬→+Y、右偏→+Z(右手定则) */
const imu_axis_map_t IMU_MAP_ICM_GYRO = {
    { 0, 1, 2 },
    { 1.0f, -1.0f, -1.0f }
};

/* BMI088 加速度(平放标定):body=(-raw_x, +raw_y, +raw_z)
 *   水平→(0,0,+9.8)、机头下→(+9.8,0,0)、右下→(0,+9.8,0) */
const imu_axis_map_t IMU_MAP_BMI_ACCEL = {
    { 0, 1, 2 },
    { -1.0f, 1.0f, 1.0f }
};

/* BMI088 陀螺(平放标定,与 accel 不同 die):body=(+raw_x, -raw_y, -raw_z)
 *   右滚→+X、机头抬→+Y、右偏→+Z(右手定则) */
const imu_axis_map_t IMU_MAP_BMI_GYRO = {
    { 0, 1, 2 },
    { 1.0f, -1.0f, -1.0f }
};
