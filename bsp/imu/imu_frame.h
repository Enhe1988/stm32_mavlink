/**
 * @file    imu_frame.h
 * @brief   传感器系 → 机体(板子)系 轴向映射,统一处理各款 IMU。
 *
 *  映射定义:body[i] = sign[i] * sensor[axis[i]]  (axis: 0=x,1=y,2=z)
 *  在统一发布函数填完芯片原始轴物理量后、板子放置旋转(imu_orientation.h)之前调用。
 *
 *  级联:芯片原始轴 →[本层 sensor→body 映射]→ 板子系 →[imu_orient_apply 平/竖]→ 机体系
 *
 *  全部映射常量集中在 imu_frame.c —— 标定只改那一个文件。
 *  注:BMI088 的 accel/gyro 是两颗 die,封装内轴向不同,故各一套;ICM 单 die 共用一套。
 */
#ifndef IMU_FRAME_H
#define IMU_FRAME_H


typedef struct {
    unsigned char axis[3];   /* 取哪个传感器轴:0=x, 1=y, 2=z */
    float         sign[3];   /* +1.0f / -1.0f */
} imu_axis_map_t;

/** @brief 就地把映射施加到一个三维矢量(accel 或 gyro)。 */
static inline void imu_axis_apply(const imu_axis_map_t *m, float v[3])
{
    const float src[3] = { v[0], v[1], v[2] };
    for (int i = 0; i < 3; ++i) {
        v[i] = m->sign[i] * src[m->axis[i]];
    }
}

/* ── 各芯片映射(定义见 imu_frame.c,标定改那里)──
 * accel/gyro 分开:加速度用"轴朝下=+9.8"约定、陀螺用右手定则,两者符号通常不同,
 * 即便单 die 的 ICM 也需各一套。 */
extern const imu_axis_map_t IMU_MAP_ICM_ACCEL;  /* ICM-45686 / ICM-42688-P 加速度 */
extern const imu_axis_map_t IMU_MAP_ICM_GYRO;   /* ICM-45686 / ICM-42688-P 陀螺 */
extern const imu_axis_map_t IMU_MAP_BMI_ACCEL;  /* BMI088 加速度 */
extern const imu_axis_map_t IMU_MAP_BMI_GYRO;   /* BMI088 陀螺 */


#endif /* IMU_FRAME_H */
