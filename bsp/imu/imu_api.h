/**
 * @file    imu_api.h
 * @brief   IMU 抽象层对外 C 接口。
 *          INS_task 等上层只认这里的 ImuData 与三个函数,与具体芯片解耦。
 *          切换 IMU = 改 bsp_config.h 一行 #define,本头不变。
 */
#ifndef IMU_API_H
#define IMU_API_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 统一 IMU 数据。已完成量纲换算 + 板子旋转 + 零偏标定。
 *        C 与 C++ 共用同一定义,内存布局一致。
 */
typedef struct {
    float    gyro[3];   /* rad/s,机体坐标系 */
    float    accel[3];  /* m/s^2,机体坐标系 */
    float    temp;      /* °C */
    uint32_t timestamp; /* 采样时刻(us 或 tick,由驱动定义) */
} ImuData;

/** @brief 软复位 + 配置 + 自检(读 WHO_AM_I)。调度器启动前调用。 */
bool imu_init(void);

/** @brief 启动采集:ICM 武装 EXTI+DMA / BMI088 起轮询任务。 */
void imu_start(void);

/** @brief 取最新一帧标定后数据。指针长期有效,内容随采集刷新。 */
const ImuData *imu_get_data(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_API_H */
