/**
 * @file    imu_api.h
 * @brief   IMU 抽象层对外 C 接口。
 *          INS_task 等上层只认这里的 ImuData 与三个函数,与具体芯片解耦。
 *          启动时自动探测并按 ICM45686 > ICM42688 > BMI088 选择。
 */
#ifndef IMU_API_H
#define IMU_API_H

#include <stdint.h>
#include <stdbool.h>


/**
 * @brief 统一 IMU 数据。已完成量纲换算 + 板子旋转 + 零偏标定。
 *        三款传感器共用；采样时间戳单位为微秒，32 位自然回绕。
 */
typedef struct {
    float    gyro[3];   /* rad/s,机体坐标系 */
    float    accel[3];  /* m/s^2,机体坐标系 */
    float    temp;      /* °C */
    uint32_t timestamp; /* 采样时刻(us) */
} ImuData;

typedef enum {
    IMU_TYPE_NONE = 0,
    IMU_TYPE_ICM45686,
    IMU_TYPE_ICM42688,
    IMU_TYPE_BMI088,
} imu_type_t;

enum {
    IMU_DETECTED_ICM45686 = (1U << 0),
    IMU_DETECTED_ICM42688 = (1U << 1),
    IMU_DETECTED_BMI088   = (1U << 2),
};

/** 调试器可直接观察：本次启动检测结果和最终选中的 IMU。 */
extern volatile uint8_t g_imu_detected_mask;
extern volatile imu_type_t g_imu_active_type;

/** 所选设备的运行诊断；供调试器观察。 */
typedef struct {
    uint32_t sample_count;
    uint32_t drdy_count;
    uint32_t missed_drdy_count;
    uint32_t dma_start_error_count;
    uint32_t spi_error_count;
    uint32_t invalid_sample_count;
    int32_t start_result; /* 0=成功，-1=SPI，-5=任务创建失败 */
    uint8_t running;
} imu_status_t;
extern volatile imu_status_t g_imu_status;

/** @brief 读取各芯片身份寄存器并按固定优先级选择，不执行传感器配置。 */
imu_type_t imu_detect(void);

/** @brief 自动探测，按优先级选择第一个能完成初始化的 IMU。 */
bool imu_init(void);

/** @brief 启动采集:ICM 武装 EXTI+DMA / BMI088 起轮询任务。 */
void imu_start(void);

/** @brief 取最新一帧标定后数据。调用后应立即复制，不要跨采样周期缓存指针。 */
const ImuData *imu_get_data(void);

/** @brief 任务中复制完整快照；尚未发布有效采样时返回 false 并复制零值。
 *         允许重复读取同一帧，通过 timestamp 判断是否为新数据。 */
bool imu_read(ImuData *out);

/** @brief 返回当前选中的型号；未检测到时返回 IMU_TYPE_NONE。 */
imu_type_t imu_get_type(void);


#endif /* IMU_API_H */
