/**
 * @file    Bmi088.hpp
 * @brief   BMI088 驱动(包现有 Bosch bmi08a/g C 驱动 + 轮询任务)。
 *          中断未引出 → 保持轮询(做法A)。填统一 ImuData(rad/s, m/s²)。
 */
#ifndef BMI088_HPP
#define BMI088_HPP

#include "imu.hpp"

extern "C" {
#include "bmi08_defs.h"   // struct bmi08_dev / bmi08_sensor_data
}

class Bmi088 : public ImuDriver {
public:
    bool init() override;
    void start() override;
    const ImuData &getData() const override { return data_; }
    uint32_t sampleCount() const { return sample_count_; }  /* 轮询累计帧数 */

private:
    static void pollTaskTramp(void *arg);
    void        pollTask();

    struct bmi08_dev   dev_{};
    ImuData            data_{};
    volatile uint32_t  sample_count_ = 0;
};

#endif /* BMI088_HPP */
