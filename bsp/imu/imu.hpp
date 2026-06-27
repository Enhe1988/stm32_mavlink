/**
 * @file    imu.hpp
 * @brief   IMU 抽象基类(PX4 式)。各芯片驱动派生实现。
 */
#ifndef IMU_HPP
#define IMU_HPP

#include "imu_api.h"   // 复用同一 ImuData 定义(C/C++ 共享)

class ImuDriver {
public:
    virtual ~ImuDriver() = default;

    /** 软复位 + 配置 + 自检。成功返回 true。 */
    virtual bool init() = 0;

    /** 启动采集节拍(做法A:各驱动自管节拍)。 */
    virtual void start() = 0;

    /** 返回最新一帧标定后数据。 */
    virtual const ImuData &getData() const = 0;
};

#endif /* IMU_HPP */
