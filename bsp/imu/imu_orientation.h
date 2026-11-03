/**
 * @file    imu_orientation.h
 * @brief   板子放置方式 → 机体系旋转(平放/竖放,编译期选,见 bsp_config.h)。
 *          对一个三维矢量(gyro 或 accel)就地施加旋转;在各 IMU 驱动填完
 *          ImuData 的最后一步,对 gyro、accel 各调一次。
 *
 *          这一级与“每芯片安装映射”分离:安装映射在各驱动内部(把芯片轴对到
 *          板子轴),这里只处理整块板子怎么放(平/竖)。零偏在板子系、放置旋转之前减。
 */
#ifndef IMU_ORIENTATION_H
#define IMU_ORIENTATION_H

#include "bsp_config.h"


static inline void imu_orient_apply(float v[3])
{
#if defined(BOARD_ORIENT_VERTICAL)
    /* 竖放(实测标定):板子系 B → 飞行器系 V:
     *   V_x = -B_z,  V_y = +B_x,  V_z = -B_y  (纯旋转,accel/gyro 共用) */
    float s0 = v[0], s1 = v[1], s2 = v[2];
    v[0] = -s2;
    v[1] =  s0;
    v[2] = -s1;
#else
    /* 平放:恒等 */
    (void)v;
#endif
}

/** @brief imu_orient_apply 的逆:飞行器系 V → 板子系 B。
 *         校准时把测到的机体系零偏转回板子系(与放置无关)存储。 */
static inline void imu_orient_unapply(float v[3])
{
#if defined(BOARD_ORIENT_VERTICAL)
    /* apply 的逆:B_x=V_y, B_y=-V_z, B_z=-V_x */
    float s0 = v[0], s1 = v[1], s2 = v[2];
    v[0] =  s1;
    v[1] = -s2;
    v[2] = -s0;
#else
    (void)v;
#endif
}


#endif /* IMU_ORIENTATION_H */
