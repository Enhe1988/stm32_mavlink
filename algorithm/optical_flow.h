#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <stdbool.h>

/* 下视、光轴沿机体 +Z。distance 为沿光轴斜距，height 为向上为正的离地高度。 */
bool optical_flow_height(float distance_m, float roll_rad, float pitch_rad, float *height_m);

/* 近光轴平移速度估计，机体 X前/Y右，单位 m/s。
 * gyro 为该积分窗口的近似角速度 rad/s；当前调用方使用接收时 INS 快照，
 * 不等同于同步的陀螺积分，快速角运动和传感器杆臂误差需实板评估。 */
bool optical_flow_velocity(const float integrated_rad[2], float integration_s,
                           float distance_m, const float gyro_rads[2], float velocity_mps[2]);

#endif
