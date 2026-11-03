/**
  ********************************************************
  * @file       mavlink_flow.h
  * @brief      光流+测距 MAVLink 端口(USART2 / MAVLINK_COMM_1)的数据与派发。
  *             接收 MAVLink v1/v2 OPTICAL_FLOW_RAD(106) 与 DISTANCE_SENSOR(132),
  *             输入周期由模块实际输出决定,独立于上位机遥测的 20Hz。
  *             协议详见仓库根目录「光流测距_串口数据协议说明.md」。
  ********************************************************
  */
#ifndef __MAVLINK_FLOW_H__
#define __MAVLINK_FLOW_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "common/mavlink.h"

typedef struct
{
    /* 来自 OPTICAL_FLOW_RAD(106) */
    float    flow_x_rad;        /* integrated_x:X 累计光流角位移(弧度) */
    float    flow_y_rad;        /* integrated_y:Y 累计光流角位移(弧度) */
    float    integration_time_s;/* 积分时间(秒),由 integration_time_us 换算 */
    float    ground_distance_m; /* 106 distance:沿下视光轴的原始斜距(米) */
    uint8_t  flow_quality;      /* quality 0~255,0=本帧光流不可信 */

    /* 来自 DISTANCE_SENSOR(132) */
    uint16_t range_cm;          /* 132 current_distance:原始斜距(厘米) */

    /* 链路健康 */
    uint32_t last_update_ms;    /* 最近一次收到任意一帧的 HAL_GetTick() */
    uint8_t  online;            /* 收到过数据置 1(消费端可结合 last_update_ms 判超时) */

    uint32_t flow_update_ms;    /* 两种报文独立计时，测距不能掩盖光流断流 */
    uint32_t range_update_ms;
    uint32_t flow_frame_count;
    uint32_t range_frame_count;
    float vx_body_mps;         /* 机体 X前，光流近悬停估计 */
    float vy_body_mps;         /* 机体 Y右 */
    float height_m;            /* 距地面，向上为正；不是 NED 的 z */
    uint32_t height_update_ms;
    uint8_t velocity_valid;
    uint8_t height_valid;
} OpticalFlow_Data_t;

/**
  * @brief  兼容旧接口；不保证跨任务快照一致性，也不主动检查超时。
  *         新代码使用 mavlink_flow_get_snapshot。
  */
const OpticalFlow_Data_t *get_optical_flow_point(void);

/* 任务上下文原子复制，按 now_ms 清除超时有效位并将无效结果置 NaN。
 * 返回是否至少收到过一条支持的消息；数值是否可用应检查各自 valid。 */
bool mavlink_flow_get_snapshot(OpticalFlow_Data_t *out, uint32_t now_ms);

/**
  * @brief  COMM_1(USART2)整帧派发:OPTICAL_FLOW_RAD / DISTANCE_SENSOR。
  *         作为 MavlinkPort_t.on_message 回调注册。
  */
void mavlink_flow_dispatch(const mavlink_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_FLOW_H__ */
