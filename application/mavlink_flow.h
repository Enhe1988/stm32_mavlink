/**
  ********************************************************
  * @file       mavlink_flow.h
  * @brief      光流+测距 MAVLink 端口(USART2 / MAVLINK_COMM_1)的数据与派发。
  *             数据源为 AT32F421(PMW3901 光流 + VL53L1X 测距),MAVLink v2 封装,
  *             50Hz 成对发送 OPTICAL_FLOW_RAD(106) 与 DISTANCE_SENSOR(132)。
  *             协议详见仓库根目录「光流测距_串口数据协议说明.md」。
  ********************************************************
  */
#ifndef __MAVLINK_FLOW_H__
#define __MAVLINK_FLOW_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "common/mavlink.h"

typedef struct
{
    /* 来自 OPTICAL_FLOW_RAD(106) */
    float    flow_x_rad;        /* integrated_x:X 累计光流角位移(弧度) */
    float    flow_y_rad;        /* integrated_y:Y 累计光流角位移(弧度) */
    float    integration_time_s;/* 积分时间(秒),由 integration_time_us 换算 */
    float    ground_distance_m; /* distance:对地距离(米) */
    uint8_t  flow_quality;      /* quality 0~255,0=本帧光流不可信 */

    /* 来自 DISTANCE_SENSOR(132) */
    uint16_t range_cm;          /* current_distance:测距值(厘米) */

    /* 链路健康 */
    uint32_t last_update_ms;    /* 最近一次收到任意一帧的 HAL_GetTick() */
    uint8_t  online;            /* 收到过数据置 1(消费端可结合 last_update_ms 判超时) */
} OpticalFlow_Data_t;

/**
  * @brief  获取光流/测距数据指针。
  */
const OpticalFlow_Data_t *get_optical_flow_point(void);

/**
  * @brief  COMM_1(USART2)整帧派发:OPTICAL_FLOW_RAD / DISTANCE_SENSOR。
  *         作为 MavlinkPort_t.on_message 回调注册。
  */
void mavlink_flow_dispatch(const mavlink_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_FLOW_H__ */
