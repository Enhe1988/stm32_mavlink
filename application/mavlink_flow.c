/**
  ********************************************************
  * @file       mavlink_flow.c
  * @brief      光流/测距端口(USART2 / COMM_1)派发逻辑。
  *             用官方 decode 解析,v2 帧同步/载荷零截断/CRC 由库自动处理。
  * @note       忽略 DISTANCE_SENSOR 的 horizontal_fov / vertical_fov(协议文档
  *             说明:发送端未初始化的脏栈数据,不可使用)。
  ********************************************************
  */
#include "mavlink_flow.h"
#include "main.h"   /* HAL_GetTick */

static OpticalFlow_Data_t flow_data;

const OpticalFlow_Data_t *get_optical_flow_point(void)
{
    return &flow_data;
}

void mavlink_flow_dispatch(const mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:   /* 106 */
    {
        mavlink_optical_flow_rad_t flow;
        mavlink_msg_optical_flow_rad_decode(msg, &flow);
        flow_data.flow_x_rad         = flow.integrated_x;
        flow_data.flow_y_rad         = flow.integrated_y;
        flow_data.integration_time_s = (float)flow.integration_time_us / 1.0e6f;
        flow_data.ground_distance_m  = flow.distance;
        flow_data.flow_quality       = flow.quality;
        flow_data.last_update_ms     = HAL_GetTick();
        flow_data.online             = 1;
        break;
    }

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:    /* 132 */
    {
        mavlink_distance_sensor_t dist;
        mavlink_msg_distance_sensor_decode(msg, &dist);
        flow_data.range_cm       = dist.current_distance;  /* 单位:厘米 */
        flow_data.last_update_ms = HAL_GetTick();
        flow_data.online         = 1;
        break;
    }

    default:
        break;
    }
}
