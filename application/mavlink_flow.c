/**
  ********************************************************
  * @file       mavlink_flow.c
  * @brief      光流/测距端口(USART2 / COMM_1)派发逻辑。
  *             用官方 decode 解析,v1/v2 帧同步/载荷零截断/CRC 由库自动处理。
  * @note       忽略 DISTANCE_SENSOR 的 horizontal_fov / vertical_fov(协议文档
  *             说明:发送端未初始化的脏栈数据,不可使用)。
  ********************************************************
  */
#include "mavlink_flow.h"
#include "main.h"   /* HAL_GetTick */
#include "INS_task.h"
#include "bsp_config.h"
#include "optical_flow.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stddef.h>

static OpticalFlow_Data_t flow_data;

const OpticalFlow_Data_t *get_optical_flow_point(void)
{
    return &flow_data;
}

bool mavlink_flow_get_snapshot(OpticalFlow_Data_t *out, uint32_t now)
{
    if (out == NULL) return false;
    taskENTER_CRITICAL();
    *out = flow_data;
    taskEXIT_CRITICAL();
    out->online = out->online && (uint32_t)(now - out->last_update_ms) <= FLOW_TIMEOUT_MS;
    out->velocity_valid = out->velocity_valid &&
        (uint32_t)(now - out->flow_update_ms) <= FLOW_TIMEOUT_MS;
    out->height_valid = out->height_valid &&
        (uint32_t)(now - out->height_update_ms) <= FLOW_TIMEOUT_MS;
    if (!out->velocity_valid) out->vx_body_mps = out->vy_body_mps = NAN;
    if (!out->height_valid) out->height_m = NAN;
    return out->flow_frame_count != 0U || out->range_frame_count != 0U;
}

static void update_height(OpticalFlow_Data_t *next, float distance,
                          const IMU_data_t *imu, bool attitude_valid, uint32_t now)
{
    next->height_m = NAN;
    next->height_update_ms = now;
    next->height_valid = attitude_valid && optical_flow_height(distance,
        imu->INS_angle[INS_ROLL_ADDRESS_OFFSET] * DEG_TO_RAD,
        imu->INS_angle[INS_PITCH_ADDRESS_OFFSET] * DEG_TO_RAD, &next->height_m);
}

/* 光流/ToF 采集处理
 * 整帧解码后填局部 next，再原子发布原始观测及估计；仅供遥测，未进入控制闭环。
 * 光流帧自身的 distance 与该帧配套；不借用可能陈旧的 DISTANCE_SENSOR 距离。
 * 旋转补偿使用接收时 INS 角速度近似，不读取发送端为 NaN 的 gyro 字段。 */
void mavlink_flow_dispatch(const mavlink_message_t *msg)
{
    if (msg == NULL || (msg->msgid != MAVLINK_MSG_ID_OPTICAL_FLOW_RAD &&
                        msg->msgid != MAVLINK_MSG_ID_DISTANCE_SENSOR)) return;
    IMU_data_t imu;
    bool attitude_valid = ins_get_snapshot(&imu);
    /* 先复制后取当前时间，避免被高优先级 INS 抢占后快照时间晚于 now。 */
    uint32_t now = HAL_GetTick();
    attitude_valid = attitude_valid &&
        (uint32_t)(now - imu.update_ms) <= CONTROL_IMU_TIMEOUT_MS;
    OpticalFlow_Data_t next;
    (void)mavlink_flow_get_snapshot(&next, now);
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:   /* 106 */
    {
        mavlink_optical_flow_rad_t flow;
        mavlink_msg_optical_flow_rad_decode(msg, &flow);
        next.flow_x_rad         = flow.integrated_x;
        next.flow_y_rad         = flow.integrated_y;
        next.integration_time_s = (float)flow.integration_time_us / 1.0e6f;
        next.ground_distance_m  = flow.distance;
        next.flow_quality       = flow.quality;
        next.flow_update_ms = now;
        next.flow_frame_count++;
        update_height(&next, flow.distance, &imu, attitude_valid, now);
        float integrated[2] = {flow.integrated_x, flow.integrated_y};
        float velocity[2] = {NAN, NAN};
        next.velocity_valid = next.height_valid && flow.quality >= FLOW_MIN_QUALITY &&
            optical_flow_velocity(integrated, next.integration_time_s, flow.distance,
                                  imu.INS_gyro, velocity);
        next.vx_body_mps = velocity[0];
        next.vy_body_mps = velocity[1];
        break;
    }

    case MAVLINK_MSG_ID_DISTANCE_SENSOR:    /* 132 */
    {
        mavlink_distance_sensor_t dist;
        mavlink_msg_distance_sensor_decode(msg, &dist);
        next.range_cm = dist.current_distance;  /* 单位:厘米 */
        next.range_update_ms = now;
        next.range_frame_count++;
        /* 协议中的 max_distance=4000 与“4米”矛盾，不将其作为可信量程。
         * 实际工作范围由 FLOW_MIN/MAX_DISTANCE_M 控制，向下安装已由用户确认。 */
        update_height(&next, dist.current_distance * 0.01f, &imu, attitude_valid, now);
        break;
    }

    default:
        break;
    }
    next.last_update_ms = now;
    next.online = 1U;
    taskENTER_CRITICAL();
    flow_data = next;
    taskEXIT_CRITICAL();
}
