/**
  ********************************************************
  * @file       mavlink_rc.c
  * @brief      遥控/上位机端口(USART3 / COMM_0)派发逻辑。
  *             从原 mavlink.c 拆出:RC_CHANNELS_OVERRIDE → elrs_data,
  *             SET_ATTITUDE_TARGET(四元数)→ pc_data(欧拉角)。
  ********************************************************
  */
#include "mavlink_rc.h"
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static MavlinkRcSnapshot_t s_rc_snapshot;
static MavlinkPcSnapshot_t s_pc_snapshot;

void mavlink_rc_get_snapshot(MavlinkRcSnapshot_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_rc_snapshot;
    taskEXIT_CRITICAL();
}

void mavlink_pc_get_snapshot(MavlinkPcSnapshot_t *out)
{
    if (out == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *out = s_pc_snapshot;
    taskEXIT_CRITICAL();
}

static float float_Map(float input_value, float input_min, float input_max,
                       float output_min, float output_max)
{
    float output_value;
    if (input_value < input_min)
    {
        output_value = output_min;
    }
    else if (input_value > input_max)
    {
        output_value = output_max;
    }
    else
    {
        output_value = output_min + (input_value - input_min) *
                       (output_max - output_min) / (input_max - input_min);
    }
    return output_value;
}

static float float_Map_with_median(float input_value, float input_min, float input_max,
                                   float median, float output_min, float output_max)
{
    float output_median = (output_max - output_min) / 2 + output_min;
    if (input_min >= input_max || output_min >= output_max ||
        median <= input_min || median >= input_max)
    {
        return output_min;
    }

    if (input_value < median)
    {
        return float_Map(input_value, input_min, median, output_min, output_median);
    }
    else
    {
        return float_Map(input_value, median, input_max, output_median, output_max);
    }
}

/* 将上位机发来的四元数转换为欧拉角(度),并写入 pc_data。 */
static void Quaternion_to_Euler_Transform(const mavlink_set_attitude_target_t *packet,
                                          PC_Data_t *out)
{
    float siny_cosp = 2.0f * (packet->q[0] * packet->q[3] + packet->q[1] * packet->q[2]);
    float cosy_cosp = 1.0f - 2.0f * (packet->q[2] * packet->q[2] + packet->q[3] * packet->q[3]);
    out->yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);

    float sinp = 2.0f * (packet->q[0] * packet->q[1] + packet->q[2] * packet->q[3]);
    float cosp = 1.0f - 2.0f * (packet->q[1] * packet->q[1] + packet->q[2] * packet->q[2]);
    out->pitch = atan2f(sinp, cosp) * (180.0f / M_PI);

    sinp = 2.0f * (packet->q[0] * packet->q[2] - packet->q[3] * packet->q[1]);
    if (fabsf(sinp) >= 1.0f)
    {
        out->roll = copysignf(M_PI / 2.0f, sinp) * (180.0f / M_PI);
    }
    else
    {
        out->roll = asinf(sinp) * (180.0f / M_PI);
    }

    out->thrust = packet->thrust; // 传入油门
}

/* ELRS/MAVLink 遥控和上位机目标
 * RC_CHANNELS_OVERRIDE 承载接收机通道，不在这里解析 ELRS 射频或原生 CRSF。
 * SET_ATTITUDE_TARGET 承载上位机姿态/推力；整帧 decode 后先填局部 next 再原子发布。
 * update_ms 供失联保护，frame_count 供遥控前馈识别新帧，valid 区分尚未收到与有效零值。
 * 注意：历史 Quaternion_to_Euler_Transform 的 Pitch/Roll 字段与公式仍待核对修正。 */
void mavlink_rc_dispatch(const mavlink_message_t *msg)
{
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_HEARTBEAT:
        break;

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
    {
        mavlink_set_attitude_target_t packet;
        MavlinkPcSnapshot_t next = {0};
        mavlink_msg_set_attitude_target_decode(msg, &packet);
        Quaternion_to_Euler_Transform(&packet, &next.data);
        next.update_ms = HAL_GetTick();
        next.valid = 1U;

        taskENTER_CRITICAL();
        next.frame_count = s_pc_snapshot.frame_count + 1U;
        s_pc_snapshot = next;
        taskEXIT_CRITICAL();
        break;
    }

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
    {
        mavlink_rc_channels_override_t packet;
        MavlinkRcSnapshot_t next = {0};
        ELRS_Data *elrs_data = &next.data;
        mavlink_msg_rc_channels_override_decode(msg, &packet);
        elrs_data->channels[0]  = packet.chan1_raw;  // Roll (通常)
        elrs_data->channels[1]  = packet.chan2_raw;  // Pitch (通常)
        elrs_data->channels[2]  = packet.chan3_raw;  // Throttle (通常)
        elrs_data->channels[3]  = packet.chan4_raw;  // Yaw (通常)
        elrs_data->channels[4]  = packet.chan5_raw;  // Aux1
        elrs_data->channels[5]  = packet.chan6_raw;  // Aux2
        elrs_data->channels[6]  = packet.chan7_raw;  // Aux3
        elrs_data->channels[7]  = packet.chan8_raw;  // Aux4
        elrs_data->channels[8]  = packet.chan9_raw;
        elrs_data->channels[9]  = packet.chan10_raw;
        elrs_data->channels[10] = packet.chan11_raw;
        elrs_data->channels[11] = packet.chan12_raw;
        elrs_data->channels[12] = packet.chan13_raw;
        elrs_data->channels[13] = packet.chan14_raw;
        elrs_data->channels[14] = packet.chan15_raw;
        elrs_data->channels[15] = packet.chan16_raw;
        elrs_data->Left_X  = float_Map_with_median(elrs_data->channels[3], 1000, 2000, 1500, -100, 100);
        elrs_data->Left_Y  = float_Map_with_median(elrs_data->channels[2], 1000, 2000, 1500, 0, 100);
        elrs_data->Right_X = float_Map_with_median(elrs_data->channels[0], 1000, 2000, 1500, -100, 100);
        elrs_data->Right_Y = float_Map_with_median(elrs_data->channels[1], 1000, 2000, 1500, -100, 100);
        elrs_data->S1 = float_Map_with_median(elrs_data->channels[8], 1000, 2000, 1500, 0, 100);
        elrs_data->S2 = float_Map_with_median(elrs_data->channels[9], 1000, 2000, 1500, 0, 100);
        elrs_data->A = elrs_data->channels[10] > 1500 ? 1 : 0;
        elrs_data->B = (elrs_data->channels[5] < 1200) ? 0 : ((elrs_data->channels[5] > 1800) ? 2 : 1);
        elrs_data->C = (elrs_data->channels[6] < 1200) ? 0 : ((elrs_data->channels[6] > 1800) ? 2 : 1);
        elrs_data->D = elrs_data->channels[11] > 1500 ? 1 : 0;
        elrs_data->E = (elrs_data->channels[4] < 1200) ? 1 : ((elrs_data->channels[4] > 1800) ? 2 : 0);
        elrs_data->F = (elrs_data->channels[7] < 1200) ? 1 : ((elrs_data->channels[7] > 1800) ? 2 : 0);

        next.update_ms = HAL_GetTick();
        next.valid = 1U;
        taskENTER_CRITICAL();
        next.frame_count = s_rc_snapshot.frame_count + 1U;
        s_rc_snapshot = next;
        taskEXIT_CRITICAL();
        break;
    }

    default:
        break;
    }
}
