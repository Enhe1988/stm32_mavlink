#include "mavlink_telemetry.h"
#include "mavlink_flow.h"
#include "INS_task.h"
#include "bsp_config.h"
#include "main.h"
#include <math.h>
#include <stddef.h>

uint16_t mavlink_telemetry_build(uint8_t *buffer, uint16_t capacity, uint32_t now)
{
    if (buffer == NULL || capacity < MAVLINK_TELEMETRY_BUFFER_SIZE) return 0U;
    IMU_data_t imu;
    OpticalFlow_Data_t flow;
    bool attitude_valid = ins_get_snapshot(&imu) &&
        (uint32_t)(HAL_GetTick() - imu.update_ms) <= CONTROL_IMU_TIMEOUT_MS;
    for (unsigned i = 0; i < 3U; ++i)
        attitude_valid = attitude_valid && isfinite(imu.INS_angle[i]) && isfinite(imu.INS_gyro[i]);
    (void)mavlink_flow_get_snapshot(&flow, now);
    bool velocity_valid = attitude_valid && flow.velocity_valid;
    bool height_valid = attitude_valid && flow.height_valid;
    int32_t flags = (attitude_valid ? TELEMETRY_ATTITUDE_VALID : 0U) |
                    (velocity_valid ? TELEMETRY_VELOCITY_VALID : 0U) |
                    (height_valid ? TELEMETRY_HEIGHT_VALID : 0U);

    mavlink_message_t msg;
    uint16_t length = 0;
    /* INS 内部 yaw/pitch/roll 单位 deg；ATTITUDE 严格按 roll/pitch/yaw rad。 */
    mavlink_msg_attitude_pack_chan(mavlink_system.sysid, mavlink_system.compid, MAVLINK_COMM_0,
        &msg, now,
        attitude_valid ? imu.INS_angle[INS_ROLL_ADDRESS_OFFSET] * DEG_TO_RAD : NAN,
        attitude_valid ? imu.INS_angle[INS_PITCH_ADDRESS_OFFSET] * DEG_TO_RAD : NAN,
        attitude_valid ? imu.INS_angle[INS_YAW_ADDRESS_OFFSET] * DEG_TO_RAD : NAN,
        attitude_valid ? imu.INS_gyro[0] : NAN,
        attitude_valid ? imu.INS_gyro[1] : NAN,
        attitude_valid ? imu.INS_gyro[2] : NAN);
    length += mavlink_msg_to_send_buffer(buffer + length, &msg);

    /* 数组固定 10B，满足生成库 name[10] 的整段复制要求。 */
    static const char names[3][10] = {"vx_body", "vy_body", "height"};
    float values[3] = {velocity_valid ? flow.vx_body_mps : NAN,
                       velocity_valid ? flow.vy_body_mps : NAN,
                       height_valid ? flow.height_m : NAN};
    for (unsigned i = 0; i < 3U; ++i) {
        mavlink_msg_named_value_float_pack_chan(mavlink_system.sysid, mavlink_system.compid,
            MAVLINK_COMM_0, &msg, now, names[i], values[i]);
        length += mavlink_msg_to_send_buffer(buffer + length, &msg);
    }
    static const char validity_name[10] = "telem_ok";
    mavlink_msg_named_value_int_pack_chan(mavlink_system.sysid, mavlink_system.compid,
        MAVLINK_COMM_0, &msg, now, validity_name, flags);
    length += mavlink_msg_to_send_buffer(buffer + length, &msg);
    return length;
}
