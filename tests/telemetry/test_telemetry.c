#include "mavlink_flow.h"
#include "mavlink_telemetry.h"
#include "optical_flow.h"
#include "INS_task.h"
#include "bsp_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

mavlink_system_t mavlink_system = {1, 1};
static IMU_data_t imu;
static uint32_t now;
uint32_t HAL_GetTick(void) { return now; }
bool ins_get_snapshot(IMU_data_t *out) { *out = imu; return out->IMU_init_flag; }

static void near(float value, float expected)
{
    assert(isfinite(value));
    assert(fabsf(value - expected) < 0.0001f);
}

static void attitude(uint32_t timestamp)
{
    now = timestamp;
    memset(&imu, 0, sizeof(imu));
    imu.IMU_init_flag = true;
    imu.update_ms = now;
}

/* 构造给定机体角轴上的观测，允许同一回归测试编译交换轴/反号配置。 */
static void flow(float body_x_rad, float body_y_rad, float distance, unsigned quality, uint32_t dt_us)
{
    mavlink_optical_flow_rad_t packet = {0};
    float raw[2];
    raw[FLOW_BODY_X_AXIS] = body_x_rad / FLOW_BODY_X_SCALE;
    raw[FLOW_BODY_Y_AXIS] = body_y_rad / FLOW_BODY_Y_SCALE;
    packet.integrated_x = raw[0];
    packet.integrated_y = raw[1];
    packet.integrated_xgyro = packet.integrated_ygyro = packet.integrated_zgyro = NAN;
    packet.integration_time_us = dt_us;
    packet.distance = distance;
    packet.quality = (uint8_t)quality;
    mavlink_message_t message;
    mavlink_msg_optical_flow_rad_encode_chan(2, 88, MAVLINK_COMM_1, &message, &packet);
    mavlink_flow_dispatch(&message);
}

static void range(uint16_t cm)
{
    mavlink_distance_sensor_t packet = {0};
    packet.current_distance = cm;
    packet.max_distance = 4000;
    packet.orientation = 25;
    packet.horizontal_fov = packet.vertical_fov = NAN;
    mavlink_message_t message;
    mavlink_msg_distance_sensor_encode_chan(2, 88, MAVLINK_COMM_1, &message, &packet);
    mavlink_flow_dispatch(&message);
}

/* 从真实打包字节重新经过官方 parser，检查 CRC、截断、消息类型、字段及序号。 */
static void check_wire(unsigned flags, float vx, float vy, float height)
{
    uint8_t bytes[MAVLINK_TELEMETRY_BUFFER_SIZE + 8U];
    memset(bytes, 0xa5, sizeof(bytes));
    assert(mavlink_telemetry_build(NULL, sizeof(bytes), now) == 0U);
    assert(mavlink_telemetry_build(bytes, MAVLINK_TELEMETRY_BUFFER_SIZE - 1U, now) == 0U);
    uint16_t count = mavlink_telemetry_build(bytes, MAVLINK_TELEMETRY_BUFFER_SIZE, now);
    assert(count > 0 && count <= MAVLINK_TELEMETRY_BUFFER_SIZE);
    for (unsigned i = MAVLINK_TELEMETRY_BUFFER_SIZE; i < sizeof(bytes); ++i) assert(bytes[i] == 0xa5);
    mavlink_message_t message;
    mavlink_status_t status;
    unsigned frames = 0;
    uint8_t previous_seq = 0;
    for (unsigned i = 0; i < count; ++i) {
        if (!mavlink_parse_char(MAVLINK_COMM_2, bytes[i], &message, &status)) continue;
        assert(message.magic == MAVLINK_STX);
        assert(message.sysid == 1 && message.compid == 1);
        if (frames) assert(message.seq == (uint8_t)(previous_seq + 1U));
        previous_seq = message.seq;
        if (frames == 0U) {
            assert(message.msgid == MAVLINK_MSG_ID_ATTITUDE);
            mavlink_attitude_t decoded;
            mavlink_msg_attitude_decode(&message, &decoded);
            assert(decoded.time_boot_ms == now);
            if (flags & TELEMETRY_ATTITUDE_VALID) {
                near(decoded.roll, imu.INS_angle[2] * DEG_TO_RAD);
                near(decoded.pitch, imu.INS_angle[1] * DEG_TO_RAD);
                near(decoded.yaw, imu.INS_angle[0] * DEG_TO_RAD);
                near(decoded.rollspeed, imu.INS_gyro[0]);
                near(decoded.pitchspeed, imu.INS_gyro[1]);
                near(decoded.yawspeed, imu.INS_gyro[2]);
            } else {
                assert(isnan(decoded.roll) && isnan(decoded.pitch) && isnan(decoded.yaw));
                assert(isnan(decoded.rollspeed) && isnan(decoded.pitchspeed) && isnan(decoded.yawspeed));
            }
        } else if (frames < 4U) {
            assert(message.msgid == MAVLINK_MSG_ID_NAMED_VALUE_FLOAT);
            mavlink_named_value_float_t decoded;
            mavlink_msg_named_value_float_decode(&message, &decoded);
            static const char names[3][10] = {"vx_body", "vy_body", "height"};
            const float expected[3] = {vx, vy, height};
            assert(decoded.time_boot_ms == now);
            assert(memcmp(decoded.name, names[frames - 1U], 10) == 0);
            if (isnan(expected[frames - 1U])) assert(isnan(decoded.value));
            else near(decoded.value, expected[frames - 1U]);
        } else {
            assert(frames == 4U && message.msgid == MAVLINK_MSG_ID_NAMED_VALUE_INT);
            mavlink_named_value_int_t decoded;
            mavlink_msg_named_value_int_decode(&message, &decoded);
            assert(decoded.time_boot_ms == now);
            assert(strncmp(decoded.name, "telem_ok", 10) == 0);
            assert(decoded.value == (int32_t)flags);
        }
        ++frames;
    }
    assert(frames == 5U);
}

int main(void)
{
    OpticalFlow_Data_t snapshot;
    /* 未收到、未初始化时必须发 NaN，不能把零值伪装成有效测量。 */
    assert(!mavlink_flow_get_snapshot(&snapshot, 0U));
    assert(isnan(snapshot.vx_body_mps) && isnan(snapshot.height_m));
    check_wire(0U, NAN, NAN, NAN);

    attitude(100U);
    imu.INS_angle[0] = 90.0f; /* yaw 不应旋转机体速度到地理系 */
    flow(-0.02f, 0.01f, 2.0f, 100U, 20000U);
    check_wire(7U, 1.0f, 2.0f, 2.0f);
    (void)mavlink_flow_get_snapshot(&snapshot, now);
    assert(snapshot.flow_frame_count == 1 && snapshot.range_frame_count == 0);
    near(snapshot.integration_time_s, 0.02f);

    /* 纯旋转观测==gyro*dt，平移应为零；忽略发送端 NaN gyro。 */
    imu.INS_gyro[0] = 0.5f;
    imu.INS_gyro[1] = -0.25f;
    flow(0.01f, -0.005f, 2.0f, 100U, 20000U);
    check_wire(7U, 0.0f, 0.0f, 2.0f);

    /* 距离厘米换算与倾斜高度投影；姿态轴序、单位同时在 wire 检查。 */
    attitude(120U);
    imu.INS_angle[2] = 20.0f;
    imu.INS_angle[1] = -10.0f;
    range(250U);
    check_wire(7U, 0.0f, 0.0f, 2.5f * cosf(20.0f * DEG_TO_RAD) * cosf(-10.0f * DEG_TO_RAD));

    /* 光流质量差只撤销速度，距离仍有效。 */
    attitude(140U);
    flow(0.0f, 0.0f, 1.0f, 0U, 20000U);
    check_wire(5U, NAN, NAN, 1.0f);
    flow(0.0f, 0.0f, 1.0f, FLOW_MIN_QUALITY, 0U);
    check_wire(5U, NAN, NAN, 1.0f);
    flow(0.0f, 0.0f, 1.0f, 100U, 100000U);
    check_wire(5U, NAN, NAN, 1.0f);
    flow(NAN, 0.0f, 1.0f, 100U, 20000U);
    check_wire(5U, NAN, NAN, 1.0f);
    flow(0.0f, INFINITY, 1.0f, 100U, 20000U);
    check_wire(5U, NAN, NAN, 1.0f);
    flow(0.0f, 0.0f, -1.0f, 100U, 20000U);
    check_wire(1U, NAN, NAN, NAN);
    flow(0.0f, 0.0f, NAN, 100U, 20000U);
    check_wire(1U, NAN, NAN, NAN);
    range(0U);
    check_wire(1U, NAN, NAN, NAN);
    range(401U);
    check_wire(1U, NAN, NAN, NAN);
    flow(0.0f, 1.0f, 2.0f, 100U, 20000U); /* 不合理速度 */
    check_wire(5U, NAN, NAN, 2.0f);

    /* 过度倾斜、失效/过期姿态不能生成新估计。 */
    imu.INS_angle[2] = 45.0f;
    flow(0.0f, 0.0f, 2.0f, 100U, 20000U);
    check_wire(1U, NAN, NAN, NAN);
    attitude(160U);
    imu.IMU_init_flag = false;
    flow(0.0f, 0.0f, 2.0f, 100U, 20000U);
    check_wire(0U, NAN, NAN, NAN);
    attitude(180U);
    flow(0.0f, 0.0f, 2.0f, 100U, 20000U);
    now += CONTROL_IMU_TIMEOUT_MS + 1U;
    check_wire(0U, NAN, NAN, NAN);
    flow(0.0f, 0.0f, 2.0f, 100U, 20000U);
    imu.update_ms = now;
    check_wire(1U, NAN, NAN, NAN);

    /* 测距单独持续到来不能掩盖光流超时；高度仍然有效。 */
    attitude(300U);
    flow(0.0f, 0.01f, 2.0f, 100U, 20000U);
    attitude(401U);
    range(150U);
    check_wire(5U, NAN, NAN, 1.5f);
    attitude(502U);
    check_wire(1U, NAN, NAN, NAN);
    (void)mavlink_flow_get_snapshot(&snapshot, now);
    assert(!snapshot.online);

    /* Tick 回绕下 40ms 仍有效，101ms 失效。 */
    attitude(UINT32_MAX - 20U);
    flow(0.0f, 0.01f, 2.0f, 100U, 20000U);
    attitude(19U);
    check_wire(7U, 1.0f, 0.0f, 2.0f);
    attitude(80U);
    check_wire(1U, NAN, NAN, NAN);

    float value;
    assert(!optical_flow_height(1.0f, NAN, 0.0f, &value));
    assert(!optical_flow_height(1.0f, 0.0f, 0.0f, NULL));
    float output[2];
    assert(!optical_flow_velocity(NULL, 0.02f, 1.0f, imu.INS_gyro, output));
    assert(isnan(output[0]) && isnan(output[1]));
    puts("Telemetry passed: wire frames, units/axes, rotation compensation, validity, independent timeouts, tick wrap");
    return 0;
}
