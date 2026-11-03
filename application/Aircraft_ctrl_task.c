/** 三轴角度外环 + 遥控变化率前馈 -> 角速度内环 -> 样机混控。 */
#include "Aircraft_ctrl_task.h"
#include "attitude_control.h"
#include "bsp_config.h"
#include "bsp_platform.h"
#include "cmsis_os.h"
#include "tim.h"
#include <math.h>

aircraft_control_t aircraft_control;
AttitudeControl g_attitude_control; /* 调试：PID、feedforward、rate_target、output */
static uint32_t s_rc_frame, s_rc_time;

static float clamp(float v, float lo, float hi)
{ return v < lo ? lo : v > hi ? hi : v; }
static float stick(float v)
{ return fabsf(v) <= RC_DEADBAND ? 0.0f : clamp(v, -100.0f, 100.0f); }

static void write_outputs(const AircraftMixOutput *out)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, out->motor[0]);
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, out->motor[1]);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, out->servo[0]);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, out->servo[1]);
    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, out->servo[2]);
}

static void pwm_init(void)
{
    aircraft_mixer_stop(&aircraft_control.actuator_output);
    write_outputs(&aircraft_control.actuator_output);
#if AIRCRAFT_PWM_ENABLE
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
#if AIRCRAFT_DUAL_MOTOR
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
#else
    if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
#endif
#endif
}

/* 任务间解耦
 * 通信任务只负责发布完整指令；本任务一次复制 RC、PC 和 INS 快照。
 * 后续状态判断和控制计算使用这份本地数据，不跨任务直接访问更新中的字段。
 * frame_count 用于区分“新遥控帧”和“1ms 周期再次读取同一帧”。 */
static void refresh_snapshots(void)
{
    MavlinkRcSnapshot_t rc;
    MavlinkPcSnapshot_t pc;
    mavlink_rc_get_snapshot(&rc);
    mavlink_pc_get_snapshot(&pc);
    aircraft_control.remote_data = rc.data;
    aircraft_control.remote_update_ms = rc.update_ms;
    aircraft_control.remote_frame_count = rc.frame_count;
    aircraft_control.remote_valid = rc.valid;
    aircraft_control.pc_data = pc.data;
    aircraft_control.pc_update_ms = pc.update_ms;
    aircraft_control.pc_valid = pc.valid;
    (void)ins_get_snapshot(&aircraft_control.imu);
}

/* 飞行状态机与失控保护
 * RC 100ms 超时、姿态无效/10ms 超时优先进入零力，不允许后续拨杆覆盖保护结果。
 * 上位机模式还检查 PC 指令 100ms 有效期；所有时间差使用无符号减法兼容 Tick 回绕。
 * 调参状态只修改 RAM 中的增益且保持停机，重新上电恢复配置文件初值。 */
static aircraft_behaviour_e select_mode(uint32_t now)
{
    const aircraft_control_t *a = &aircraft_control;
    if (!a->remote_valid || now - a->remote_update_ms > RC_LINK_TIMEOUT_MS ||
        !a->imu.IMU_init_flag || now - a->imu.update_ms > CONTROL_IMU_TIMEOUT_MS) return Aircraft_ZERO_FORCE;
    /* 先匹配调参模式，避免被相同 B/C 组合的普通模式覆盖；调参时停机。 */
    if (a->remote_data.E == 2) {
        if (a->remote_data.B == 0 && a->remote_data.C == 0) return Aircraft_InRing_P;
        if (a->remote_data.B == 2 && a->remote_data.C == 2) return Aircraft_InRing_R;
        if (a->remote_data.B == 0 && a->remote_data.C == 2) return Aircraft_OutRing_P;
    }
    if (a->remote_data.B == 2 && a->remote_data.C == 0) return Aircraft_RUN;
    if (a->remote_data.B == 2 && a->remote_data.C == 2 && a->pc_valid &&
        now - a->pc_update_ms <= CONTROL_PC_TIMEOUT_MS) return Aircraft_Auto;
    return Aircraft_ZERO_FORCE;
}

static void tune_stopped(void)
{
    if (aircraft_control.remote_data.F != 2) return;
    float knob = clamp(aircraft_control.remote_data.Left_Y, 0.0f, 100.0f);
    switch (aircraft_control.aircraft_behaviour) {
    case Aircraft_InRing_P: g_attitude_control.angle[CTRL_PITCH].Kp = 0.1f * knob; break;
    case Aircraft_InRing_R: g_attitude_control.rate[CTRL_PITCH].Kp = 0.05f * knob; break;
    case Aircraft_OutRing_P: g_attitude_control.rate[CTRL_ROLL].Kp = 0.05f * knob; break;
    case Aircraft_OutRing_R: g_attitude_control.angle[CTRL_ROLL].Kp = 0.05f * knob; break;
    default: break;
    }
}

/* 串级 PID + 前馈 + 构型混控
 * RC 摇杆映射为 Roll/Pitch 目标角（±20deg）；Yaw 摇杆映射为航向目标变化率。
 * INS 的欧拉角原序为 yaw/pitch/roll，这里重排为控制器统一的 roll/pitch/yaw。
 * 陀螺仪 rad/s 转 deg/s 后在控制器内部经过三轴独立二阶低通，再进入角速度内环。
 * 目标变化率前馈按真实 RC 帧间隔求导，不用 1ms 周期替代帧间隔，避免尖峰被放大。
 * 内环输出单位为 us 增量；交给 mixer 后才变为当前样机的电机/舵机绝对脉宽。 */
static void control_step(float dt)
{
    aircraft_control_t *a = &aircraft_control;
    aircraft_mixer_stop(&a->actuator_output);
    if (a->aircraft_behaviour != Aircraft_RUN && a->aircraft_behaviour != Aircraft_Auto) {
        attitude_control_reset(&g_attitude_control);
        tune_stopped();
        return;
    }
    float target[3], throttle, frame_dt = 0.0f;
    bool updated = false;
    bool rc_mode = a->aircraft_behaviour == Aircraft_RUN;
    if (rc_mode) {
        target[CTRL_ROLL] = stick(a->remote_data.Right_X) * CONTROL_MAX_ANGLE_DEG / 100.0f;
        target[CTRL_PITCH] = stick(a->remote_data.Right_Y) * CONTROL_MAX_ANGLE_DEG / 100.0f;
        a->yaw_set_angle = attitude_wrap_degrees(a->yaw_set_angle +
            stick(a->remote_data.Left_X) * CONTROL_YAW_RATE_DPS / 100.0f * dt);
        target[CTRL_YAW] = a->yaw_set_angle;
        throttle = a->remote_data.Left_Y * 0.01f;
        updated = a->remote_frame_count != s_rc_frame;
        if (updated) {
            frame_dt = (uint32_t)(a->remote_update_ms - s_rc_time) * 0.001f;
            s_rc_frame = a->remote_frame_count;
            s_rc_time = a->remote_update_ms;
        }
    } else {
        target[CTRL_ROLL] = clamp(a->pc_data.roll, -CONTROL_MAX_ANGLE_DEG, CONTROL_MAX_ANGLE_DEG);
        target[CTRL_PITCH] = clamp(a->pc_data.pitch, -CONTROL_MAX_ANGLE_DEG, CONTROL_MAX_ANGLE_DEG);
        target[CTRL_YAW] = a->pc_data.yaw;
        throttle = a->pc_data.thrust;
    }
    float angle[3] = {a->imu.INS_angle[2], a->imu.INS_angle[1], a->imu.INS_angle[0]};
    float rate[3] = {a->imu.INS_gyro[0] * RAD_TO_DEG,
        a->imu.INS_gyro[1] * RAD_TO_DEG, a->imu.INS_gyro[2] * RAD_TO_DEG};
    if (!attitude_control_step(&g_attitude_control, target, angle, rate, dt,
                               updated, frame_dt, rc_mode && CONTROL_RC_FF_ENABLE, throttle)) return;
    AircraftMixInput input = {throttle, g_attitude_control.output[CTRL_ROLL],
        g_attitude_control.output[CTRL_PITCH], g_attitude_control.output[CTRL_YAW]};
    (void)aircraft_mixer_apply(&input, &a->actuator_output);
}

/* FreeRTOS 固定周期控制
 * vTaskDelayUntil 基于上一计划唤醒点推进 1ms，避免把本轮运算耗时累加进下一周期。
 * DWT 另测真实 dt，供积分/微分使用；绝对周期调度不意味着实际执行间隔绝无抖动。
 * 切换模式清空 PID 历史和前馈历史，防止上一模式积分/目标差分影响新模式。 */
void Aircraft_ctrl_task(void const *pvParameters)
{
    (void)pvParameters;
    if (!attitude_control_init(&g_attitude_control)) Error_Handler();
    pwm_init();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    configASSERT(period > 0);
    TickType_t wake = xTaskGetTickCount();
    uint32_t previous_us = (uint32_t)bsp_time_us();
    for (;;) {
        uint32_t now_us = (uint32_t)bsp_time_us();
        float dt = (uint32_t)(now_us - previous_us) * 1.0e-6f;
        previous_us = now_us;
        if (dt <= 0.0f) dt = CONTROL_PERIOD_MS * 0.001f;
        refresh_snapshots();
        aircraft_control.aircraft_behaviour = select_mode(HAL_GetTick());
        if (aircraft_control.aircraft_behaviour != aircraft_control.last_aircraft_behaviour) {
            attitude_control_reset(&g_attitude_control);
            aircraft_control.yaw_set_angle = aircraft_control.imu.INS_angle[0];
            s_rc_frame = 0;
            s_rc_time = aircraft_control.remote_update_ms;
        }
        aircraft_control.last_aircraft_behaviour = aircraft_control.aircraft_behaviour;
        control_step(dt);
        write_outputs(&aircraft_control.actuator_output);
        vTaskDelayUntil(&wake, period);
    }
}
