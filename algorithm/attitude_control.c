#include "attitude_control.h"
#include "bsp_config.h"
#include <math.h>
#include <string.h>

static float limit(float value, float maximum)
{
    return value > maximum ? maximum : value < -maximum ? -maximum : value;
}

static float initial_cutoff(void)
{
    return CONTROL_GYRO_LPF_DYNAMIC_ENABLE ? CONTROL_GYRO_LPF_MIN_HZ : CONTROL_GYRO_LPF_FIXED_HZ;
}

/* 动态调整二阶低通。
 * 控制任务以 1kHz 运行，因此滤波器 fs 用该名义频率，而不是遥控帧频率。
 * 每 10ms 根据基础油门计算 fc_target，50ms 时间常数平滑 fc，再更换系数。
 * 更新系数保留历史状态；不在 ISR 计算三角函数。高油门允许更高截止频率，
 * 在噪声抑制与控制延迟之间做可配置折中，不代表已经识别实机振动频谱。 */
static void filter_rates(AttitudeControl *control, const float rate[3], float dt, float throttle)
{
#if CONTROL_GYRO_LPF_ENABLE && CONTROL_GYRO_LPF_DYNAMIC_ENABLE
    control->lpf_elapsed += dt;
    if (control->lpf_elapsed >= CONTROL_GYRO_LPF_UPDATE_S) {
        float t = fminf(fmaxf(throttle, 0.0f), 1.0f);
        float target = CONTROL_GYRO_LPF_MIN_HZ + t * (CONTROL_GYRO_LPF_MAX_HZ - CONTROL_GYRO_LPF_MIN_HZ);
        float alpha = control->lpf_elapsed / (CONTROL_GYRO_LPF_TAU_S + control->lpf_elapsed);
        float cutoff = control->lpf_cutoff_hz + alpha * (target - control->lpf_cutoff_hz);
        if (fabsf(cutoff - control->lpf_cutoff_hz) >= 0.05f) {
            for (int i = 0; i < CTRL_AXES; ++i) {
                (void)LPF2nd_SetCutoff(&control->gyro_lpf[i], 1000.0f / CONTROL_PERIOD_MS, cutoff);
            }
            control->lpf_cutoff_hz = cutoff;
        }
        control->lpf_elapsed = 0.0f;
    }
#else
    (void)dt;
    (void)throttle;
#endif
    for (int i = 0; i < CTRL_AXES; ++i) {
#if CONTROL_GYRO_LPF_ENABLE
        control->filtered_rate[i] = LPF2nd_Apply(&control->gyro_lpf[i], rate[i]);
#else
        control->filtered_rate[i] = rate[i];
#endif
    }
}

float attitude_wrap_degrees(float angle)
{
    angle = fmodf(angle + 180.0f, 360.0f);
    return (angle < 0.0f ? angle + 360.0f : angle) - 180.0f;
}

/* 配置文件适配三款样机
 * CONTROL_*_PID 已在 bsp_config.h 按 AIRCRAFT_MODEL 别名到 MODEL_A/B/C_*。
 * 本文件不再按样机复制控制算法；只加载当前样机的两级 PID 和滤波范围。
 * 配置是编译期初值，控制运行期间调参只改变本结构体中的参数。 */
bool attitude_control_init(AttitudeControl *control)
{
    static const float angle_pid[3][3] = {
        CONTROL_ANGLE_ROLL_PID, CONTROL_ANGLE_PITCH_PID, CONTROL_ANGLE_YAW_PID
    };
    static const float rate_pid[3][3] = {
        CONTROL_RATE_ROLL_PID, CONTROL_RATE_PITCH_PID, CONTROL_RATE_YAW_PID
    };
    memset(control, 0, sizeof(*control));
#if CONTROL_GYRO_LPF_ENABLE && CONTROL_GYRO_LPF_DYNAMIC_ENABLE
    if (!isfinite(CONTROL_GYRO_LPF_MAX_HZ) || CONTROL_GYRO_LPF_MIN_HZ > CONTROL_GYRO_LPF_MAX_HZ ||
        CONTROL_GYRO_LPF_MAX_HZ >= 500.0f / CONTROL_PERIOD_MS) return false;
#endif
    for (int i = 0; i < CTRL_AXES; ++i) {
        PID_init(&control->angle[i], PID_POSITION, angle_pid[i], CONTROL_MAX_RATE_DPS, CONTROL_MAX_RATE_DPS);
        PID_init(&control->rate[i], PID_POSITION, rate_pid[i], CONTROL_MAX_OUTPUT_US, CONTROL_MAX_INTEGRAL_US);
        if (!LPF2nd_Init(&control->gyro_lpf[i], 1000.0f / CONTROL_PERIOD_MS, initial_cutoff())) return false;
    }
    control->lpf_cutoff_hz = initial_cutoff();
    return true;
}

void attitude_control_reset(AttitudeControl *control)
{
    for (int i = 0; i < CTRL_AXES; ++i) {
        PID_clear(&control->angle[i]);
        PID_clear(&control->rate[i]);
        control->ff_raw[i] = control->feedforward[i] = 0.0f;
        control->rate_target[i] = control->output[i] = 0.0f;
        control->filtered_rate[i] = 0.0f;
        (void)LPF2nd_Init(&control->gyro_lpf[i], 1000.0f / CONTROL_PERIOD_MS, initial_cutoff());
    }
    control->target_initialized = 0U;
    control->lpf_cutoff_hz = initial_cutoff();
    control->lpf_elapsed = 0.0f;
}

/* 姿态串级 PID、前馈和振动噪声过滤
 * 1) 角速度测量经二阶 Butterworth；目标与输出不经过该滤波器，避免额外目标延迟。
 * 2) 角度外环将角度误差变成 deg/s；遥控前馈也为 deg/s，二者可相加并统一限幅。
 * 3) 角速度内环对比 filtered_rate 与 rate_target，输出 us 增量进入混控。
 * 前馈的一阶平滑与陀螺仪的二阶滤波是两种不同状态，不能混用或宣称等价。
 * 参数/频率范围为初值；本实现不自动识别扑翼振动频谱，也不证明实飞抑振效果。 */
bool attitude_control_step(AttitudeControl *control, const float target[3],
    const float angle[3], const float rate[3], float dt,
    bool target_updated, float target_dt, bool ff_enabled, float throttle)
{
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.02f || !isfinite(throttle)) {
        attitude_control_reset(control);
        return false;
    }
    for (int i = 0; i < CTRL_AXES; ++i) {
        if (!isfinite(target[i]) || !isfinite(angle[i]) || !isfinite(rate[i])) {
            attitude_control_reset(control);
            return false;
        }
    }
    filter_rates(control, rate, dt, throttle);
    bool first = !control->target_initialized;
    for (int i = 0; i < CTRL_AXES; ++i) {
        if (first || target_updated) {
            float change = target[i] - control->previous_target[i];
            if (i == CTRL_YAW) {
                change = attitude_wrap_degrees(change);
            }
            /* 新帧才更新差分，旧帧保持输入并逐周期平滑；首帧无历史故置零。
             * Yaw 已先折回 [-180,180)，从 +179 到 -179 的变化按 +2deg 计算。 */
            control->ff_raw[i] = (!first && ff_enabled && isfinite(target_dt) && target_dt > 0.0f)
                ? limit(CONTROL_RC_FF_GAIN * change / target_dt, CONTROL_RC_FF_MAX_DPS) : 0.0f;
            control->previous_target[i] = target[i];
        }
        if (!ff_enabled) {
            control->ff_raw[i] = control->feedforward[i] = 0.0f;
        }
        float alpha = dt / (CONTROL_RC_FF_TAU_S + dt);
        control->feedforward[i] += alpha * (control->ff_raw[i] - control->feedforward[i]);
        /* Yaw 走最短角度误差；角度外环输出 deg/s，内环反馈也是 deg/s。 */
        float set = i == CTRL_YAW ? angle[i] + attitude_wrap_degrees(target[i] - angle[i]) : target[i];
        /* 两级 PID 必须串接：不能直接拿 outer 驱动执行器，内环必须使用角速度反馈。 */
        float outer = PID_calc_dt(&control->angle[i], angle[i], set, dt);
        control->rate_target[i] = limit(outer + control->feedforward[i], CONTROL_MAX_RATE_DPS);
        control->output[i] = PID_calc_dt(&control->rate[i], control->filtered_rate[i], control->rate_target[i], dt);
    }
    control->target_initialized = 1U;
    return true;
}
