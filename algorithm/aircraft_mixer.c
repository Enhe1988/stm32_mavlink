#include "aircraft_mixer.h"
#include "bsp_config.h"
#include <math.h>
#include <stddef.h>

static float clamp(float value, float low, float high)
{
    return value < low ? low : value > high ? high : value;
}

static uint16_t servo_pwm(float center, float offset)
{
    return (uint16_t)clamp(center + offset, SERVO_MIN_US, SERVO_MAX_US);
}

void aircraft_mixer_stop(AircraftMixOutput *out)
{
    out->motor[0] = out->motor[1] = MOTOR_STOP_US;
    out->servo[0] = SERVO1_CENTER_US;
    out->servo[1] = SERVO2_CENTER_US;
    out->servo[2] = SERVO3_CENTER_US;
}

/* 电机差速/推力矢量混控
 * A：左右电机=基础油门±Roll差速，双舵机=中位+Pitch+Yaw / 中位-Pitch+Yaw。
 * B/C：单电机只接油门，三个舵机分别接 Yaw/Pitch/Roll，B/C 只在 IMU 安装旋转上不同。
 * PID 输出是三轴控制增量，构型差异集中在这里，不复制上层姿态控制任务。
 * 电机和舵机分别限幅；未使用通道固定停机/回中；非法数值也返回停机/回中值。 */
bool aircraft_mixer_apply(const AircraftMixInput *input, AircraftMixOutput *out)
{
    if (out == NULL) {
        return false;
    }
    aircraft_mixer_stop(out);
    if (input == NULL || !isfinite(input->throttle) || !isfinite(input->roll) ||
        !isfinite(input->pitch) || !isfinite(input->yaw)) {
        return false;
    }

    float base = MOTOR_STOP_US + clamp(input->throttle, 0.0f, 1.0f)
                               * (MOTOR_MAX_US - MOTOR_STOP_US);
#if AIRCRAFT_DUAL_MOTOR
    /* 对称限制差速，保持左右平均油门；零油门时禁止差速拉起单侧电机。
     * 满油门时无可用对称余量，差速同样为零。 */
    float limit = fminf(A_MOTOR_DIFF_MAX_US,
                       fminf(base - MOTOR_STOP_US, MOTOR_MAX_US - base));
    float diff = clamp(input->roll * A_MOTOR_ROLL_GAIN, -limit, limit);
    out->motor[0] = (uint16_t)clamp(base + diff, MOTOR_STOP_US, MOTOR_MAX_US);
    out->motor[1] = (uint16_t)clamp(base - diff, MOTOR_STOP_US, MOTOR_MAX_US);
    out->servo[0] = servo_pwm(SERVO1_CENTER_US,
        input->pitch * A_SERVO1_PITCH_GAIN + input->yaw * A_SERVO1_YAW_GAIN);
    out->servo[1] = servo_pwm(SERVO2_CENTER_US,
        input->pitch * A_SERVO2_PITCH_GAIN + input->yaw * A_SERVO2_YAW_GAIN);
#else
    out->motor[0] = (uint16_t)base;
    out->servo[0] = servo_pwm(SERVO1_CENTER_US, input->yaw * BC_SERVO_YAW_GAIN);
    out->servo[1] = servo_pwm(SERVO2_CENTER_US, input->pitch * BC_SERVO_PITCH_GAIN);
    out->servo[2] = servo_pwm(SERVO3_CENTER_US, input->roll * BC_SERVO_ROLL_GAIN);
#endif
    return true;
}
