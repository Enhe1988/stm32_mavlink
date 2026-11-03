#include "attitude_control.h"
#include "aircraft_mixer.h"
#include "bsp_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(float a, float b) { assert(fabsf(a - b) < 0.0002f); }

int main(void)
{
    AttitudeControl c;
    float target[3] = {1, 1, 1}, angle[3] = {0}, rate[3] = {0};
    assert(attitude_control_init(&c));
    /* 配置加载按当前样机独立核对；下方算法场景使用固定参数，避免实机调参
     * 改变测试期望值。不能让 B/C 的回归假定它们与 A 使用相同 PID。 */
#if AIRCRAFT_MODEL == AIRCRAFT_MODEL_A
    const float expected_angle[3][3] = {MODEL_A_ANGLE_ROLL_PID, MODEL_A_ANGLE_PITCH_PID, MODEL_A_ANGLE_YAW_PID};
    const float expected_rate[3][3] = {MODEL_A_RATE_ROLL_PID, MODEL_A_RATE_PITCH_PID, MODEL_A_RATE_YAW_PID};
#elif AIRCRAFT_MODEL == AIRCRAFT_MODEL_B
    const float expected_angle[3][3] = {MODEL_B_ANGLE_ROLL_PID, MODEL_B_ANGLE_PITCH_PID, MODEL_B_ANGLE_YAW_PID};
    const float expected_rate[3][3] = {MODEL_B_RATE_ROLL_PID, MODEL_B_RATE_PITCH_PID, MODEL_B_RATE_YAW_PID};
#else
    const float expected_angle[3][3] = {MODEL_C_ANGLE_ROLL_PID, MODEL_C_ANGLE_PITCH_PID, MODEL_C_ANGLE_YAW_PID};
    const float expected_rate[3][3] = {MODEL_C_RATE_ROLL_PID, MODEL_C_RATE_PITCH_PID, MODEL_C_RATE_YAW_PID};
#endif
    for (unsigned i = 0; i < 3; ++i) {
        near(c.angle[i].Kp, expected_angle[i][0]);
        near(c.angle[i].Ki, expected_angle[i][1]);
        near(c.angle[i].Kd, expected_angle[i][2]);
        near(c.rate[i].Kp, expected_rate[i][0]);
        near(c.rate[i].Ki, expected_rate[i][1]);
        near(c.rate[i].Kd, expected_rate[i][2]);
        c.angle[i].Kp = i == 0 ? 5.0f : 2.0f;
        c.angle[i].Ki = c.angle[i].Kd = 0.0f;
        c.rate[i].Kp = 0.5f;
        c.rate[i].Ki = c.rate[i].Kd = 0.0f;
    }
    assert(attitude_control_step(&c, target, angle, rate, .001f, true, .02f, true, .5f));
    /* 首帧没有前馈；外环输出进入内环，不能直接把外环输出当执行器输出。 */
    near(c.feedforward[0], 0); near(c.rate_target[0], 5); near(c.output[0], 2.5f);
    near(c.rate_target[1], 2); near(c.output[1], 1); near(c.output[2], 1);
    target[0] = target[1] = target[2] = 0;
    rate[0] = 10;
    attitude_control_step(&c, target, angle, rate, .001f, true, .02f, false, .5f);
    assert(c.filtered_rate[0] > 0 && c.filtered_rate[0] < rate[0]);
    near(c.output[0], -.5f * c.filtered_rate[0]); /* 滤波后的反馈实际进入内环。 */

    attitude_control_reset(&c);
    rate[0] = 0;
    attitude_control_step(&c, target, angle, rate, .001f, true, .02f, true, .5f);
    target[0] = 1;
    attitude_control_step(&c, target, angle, rate, .001f, true, .02f, true, .5f);
    near(c.ff_raw[0], 25); /* 按 20ms 指令间隔求差分，而非 1ms 控制周期。 */
    near(c.feedforward[0], 25.0f / 21.0f);
    near(c.output[0], (5.0f + 25.0f / 21.0f) * .5f);
    float previous = c.feedforward[0];
    attitude_control_step(&c, target, angle, rate, .001f, false, 0, true, .5f);
    near(c.ff_raw[0], 25); assert(c.feedforward[0] > previous);
    previous = c.feedforward[0];
    attitude_control_step(&c, target, angle, rate, .001f, true, .02f, true, .5f);
    near(c.ff_raw[0], 0); assert(c.feedforward[0] < previous);

    target[0] = 100;
    attitude_control_step(&c, target, angle, rate, .001f, true, .001f, true, .5f);
    near(c.ff_raw[0], CONTROL_RC_FF_MAX_DPS);
    near(c.rate_target[0], CONTROL_MAX_RATE_DPS);
    rate[0] = -10000;
    attitude_control_step(&c, target, angle, rate, .001f, false, 0, false, .5f);
    for (unsigned n = 0; n < 500; ++n)
        attitude_control_step(&c, target, angle, rate, .001f, false, 0, false, .5f);
    near(c.output[0], CONTROL_MAX_OUTPUT_US); near(c.feedforward[0], 0);
    AircraftMixInput in = {.throttle=.5f, .roll=c.output[0], .pitch=0, .yaw=0};
    AircraftMixOutput out;
    assert(aircraft_mixer_apply(&in, &out));
#if AIRCRAFT_DUAL_MOTOR
    assert(out.motor[0] > out.motor[1]);
#else
    assert(out.servo[2] < SERVO3_CENTER_US);
#endif
    attitude_control_reset(&c);
    target[0] = 0; rate[0] = 0; target[2] = angle[2] = 179;
    attitude_control_step(&c, target, angle, rate, .001f, true, .02f, true, .5f);
    target[2] = -179;
    attitude_control_step(&c, target, angle, rate, .001f, true, .02f, true, .5f);
    near(c.ff_raw[2], 50); near(c.angle[2].error[0], 2); /* 跨 ±180° 取短路径。 */
    attitude_control_reset(&c);
    attitude_control_step(&c, target, angle, rate, .001f, true, .02f, true, .5f);
    near(c.feedforward[2], 0);
    assert(!attitude_control_step(&c, target, angle, rate, 0, false, 0, true, .5f));
    near(c.output[2], 0);
    target[0] = NAN;
    assert(!attitude_control_step(&c, target, angle, rate, .001f, true, .02f, true, .5f));

    /* 动态调度实际更换 fc，直流角速度不因换系数出现冲击。 */
    attitude_control_reset(&c);
    target[0] = target[1] = target[2] = 0;
    angle[0] = angle[1] = angle[2] = 0;
    rate[0] = 7; rate[1] = -3; rate[2] = 1;
    for (unsigned n = 0; n < 2000; ++n) {
        assert(attitude_control_step(&c, target, angle, rate, .001f, false, 0, false, 1));
        near(c.filtered_rate[0], 7);
    }
    assert(c.lpf_cutoff_hz > 79 && c.lpf_cutoff_hz <= 80);
    for (unsigned n = 0; n < 2000; ++n)
        attitude_control_step(&c, target, angle, rate, .001f, false, 0, false, 0);
    assert(c.lpf_cutoff_hz >= 30 && c.lpf_cutoff_hz < 31);
    near(c.filtered_rate[1], -3);

    pid_type_def pid;
    const float gains[3] = {0, 2, 0};
    PID_init(&pid, PID_POSITION, gains, 10, 5);
    near(PID_calc_dt(&pid, 0, 1, .1f), .2f); /* I 项乘真实 dt。 */
    PID_clear(&pid); pid.Kp = 1; pid.max_out = 1;
    near(PID_calc_dt(&pid, 0, 100, .1f), 1); near(pid.Iout, 0); /* 抗饱和 */
    printf("Model %d: cascade, rate feedback, RC feedforward, wrap, reset, dt PID and mixer passed\n", AIRCRAFT_MODEL);
    return 0;
}
