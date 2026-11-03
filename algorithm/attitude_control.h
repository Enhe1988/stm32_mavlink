#ifndef ATTITUDE_CONTROL_H
#define ATTITUDE_CONTROL_H
#include "pid.h"
#include "filter.h"

/* 控制器内部统一按 Roll/Pitch/Yaw 排列，角度 deg、角速度 deg/s。 */
enum { CTRL_ROLL, CTRL_PITCH, CTRL_YAW, CTRL_AXES };
typedef struct {
    pid_type_def angle[CTRL_AXES];
    pid_type_def rate[CTRL_AXES];
    float previous_target[CTRL_AXES];
    float ff_raw[CTRL_AXES];
    float feedforward[CTRL_AXES];
    float rate_target[CTRL_AXES];
    float output[CTRL_AXES];
    uint8_t target_initialized;
    LPF2ndFilter gyro_lpf[CTRL_AXES]; /* 三轴角速度反馈各自独立的二阶状态 */
    float filtered_rate[CTRL_AXES];
    float lpf_cutoff_hz;  /* 调试器观测当前动态截止频率 */
    float lpf_elapsed;
} AttitudeControl;

/* 返回 false 表示滤波截止频率配置非法；调用层不得带着无效系数进入控制。 */
bool attitude_control_init(AttitudeControl *control);
void attitude_control_reset(AttitudeControl *control);
float attitude_wrap_degrees(float angle);
/** target_updated 只在新指令帧到达时置位；target_dt 是相邻帧间隔秒数。
 * 仅遥控启用前馈，上位机保持同一串级控制器、前馈关闭。 */
bool attitude_control_step(AttitudeControl *control, const float target[3],
    const float angle[3], const float rate[3], float dt,
    bool target_updated, float target_dt, bool ff_enabled, float throttle);
#endif
