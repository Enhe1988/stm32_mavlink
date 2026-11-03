#ifndef AIRCRAFT_MIXER_H
#define AIRCRAFT_MIXER_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float throttle; /* 0..1，总油门 */
    float roll;     /* 姿态控制器输出，单位 us 增量 */
    float pitch;
    float yaw;
} AircraftMixInput;

typedef struct {
    uint16_t motor[2]; /* TIM3 CH3 / CH4；B/C 的第二路固定停机 */
    uint16_t servo[3]; /* TIM5 CH2 / CH3 / CH4；A 的第三路固定回中 */
} AircraftMixOutput;

void aircraft_mixer_stop(AircraftMixOutput *out);
/** 按编译期样机混控；非法数值返回 false，输出停机/回中值。 */
bool aircraft_mixer_apply(const AircraftMixInput *input, AircraftMixOutput *out);
#endif
