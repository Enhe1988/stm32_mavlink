#include "attitude_control.h"
#include "bsp_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void)
{
    AttitudeControl c;
    assert(attitude_control_init(&c));
    float target[3] = {0}, angle[3] = {0}, rate[3] = {10, 0, 0};
    assert(attitude_control_step(&c, target, angle, rate, .001f, true, .02f, false, 1));
    rate[0] = 0;
    assert(attitude_control_step(&c, target, angle, rate, .001f, false, 0, false, 1));
#if CONTROL_GYRO_LPF_ENABLE
    assert(c.filtered_rate[0] > 0); /* 固定带宽仍在反馈路径过滤。 */
    assert(fabsf(c.lpf_cutoff_hz - CONTROL_GYRO_LPF_FIXED_HZ) < .001f);
#else
    assert(c.filtered_rate[0] == 0); /* 旁路时严格等于原始输入。 */
#endif
    puts("Fixed-cutoff / bypass configuration passed");
    return 0;
}
