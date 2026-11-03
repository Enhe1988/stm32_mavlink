#include "filter.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* 用实际 C 差分实现测稳态正弦增益，验证真实频响，而非只对照系数。 */
static float gain(float cutoff, float frequency)
{
    LPF2ndFilter f;
    assert(LPF2nd_Init(&f, 1000, cutoff));
    double input_energy = 0, output_energy = 0;
    for (unsigned n = 0; n < 6000; ++n) {
        float in = sinf(2.0f * 3.14159265359f * frequency * n / 1000.0f);
        float out = LPF2nd_Apply(&f, in);
        if (n >= 1000) { input_energy += in * in; output_energy += out * out; }
    }
    return (float)sqrt(output_energy / input_energy);
}

int main(void)
{
    assert(gain(30, 5) > .99f);
    assert(fabsf(gain(30, 30) - .70710678f) < .002f);
    assert(gain(30, 150) < .05f);
    assert(gain(80, 150) > gain(30, 150) * 4);
    LPF2ndFilter f;
    assert(LPF2nd_Init(&f, 1000, 30));
    assert(LPF2nd_Apply(&f, 12) == 12); /* 首帧稳态预填，无虚假启动斜坡。 */
    for (unsigned n = 0; n < 5000; ++n) {
        assert(LPF2nd_SetCutoff(&f, 1000, 30.0f + 50.0f * (n % 100) / 99.0f));
        assert(fabsf(LPF2nd_Apply(&f, 12) - 12) < .001f);
    }
    float previous = f.b0;
    assert(!LPF2nd_SetCutoff(&f, 1000, 500));
    assert(!LPF2nd_SetCutoff(&f, 1000, 0));
    assert(!LPF2nd_SetCutoff(&f, NAN, 30));
    assert(f.b0 == previous); /* 非法参数不破坏已有系数/状态。 */
    assert(isfinite(LPF2nd_Apply(&f, NAN)));
    LPF2nd_Reset(&f, -3);
    assert(fabsf(LPF2nd_Apply(&f, -3) + 3) < .001f);
    puts("Butterworth: passband, -3dB cutoff, high-frequency attenuation, dynamic continuity and invalid parameters passed");
    return 0;
}
