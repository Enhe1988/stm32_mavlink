#include "filter.h"
#include <math.h>
#include <stddef.h>

/**
 * @brief 初始化 30Hz 二阶 Butterworth 低通滤波器
 */
void LPF2nd_Init_30Hz(LPF2ndFilter *f) {
    (void)LPF2nd_Init(f, 1000.0f, 30.0f);
}

/* 二阶巴特沃斯低通。
 * 模拟原型 H(s)=wc²/(s²+sqrt(2)wc*s+wc²)，用带预畸变的双线性变换离散化。
 * K=tan(pi*fc/fs)，a0 归一化为 1；fc 必须小于 Nyquist 频率 fs/2。
 * 一组系数对一个轴有效，三个轴必须各自保存 x/y 历史，不能共用一个滤波状态。 */
bool LPF2nd_SetCutoff(LPF2ndFilter *f, float sample_hz, float cutoff_hz)
{
    if (f == NULL || !isfinite(sample_hz) || !isfinite(cutoff_hz) ||
        sample_hz <= 0.0f || cutoff_hz <= 0.0f || cutoff_hz >= sample_hz * 0.5f) return false;
    float k = tanf(3.14159265358979323846f * cutoff_hz / sample_hz);
    float k2 = k * k;
    float norm = 1.0f / (1.0f + 1.4142135623730951f * k + k2);
    f->b0 = k2 * norm;
    f->b1 = 2.0f * f->b0;
    f->b2 = f->b0;
    f->a1 = 2.0f * (k2 - 1.0f) * norm;
    f->a2 = (1.0f - 1.4142135623730951f * k + k2) * norm;
    return true;
}

bool LPF2nd_Init(LPF2ndFilter *f, float sample_hz, float cutoff_hz)
{
    if (!LPF2nd_SetCutoff(f, sample_hz, cutoff_hz)) return false;
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
    f->initialized = false;
    return true;
}

void LPF2nd_Reset(LPF2ndFilter *f, float input)
{
    /* 用当前角速度预填稳态历史，避免解锁时从 0 慢慢爬升成虚假的角速度误差。 */
    f->x1 = f->x2 = f->y1 = f->y2 = input;
    f->initialized = true;
}

float LPF2nd_Apply(LPF2ndFilter *f, float input) {
    if (!isfinite(input)) return f->initialized ? f->y1 : 0.0f;
    if (!f->initialized) { LPF2nd_Reset(f, input); return input; }
    /* 直接 I 型差分方程：y[n]=Σb*x - a1*y[n-1] - a2*y[n-2]。 */
    float out = f->b0 * input + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
    f->x2 = f->x1; f->x1 = input;
    f->y2 = f->y1; f->y1 = out;
    return out;
}
