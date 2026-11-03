#ifndef FILTER_H
#define FILTER_H
#include <stdbool.h>
typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    bool initialized;
} LPF2ndFilter;
/* 二阶 Butterworth：截止频率处幅值为直流的 1/sqrt(2)，两端频率单位 Hz。 */
bool LPF2nd_Init(LPF2ndFilter *f, float sample_hz, float cutoff_hz);
/* 只换系数，保留历史状态；动态调频时不得重新 Init，否则每次都会产生启动瞬态。 */
bool LPF2nd_SetCutoff(LPF2ndFilter *f, float sample_hz, float cutoff_hz);
void LPF2nd_Reset(LPF2ndFilter *f, float input);
void LPF2nd_Init_30Hz(LPF2ndFilter *f);
float LPF2nd_Apply(LPF2ndFilter *f, float input);

#endif
