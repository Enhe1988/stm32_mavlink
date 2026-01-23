#include "filter.h"

/**
 * @brief 初始化 30Hz 二阶 Butterworth 低通滤波器
 */
void LPF2nd_Init_30Hz(LPF2ndFilter *f) {
    // 截止频率 30Hz, 采样 1000Hz
    f->b0 = 0.0078202f;
    f->b1 = 0.0156404f;
    f->b2 = 0.0078202f;
    f->a1 = -1.7347257f;
    f->a2 = 0.7660065f;
    
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

float LPF2nd_Apply(LPF2ndFilter *f, float input) {
    float out = f->b0 * input + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
    f->x2 = f->x1; f->x1 = input;
    f->y2 = f->y1; f->y1 = out;
    return out;
}