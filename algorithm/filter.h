#ifndef __FILTER_H__
#define __FILTER_H__
typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} LPF2ndFilter;
void LPF2nd_Init_30Hz(LPF2ndFilter *f);
float LPF2nd_Apply(LPF2ndFilter *f, float input);







#endif