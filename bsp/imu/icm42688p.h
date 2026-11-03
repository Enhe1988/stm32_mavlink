/** Chip-specific setup and physical-unit decoding (before board transforms). */
#ifndef ICM42688P_H
#define ICM42688P_H
#include "imu_api.h"
bool icm42688p_probe(void);
bool icm42688p_init(void);
bool icm42688p_enable_drdy(void);
uint8_t icm42688p_who_am_i(void);
int icm42688p_init_result(void);
bool icm42688p_decode(const uint8_t *sample, ImuData *out);
#endif
