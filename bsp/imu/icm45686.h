/** Chip-specific setup and physical-unit decoding (before board transforms). */
#ifndef ICM45686_H
#define ICM45686_H
#include "imu_api.h"
bool icm45686_probe(void);
bool icm45686_init(void);
bool icm45686_enable_drdy(void);
uint8_t icm45686_who_am_i(void);
int icm45686_init_result(void);
bool icm45686_decode(const uint8_t *sample, ImuData *out);
#endif
