#ifndef IMU_BMI088_H
#define IMU_BMI088_H
#include <stdbool.h>
#include <stdint.h>
bool bmi088_probe(void);
bool bmi088_init(void);
bool bmi088_start(void);
uint8_t bmi088_accel_id(void);
uint8_t bmi088_gyro_id(void);
#endif
