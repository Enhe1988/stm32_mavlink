/* Driver-only publication interface. Input is in sensor axes and physical units. */
#ifndef IMU_INTERNAL_H
#define IMU_INTERNAL_H
#include "imu_api.h"
void imu_publish(ImuData *sample);
#endif
