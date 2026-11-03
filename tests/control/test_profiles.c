#include "aircraft_mixer.h"
#include "imu_orientation.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void expect_stop(const AircraftMixOutput *out)
{
    assert(out->motor[0] == MOTOR_STOP_US && out->motor[1] == MOTOR_STOP_US);
    assert(out->servo[0] == SERVO1_CENTER_US && out->servo[1] == SERVO2_CENTER_US);
    assert(out->servo[2] == SERVO3_CENTER_US);
}

int main(void)
{
    float v[3] = {1, 2, 3};
    imu_orient_apply(v);
#if AIRCRAFT_MODEL == AIRCRAFT_MODEL_C
    assert(v[0] == 1 && v[1] == 2 && v[2] == 3);
#else
    assert(v[0] == -3 && v[1] == 1 && v[2] == -2);
#endif
    imu_orient_unapply(v);
    assert(v[0] == 1 && v[1] == 2 && v[2] == 3);

    AircraftMixInput in = {0.5f, 0, 0, 0};
    AircraftMixOutput out;
    aircraft_mixer_stop(&out);
    expect_stop(&out);
    assert(aircraft_mixer_apply(&in, &out));
    assert(out.motor[0] == 1450);
#if AIRCRAFT_DUAL_MOTOR
    assert(out.motor[1] == 1450);
    in.roll = 100; aircraft_mixer_apply(&in, &out);
    assert(out.motor[0] == 1480 && out.motor[1] == 1420);
    assert(out.servo[2] == SERVO3_CENTER_US);
    in.roll = -100; aircraft_mixer_apply(&in, &out);
    assert(out.motor[0] == 1420 && out.motor[1] == 1480);
    in.roll = 10000; aircraft_mixer_apply(&in, &out);
    assert(out.motor[0] == 1650 && out.motor[1] == 1250);
    in.throttle = 0; aircraft_mixer_apply(&in, &out);
    assert(out.motor[0] == MOTOR_STOP_US && out.motor[1] == MOTOR_STOP_US);
    in.throttle = 1; aircraft_mixer_apply(&in, &out);
    assert(out.motor[0] == MOTOR_MAX_US && out.motor[1] == MOTOR_MAX_US);
    in.throttle = 0.1f; aircraft_mixer_apply(&in, &out);
    assert(out.motor[0] == 1180 && out.motor[1] == 1000);
    in.throttle = 0.5f; in.roll = 0; in.pitch = 50; in.yaw = 30;
    aircraft_mixer_apply(&in, &out);
    assert(out.servo[0] == 1580 && out.servo[1] == 1480 && out.servo[2] == 1500);
#else
    assert(out.motor[1] == MOTOR_STOP_US);
    in.roll = 100; aircraft_mixer_apply(&in, &out);
    assert(out.servo[0] == 1500 && out.servo[1] == 1500 && out.servo[2] == 1400);
    assert(out.motor[0] == 1450 && out.motor[1] == MOTOR_STOP_US);
    in.pitch = 50; in.yaw = 30; aircraft_mixer_apply(&in, &out);
    assert(out.servo[0] == 1530 && out.servo[1] == 1550 && out.servo[2] == 1400);
#endif
    in.throttle = 3; aircraft_mixer_apply(&in, &out);
    assert(out.motor[0] <= MOTOR_MAX_US);
    in.throttle = -1; aircraft_mixer_apply(&in, &out);
    assert(out.motor[0] == MOTOR_STOP_US && out.motor[1] == MOTOR_STOP_US);
    in.pitch = 100000; in.yaw = 0; aircraft_mixer_apply(&in, &out);
#if AIRCRAFT_DUAL_MOTOR
    assert(out.servo[0] == SERVO_MAX_US && out.servo[1] == SERVO_MIN_US);
#else
    assert(out.servo[1] == SERVO_MAX_US);
#endif
    in.pitch = NAN; assert(!aircraft_mixer_apply(&in, &out)); expect_stop(&out);
    in.pitch = 0; in.roll = INFINITY;
    assert(!aircraft_mixer_apply(&in, &out)); expect_stop(&out);
    in.roll = 0; in.throttle = NAN;
    assert(!aircraft_mixer_apply(&in, &out)); expect_stop(&out);
    assert(!aircraft_mixer_apply(NULL, &out)); expect_stop(&out);
    assert(!aircraft_mixer_apply(&in, NULL));
    printf("Model %d: mixing, saturation, inactive channels, orientation and stop values passed\n", AIRCRAFT_MODEL);
    return 0;
}
