#include "optical_flow.h"
#include "bsp_config.h"
#include <math.h>
#include <stddef.h>

static bool valid_distance(float distance)
{
    return isfinite(distance) && distance >= FLOW_MIN_DISTANCE_M && distance <= FLOW_MAX_DISTANCE_M;
}

bool optical_flow_height(float distance, float roll, float pitch, float *height)
{
    if (height == NULL) return false;
    *height = NAN;
    if (!valid_distance(distance) || !isfinite(roll) || !isfinite(pitch)) return false;
    float tilt_cos = cosf(roll) * cosf(pitch);
    if (tilt_cos < FLOW_MIN_TILT_COS) return false;
    *height = distance * tilt_cos;
    return true;
}

bool optical_flow_velocity(const float integrated[2], float dt, float distance,
                           const float gyro[2], float velocity[2])
{
    if (velocity == NULL) return false;
    velocity[0] = velocity[1] = NAN;
    if (integrated == NULL || gyro == NULL || !valid_distance(distance) ||
        !isfinite(dt) || dt < FLOW_MIN_INTEGRATION_S || dt > FLOW_MAX_INTEGRATION_S ||
        !isfinite(integrated[0]) || !isfinite(integrated[1]) ||
        !isfinite(gyro[0]) || !isfinite(gyro[1])) return false;

    /* MAVLink OPTICAL_FLOW_RAD 的角轴约定：前移 +flow_y，右移 -flow_x。
     * 先映射/标定原始光流，再减 gyro；发送端 gyro 为 NaN，使用本机 INS。 */
    float flow_x = integrated[FLOW_BODY_X_AXIS] * FLOW_BODY_X_SCALE / dt - gyro[0];
    float flow_y = integrated[FLOW_BODY_Y_AXIS] * FLOW_BODY_Y_SCALE / dt - gyro[1];
    float vx = flow_y * distance;
    float vy = -flow_x * distance;
    if (!isfinite(vx) || !isfinite(vy) ||
        fabsf(vx) > FLOW_MAX_SPEED_MPS || fabsf(vy) > FLOW_MAX_SPEED_MPS) return false;
    velocity[0] = vx;
    velocity[1] = vy;
    return true;
}
