#ifndef MAVLINK_TELEMETRY_H
#define MAVLINK_TELEMETRY_H

#include <stdint.h>

/* 1×ATTITUDE(40B) + 3×NAMED_VALUE_FLOAT(30B) + 1×NAMED_VALUE_INT(30B)，
 * MAVLink v2 无签名最大 160B。整批 DMA，缓冲在完成前由调用方保持不变。 */
#define MAVLINK_TELEMETRY_BUFFER_SIZE 160U
#define TELEMETRY_ATTITUDE_VALID 1U
#define TELEMETRY_VELOCITY_VALID 2U
#define TELEMETRY_HEIGHT_VALID 4U

/* 仅在 MAVLink 任务调用；返回写入字节数，容量不足返回 0 且不推进发送序号。 */
uint16_t mavlink_telemetry_build(uint8_t *buffer, uint16_t capacity, uint32_t now_ms);

#endif
