/** @file
 *  @brief MAVLink comm protocol built from common.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_H
#define MAVLINK_H

#define MAVLINK_PRIMARY_XML_HASH -66877331833629974

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#ifndef MAVLINK_ENDIAN
#define MAVLINK_ENDIAN MAVLINK_LITTLE_ENDIAN
#endif

#ifndef MAVLINK_ALIGNED_FIELDS
#define MAVLINK_ALIGNED_FIELDS 1
#endif

#ifndef MAVLINK_CRC_EXTRA
#define MAVLINK_CRC_EXTRA 1
#endif

#ifndef MAVLINK_COMMAND_24BIT
#define MAVLINK_COMMAND_24BIT 1
#endif

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

#include "mavlink_types.h"
extern mavlink_system_t mavlink_system;
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length);
/* use efficient approach, see mavlink_helpers.h */
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes

#include "version.h"
#include "common.h"
/* 注:应用层接口(端口/RC/光流/任务)与 ELRS_Data/PC_Data_t 已迁出本生成头文件,
 * 见 application/mavlink_app.h、mavlink_rc.h、mavlink_flow.h。
 * 这样重新生成 MAVLink 库不会再覆盖工程自定义代码。 */
#endif // MAVLINK_H
