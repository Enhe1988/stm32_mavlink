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
void mavlink_task(void const *pvParameters);
void Mavlin_RX_InterruptHandler(void);
void Loop_Mavlink_Parse(void);
void Mavlink_Init(void);
void mavlink_test(void);
void Mavlink_Rece_Enable(void);
typedef struct
{
    // CRSF_FRAMETYPE_RC_CHANNELS_PACKED  遥控器通道打包帧类型
    uint16_t channels[16]; // 存储16个通道的数据
    float Left_X;          // 左摇杆x轴
    float Left_Y;          // 左摇杆y轴
    float Right_X;         // 右摇杆x轴
    float Right_Y;         // 右摇杆y轴
    float S1;              // 左滑块
    float S2;              // 右滑块
    uint8_t A;             // 按键A
    uint8_t B;             // 拨杆B
    uint8_t C;             // 拨杆C
    uint8_t D;             // 按键D
    uint8_t E;             // 拨杆E
    uint8_t F;             // 拨杆F

    // CRSF_FRAMETYPE_LINK_STATISTICS  链路统计帧类型
    uint8_t uplink_RSSI_1;         // 上行RSSI 1
    uint8_t uplink_RSSI_2;         // 上行RSSI 2
    uint8_t uplink_Link_quality;   // 上行链路质量
    int8_t uplink_SNR;             // 上行信噪比
    uint8_t active_antenna;        // 当前活跃天线
    uint8_t rf_Mode;               // 射频模式
    uint8_t uplink_TX_Power;       // 上行传输功率  0mW 10mW 25mW 100mW 500mW 1000mW 2000mW 50mW
    uint8_t downlink_RSSI;         // 下行RSSI
    uint8_t downlink_Link_quality; // 下行链路质量
    int8_t downlink_SNR;           // 下行信噪比

    // CRSF_FRAMETYPE_HEARTBEAT 心跳
    uint16_t heartbeat_counter; // 心跳计数器

} ELRS_Data;
typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float thrust;

} PC_Data_t;//来自PC的数据
const ELRS_Data *get_remote_control_point(void);
const PC_Data_t *get_pc_data_point(void);
#endif // MAVLINK_H
