/**
  ********************************************************
  * @file       mavlink_rc.h
  * @brief      遥控/上位机 MAVLink 端口(USART3 / MAVLINK_COMM_0)的数据与派发。
  *             解析 RC_CHANNELS_OVERRIDE(遥控器通道)与 SET_ATTITUDE_TARGET
  *             (上位机姿态四元数),产出 ELRS_Data / PC_Data_t 供飞控读取。
  *
  * @note       ELRS_Data / PC_Data_t 原先被手动追加在生成库 common/mavlink.h 里,
  *             重新生成 MAVLink 库会丢失;现迁至本应用头文件,消除该隐患。
  ********************************************************
  */
#ifndef __MAVLINK_RC_H__
#define __MAVLINK_RC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "common/mavlink.h"

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

/**
  * @brief  获取遥控器数据指针
  */
const ELRS_Data *get_remote_control_point(void);

/**
  * @brief  获取上位机数据指针
  */
const PC_Data_t *get_pc_data_point(void);

/**
  * @brief  COMM_0(USART3)整帧派发:RC_CHANNELS_OVERRIDE / SET_ATTITUDE_TARGET。
  *         作为 MavlinkPort_t.on_message 回调注册。
  */
void mavlink_rc_dispatch(const mavlink_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_RC_H__ */
