/**
  ********************************************************
  * @file       mavlink_app.c
  * @brief      MAVLink 应用层装配。包含 MAVLink 库底层钩子(发送/system id)、
  *             各串口端口实例、DMA 接收缓冲、初始化与解析任务。
  *
  *             接收链路改为 DMA(circular)+IDLE(方案A,见 mavlink_port.c),
  *             不再使用逐字节中断与 ringbuffer。
  ********************************************************
  */
#include "mavlink_app.h"
#include "mavlink_port.h"
#include "mavlink_rc.h"
#include "mavlink_flow.h"
#include "usart.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common/mavlink.h"
#include "mavlink_types.h"
#include <string.h>

/* ───────── MAVLink 库底层钩子 ───────── */

/* System ID, Component ID(供 mavlink_helpers.h 使用) */
mavlink_system_t mavlink_system = { 1, 1 };

/* 把库的发送宏 MAVLINK_SEND_UART_BYTES 关联到 USART3(遥测/QGC 下行) */
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
{
    (void)chan;
    HAL_UART_Transmit(&huart3, (uint8_t *)ch, length, 2000);
}

/* ───────── DMA 接收缓冲(放 RAM_D2 非缓存区 .dma_buffer,见 icm_platform.c MPU)───────── */

/* USART3 420000 baud,10ms 轮询约 420B,留余量 → 512B */
#define RC_RX_BUF_SIZE   512U
__attribute__((section(".dma_buffer"), aligned(4)))
static uint8_t s_rc_rxbuf[RC_RX_BUF_SIZE];

static MavlinkPort_t g_rc_port = {
    .huart      = &huart3,
    .chan       = MAVLINK_COMM_0,
    .dma_buf    = s_rc_rxbuf,
    .buf_size   = RC_RX_BUF_SIZE,
    .on_message = mavlink_rc_dispatch,
};

#if MAVLINK_FLOW_PORT_ENABLE
extern UART_HandleTypeDef huart2;

/* USART2 115200 baud,10ms 轮询约 115B → 256B 足够 */
#define FLOW_RX_BUF_SIZE 256U
__attribute__((section(".dma_buffer"), aligned(4)))
static uint8_t s_flow_rxbuf[FLOW_RX_BUF_SIZE];

static MavlinkPort_t g_flow_port = {
    .huart      = &huart2,
    .chan       = MAVLINK_COMM_1,
    .dma_buf    = s_flow_rxbuf,
    .buf_size   = FLOW_RX_BUF_SIZE,
    .on_message = mavlink_flow_dispatch,
};
#endif

/* ───────── 初始化 ───────── */
void Mavlink_Init(void)
{
    MavPort_Start(&g_rc_port);
#if MAVLINK_FLOW_PORT_ENABLE
    MavPort_Start(&g_flow_port);
#endif
}

/* ───────── 10ms 硬编码心跳(保留原有功能,verbatim 自原 mavlink.c)───────── */
static mavlink_message_t lastmsg;

static void mavlink_test_heartbeat2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    (void)system_id;
    (void)component_id;
    (void)last_msg;
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
    if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HEARTBEAT >= 256)
    {
        return;
    }
#endif
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    (void)buffer;
    mavlink_heartbeat_t packet_in =
    {
        963497464, 17, 84, 151, 218, 3
    };
    mavlink_heartbeat_t packet1, packet2;
    memset(&packet1, 0, sizeof(packet1));
    packet1.custom_mode = packet_in.custom_mode;
    packet1.type = packet_in.type;
    packet1.autopilot = packet_in.autopilot;
    packet1.base_mode = packet_in.base_mode;
    packet1.system_status = packet_in.system_status;
    packet1.mavlink_version = packet_in.mavlink_version;

#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
    {
        // cope with extensions
        memset(MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1) - MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN);
    }
#endif
    memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_heartbeat_send(MAVLINK_COMM_0, packet1.type, packet1.autopilot, packet1.base_mode, packet1.custom_mode, packet1.system_status);
    MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

/* ───────── 解析任务 ───────── */
void mavlink_task(void const *pvParameters)
{
    (void)pvParameters;
    while (1)
    {
        MavPort_Poll(&g_rc_port);
#if MAVLINK_FLOW_PORT_ENABLE
        MavPort_Poll(&g_flow_port);
#endif
        mavlink_test_heartbeat2(1, 1, &lastmsg);   /* 保留:10ms 心跳 */
        vTaskDelay(10);
    }
}
