/**
  ********************************************************
  * @file       mavlink_port.h
 * @brief      通用 MAVLink 串口端口:DMA(circular)+IDLE/HT/TC 事件驱动接收,
 *             在任务中逐字节喂官方 parser。
  *             每个物理串口对应一个 MavlinkPort_t 实例;不同端口必须使用不同的
  *             mavlink_channel_t,否则两路字节流会污染同一个拼包状态机。
  *
 *             DMA 缓冲本身就是零拷贝环形缓冲:TC 中断累计 DMA 回绕次数,
 *             IDLE/HT/TC 只唤醒解析任务;任务根据回绕次数和 NDTR 计算累计
 *             生产字节数,从 tail 排空新数据。不会在 ISR 中解析 MAVLink。
  ********************************************************
  */
#ifndef __MAVLINK_PORT_H__
#define __MAVLINK_PORT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "common/mavlink.h"

typedef struct
{
    UART_HandleTypeDef *huart;     /* 绑定串口,其 hdmarx 必须配成 DMA_CIRCULAR */
    mavlink_channel_t   chan;      /* MAVLINK_COMM_0/1/...,各端口唯一 */
    uint8_t            *dma_buf;   /* DMA 环形缓冲,须放非缓存区(.dma_buffer) */
    uint16_t            buf_size;  /* 缓冲字节数 */
    uint16_t            tail;      /* 软件读指针(已喂给 parser 到此) */
    volatile uint32_t   dma_wraps; /* DMA TC 次数,由 ISR 更新 */
    uint32_t            consumed;  /* 任务累计消费字节数 */
    volatile uint32_t   rx_events; /* IDLE/HT/TC 事件计数 */
    volatile uint32_t   uart_errors;
    uint32_t            overrun_count;
    uint32_t            dropped_bytes;
    mavlink_message_t   msg;       /* 本端口独立的拼包结果 */
    mavlink_status_t    status;    /* 本端口独立的解析状态 */
    void (*on_message)(const mavlink_message_t *msg); /* 整帧派发回调 */
} MavlinkPort_t;

/**
  * @brief  启动端口 DMA 接收(circular,启动一次后长期运行)。
  */
HAL_StatusTypeDef MavPort_Start(MavlinkPort_t *port);

/**
  * @brief  记录一次 HAL UART RX 事件。只能在 HAL_UARTEx_RxEventCallback 中调用。
  */
void MavPort_OnRxEventFromISR(MavlinkPort_t *port,
                              HAL_UART_RxEventTypeTypeDef event_type);

/**
  * @brief  排空 DMA 已生产的新字节并喂给 parser;只能在任务中调用。
  * @return 本次消费的字节数。
  */
uint32_t MavPort_Drain(MavlinkPort_t *port);

#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_PORT_H__ */
