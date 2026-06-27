/**
  ********************************************************
  * @file       mavlink_port.h
  * @brief      通用 MAVLink 串口端口:DMA(circular)+IDLE 接收,逐字节喂官方 parser。
  *             每个物理串口对应一个 MavlinkPort_t 实例;不同端口必须使用不同的
  *             mavlink_channel_t,否则两路字节流会污染同一个拼包状态机。
  *
  *             接收机制(方案A,无 ringbuffer):
  *               DMA 硬件循环写入 dma_buf —— head = buf_size - NDTR(DMA剩余计数);
  *               软件维护 tail,任务周期把 [tail, head) 的新字节逐个喂 parser;
  *               缓冲靠 DMA 循环原地覆盖,不需要任何清除动作。
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
    mavlink_message_t   msg;       /* 本端口独立的拼包结果 */
    mavlink_status_t    status;    /* 本端口独立的解析状态 */
    void (*on_message)(const mavlink_message_t *msg); /* 整帧派发回调 */
} MavlinkPort_t;

/**
  * @brief  启动端口 DMA 接收(circular,启动一次后长期运行)。
  */
void MavPort_Start(MavlinkPort_t *port);

/**
  * @brief  排空 [tail, head) 的新字节并喂给 parser;在任务里周期调用。
  */
void MavPort_Poll(MavlinkPort_t *port);

#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_PORT_H__ */
