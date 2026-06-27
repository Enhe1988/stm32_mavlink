/**
  ********************************************************
  * @file       mavlink_port.c
  * @brief      通用 MAVLink 串口端口实现(见 mavlink_port.h 说明)。
  ********************************************************
  */
#include "mavlink_port.h"

void MavPort_Start(MavlinkPort_t *port)
{
    port->tail = 0;

    /* circular 模式启动一次即长期运行;IDLE 用于及时收尾,无需在事件里重启 */
    HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->dma_buf, port->buf_size);

    /* 半满中断对本方案无用(我们靠 NDTR 算 head),关掉以减少 ISR 抖动 */
    if (port->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(port->huart->hdmarx, DMA_IT_HT);
    }
}

void MavPort_Poll(MavlinkPort_t *port)
{
    /* head = DMA 下一个要写的位置 = 缓冲大小 - 剩余未传输字节(NDTR) */
    uint16_t head = port->buf_size -
                    (uint16_t)__HAL_DMA_GET_COUNTER(port->huart->hdmarx);

    while (port->tail != head)
    {
        uint8_t byte = port->dma_buf[port->tail];

        if (mavlink_parse_char(port->chan, byte, &port->msg, &port->status))
        {
            if (port->on_message != NULL)
            {
                port->on_message(&port->msg);
            }
        }

        if (++port->tail >= port->buf_size)
        {
            port->tail = 0;   /* 走到缓冲末尾,绕回开头 */
        }
    }
}
