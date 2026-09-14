/**
  ********************************************************
  * @file       mavlink_port.c
  * @brief      通用 MAVLink 串口端口实现(见 mavlink_port.h 说明)。
  ********************************************************
  */
#include "mavlink_port.h"

HAL_StatusTypeDef MavPort_Start(MavlinkPort_t *port)
{
    port->tail = 0;
    port->dma_wraps = 0;
    port->consumed = 0;
    mavlink_reset_channel_status(port->chan);

    /* circular 模式启动一次即长期运行,IDLE/HT/TC 事件中都不重启 DMA。 */
    return HAL_UARTEx_ReceiveToIdle_DMA(port->huart,
                                        port->dma_buf,
                                        port->buf_size);
}

void MavPort_OnRxEventFromISR(MavlinkPort_t *port,
                              HAL_UART_RxEventTypeTypeDef event_type)
{
    if (port == NULL)
    {
        return;
    }

    /* circular DMA 每完成一整圈产生一次 TC；IDLE/HT 只负责低延迟唤醒。 */
    if (event_type == HAL_UART_RXEVENT_TC)
    {
        port->dma_wraps++;
    }
    port->rx_events++;
}

uint32_t MavPort_Drain(MavlinkPort_t *port)
{
    uint32_t wraps_before;
    uint32_t wraps_after;
    uint16_t remaining;

    /* TC ISR 可能恰好夹在读取回绕计数和 NDTR 之间,不一致时重读。 */
    do
    {
        wraps_before = port->dma_wraps;
        remaining = (uint16_t)__HAL_DMA_GET_COUNTER(port->huart->hdmarx);
        wraps_after = port->dma_wraps;
    } while (wraps_before != wraps_after);

    /* position 允许等于 buf_size:表示 NDTR 已到 0、TC ISR 尚未执行。 */
    uint32_t position = (uint32_t)port->buf_size - remaining;
    uint32_t produced = wraps_after * (uint32_t)port->buf_size + position;
    uint32_t available = produced - port->consumed;

    if (available > port->buf_size)
    {
        uint32_t dropped = available - port->buf_size;
        port->overrun_count++;
        port->dropped_bytes += dropped;
        port->consumed = produced - port->buf_size;
        port->tail = (uint16_t)(port->consumed % port->buf_size);
        available = port->buf_size;

        /* 覆盖可能截断了一帧,丢弃 parser 中尚未完成的残帧。 */
        mavlink_reset_channel_status(port->chan);
    }

    uint32_t drained = available;

    while (available-- > 0U)
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
            port->tail = 0;
        }
        port->consumed++;
    }

    return drained;
}
