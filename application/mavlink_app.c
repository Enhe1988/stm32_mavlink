/**
  ********************************************************
  * @file       mavlink_app.c
  * @brief      MAVLink 应用层装配。包含 MAVLink 库底层钩子(发送/system id)、
  *             各串口端口实例、DMA 接收缓冲、初始化与解析任务。
  *
  *             接收链路为 DMA(circular)+IDLE/HT/TC 任务通知(见 mavlink_port.c),
  *             DMA 缓冲本身作为零拷贝环形缓冲。
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

/* USART3 460800 baud,1024B 可容纳约 22ms 的满速连续数据。 */
#define RC_RX_BUF_SIZE   1024U
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

/* USART2 115200 baud,256B 可容纳约 22ms 的满速连续数据。 */
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

#define MAVLINK_EVENT_RC_RX       (1UL << 0)
#define MAVLINK_EVENT_FLOW_RX     (1UL << 1)
#define MAVLINK_EVENT_RC_ERROR    (1UL << 2)
#define MAVLINK_EVENT_FLOW_ERROR  (1UL << 3)

static TaskHandle_t s_mavlink_task;

static void mavlink_notify_from_isr(uint32_t event)
{
    BaseType_t higher_priority_woken = pdFALSE;

    if (s_mavlink_task != NULL)
    {
        xTaskNotifyFromISR(s_mavlink_task,
                           event,
                           eSetBits,
                           &higher_priority_woken);
        portYIELD_FROM_ISR(higher_priority_woken);
    }
}

/* ───────── 初始化 ───────── */
void Mavlink_Init(void)
{
    while (MavPort_Start(&g_rc_port) != HAL_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
#if MAVLINK_FLOW_PORT_ENABLE
    while (MavPort_Start(&g_flow_port) != HAL_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
#endif
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    (void)size;
    HAL_UART_RxEventTypeTypeDef event_type = HAL_UARTEx_GetRxEventType(huart);

    if (huart == g_rc_port.huart)
    {
        MavPort_OnRxEventFromISR(&g_rc_port, event_type);
        mavlink_notify_from_isr(MAVLINK_EVENT_RC_RX);
    }
#if MAVLINK_FLOW_PORT_ENABLE
    else if (huart == g_flow_port.huart)
    {
        MavPort_OnRxEventFromISR(&g_flow_port, event_type);
        mavlink_notify_from_isr(MAVLINK_EVENT_FLOW_RX);
    }
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_rc_port.huart)
    {
        g_rc_port.uart_errors++;
        mavlink_notify_from_isr(MAVLINK_EVENT_RC_ERROR);
    }
#if MAVLINK_FLOW_PORT_ENABLE
    else if (huart == g_flow_port.huart)
    {
        g_flow_port.uart_errors++;
        mavlink_notify_from_isr(MAVLINK_EVENT_FLOW_ERROR);
    }
#endif
}

static void mavlink_restart_port(MavlinkPort_t *port)
{
    (void)HAL_UART_AbortReceive(port->huart);
    while (MavPort_Start(port) != HAL_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

/* ───────── 2Hz 测试发送:固定 float,经 USART3 → 上位机 ───────── */
/* 用 NAMED_VALUE_FLOAT(msg 251),QGC/Mission Planner/MAVLink Inspector 可直接显示。 */
static void mavlink_test_send_float(void)
{
    const float test_value = 3.14f;   /* 固定测试数,验证发送链路 */
    static const char test_name[10] = "test";
    mavlink_msg_named_value_float_send(MAVLINK_COMM_0,
                                        HAL_GetTick(),
                                        test_name,
                                        test_value);
}

/* ───────── 解析任务 ───────── */
void mavlink_task(void const *pvParameters)
{
    (void)pvParameters;
    const TickType_t tx_period = pdMS_TO_TICKS(500U);

    s_mavlink_task = xTaskGetCurrentTaskHandle();
    Mavlink_Init();
    TickType_t next_tx = xTaskGetTickCount() + tx_period;

    while (1)
    {
        uint32_t events = 0;
        TickType_t now = xTaskGetTickCount();
        int32_t until_tx = (int32_t)(next_tx - now);
        TickType_t wait_ticks = (until_tx > 0) ? (TickType_t)until_tx : 0U;

        (void)xTaskNotifyWait(0U,
                              UINT32_MAX,
                              &events,
                              wait_ticks);

        if ((events & MAVLINK_EVENT_RC_ERROR) != 0U)
        {
            mavlink_restart_port(&g_rc_port);
            events &= ~MAVLINK_EVENT_RC_RX;
        }
        else if ((events & MAVLINK_EVENT_RC_RX) != 0U)
        {
            (void)MavPort_Drain(&g_rc_port);
        }

#if MAVLINK_FLOW_PORT_ENABLE
        if ((events & MAVLINK_EVENT_FLOW_ERROR) != 0U)
        {
            mavlink_restart_port(&g_flow_port);
            events &= ~MAVLINK_EVENT_FLOW_RX;
        }
        else if ((events & MAVLINK_EVENT_FLOW_RX) != 0U)
        {
            (void)MavPort_Drain(&g_flow_port);
        }
#endif
        now = xTaskGetTickCount();
        if ((int32_t)(now - next_tx) >= 0)
        {
            mavlink_test_send_float();
            do
            {
                next_tx += tx_period;
            } while ((int32_t)(now - next_tx) >= 0);
        }
    }
}
