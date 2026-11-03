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
#include "mavlink_telemetry.h"
#include "bsp_config.h"
#include "usart.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common/mavlink.h"
#include "mavlink_types.h"

/* ───────── MAVLink 库底层钩子 ───────── */

/* System ID, Component ID(供 mavlink_helpers.h 使用) */
mavlink_system_t mavlink_system = { 1, 1 };

/* 兼容库的 convenience send 钩子；周期遥测走下面的整批 DMA。
 * 光流 COMM_1 为单向输入，不允许该钩子向光流模块回发。只在 MAVLink 任务调用。 */
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
{
    if (chan == MAVLINK_COMM_0 && ch != NULL && length > 0)
        (void)HAL_UART_Transmit(&huart3, (uint8_t *)ch, (uint16_t)length, 5U);
}

/* ───────── DMA 接收缓冲(放 RAM_D2 非缓存区 .dma_buffer,见 bsp_platform.c MPU)───────── */

/* USART3 460800 baud,1024B 可容纳约 22ms 的满速连续数据。 */
#define RC_RX_BUF_SIZE   1024U
__attribute__((section(".dma_buffer"), aligned(4)))
static uint8_t s_rc_rxbuf[RC_RX_BUF_SIZE];

__attribute__((section(".dma_buffer"), aligned(4)))
static uint8_t s_telemetry_txbuf[MAVLINK_TELEMETRY_BUFFER_SIZE];
static bool s_telemetry_tx_active;
static uint32_t s_telemetry_tx_start_ms;
volatile uint32_t g_telemetry_tx_errors;
volatile uint32_t g_telemetry_tx_skipped;

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

/* IDLE/HT/TC + 任务通知
 * IDLE 在短帧结束时及时唤醒；HT/TC 保证无空闲的连续流也能定期处理。
 * TC 额外累计硬件回绕；回调只计事件/发通知，不阻塞等待、不解析帧。
 * 相关 UART/DMA 中断逻辑优先级为 5，满足 FreeRTOS FromISR 调用约束。 */
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

/* 20Hz 遥测整批 DMA：仅姿态/速度/高度及有效位，不发送心跳或测试帧。
 * buffer 在 UART TC 结束前不能覆盖。
 * 超时仅终止 TX，不中断 RX circular DMA；下一周期重发当前值，不积压旧遥测。
 * 160B @460800 最长约 3.5ms，.dma_buffer 总量仍由链接脚本限制在 2KB 内。 */
static void mavlink_send_telemetry(void)
{
    uint32_t now = HAL_GetTick();
    if (s_telemetry_tx_active && huart3.gState != HAL_UART_STATE_READY) {
        if ((uint32_t)(now - s_telemetry_tx_start_ms) <= MAVLINK_TELEMETRY_TX_TIMEOUT_MS) {
            g_telemetry_tx_skipped++;
            return;
        }
        if (HAL_UART_AbortTransmit(&huart3) != HAL_OK) {
            g_telemetry_tx_errors++;
            return;
        }
        g_telemetry_tx_errors++;
    }
    s_telemetry_tx_active = false;
    /* 也防止兼容发送钩子或将来其他 TX 调用尚未结束时覆盖缓冲。 */
    if (huart3.gState != HAL_UART_STATE_READY) {
        g_telemetry_tx_skipped++;
        return;
    }
    uint16_t length = mavlink_telemetry_build(s_telemetry_txbuf, sizeof(s_telemetry_txbuf), now);
    if (length == 0U || HAL_UART_Transmit_DMA(&huart3, s_telemetry_txbuf, length) != HAL_OK) {
        g_telemetry_tx_errors++;
        return;
    }
    s_telemetry_tx_active = true;
    s_telemetry_tx_start_ms = now;
}

/* ───────── 解析任务 ───────── */
/* 通信事件驱动与控制任务分离
 * USART3/COMM_0 与 USART2/COMM_1 分别排空自己的字节流和 parser 状态。
 * xTaskNotifyWait 的超时取距 next_tx 的剩余时间；无 RX 事件也能按时发送。
 * 周期截止点按绝对 Tick 推进，不用“循环 N 次”分频，因此频繁 RX 不会改变发送节拍。
 * TX 使用 normal DMA 整批发送，RX 使用 circular DMA。 */
void mavlink_task(void const *pvParameters)
{
    (void)pvParameters;
    const TickType_t tx_period = pdMS_TO_TICKS(MAVLINK_TELEMETRY_PERIOD_MS);
    configASSERT(tx_period > 0);

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
            mavlink_send_telemetry();
            now = xTaskGetTickCount();
            do
            {
                next_tx += tx_period;
            } while ((int32_t)(now - next_tx) >= 0);
        }
    }
}
