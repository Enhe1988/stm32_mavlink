/* Include the actual app to exercise its private DMA-buffer ownership helper. */
#include "../../application/mavlink_app.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart2, huart3;
static uint32_t now;
static unsigned builds, starts, aborts, receive_aborts;
static HAL_StatusTypeDef start_result = HAL_OK, abort_result = HAL_OK;
uint32_t HAL_GetTick(void) { return now; }
uint16_t mavlink_telemetry_build(uint8_t *bytes, uint16_t capacity, uint32_t timestamp)
{
    assert(timestamp == now && capacity == MAVLINK_TELEMETRY_BUFFER_SIZE);
    memset(bytes, (int)++builds, capacity);
    return capacity;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *bytes, uint16_t count)
{
    assert(uart == &huart3 && bytes == s_telemetry_txbuf);
    assert(count == MAVLINK_TELEMETRY_BUFFER_SIZE);
    ++starts;
    if (start_result == HAL_OK) uart->gState = HAL_UART_STATE_BUSY_TX;
    return start_result;
}
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart)
{
    assert(uart == &huart3);
    ++aborts;
    if (abort_result == HAL_OK) uart->gState = HAL_UART_STATE_READY;
    return abort_result;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{ (void)uart; ++receive_aborts; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *bytes, uint16_t size, uint32_t timeout)
{ (void)bytes; (void)size; assert(uart == &huart3 && timeout <= 5U); return HAL_OK; }
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart)
{ (void)uart; return HAL_UART_RXEVENT_IDLE; }
HAL_StatusTypeDef MavPort_Start(MavlinkPort_t *port) { (void)port; return HAL_OK; }
void MavPort_OnRxEventFromISR(MavlinkPort_t *port, HAL_UART_RxEventTypeTypeDef event)
{ (void)port; (void)event; }
uint32_t MavPort_Drain(MavlinkPort_t *port) { (void)port; return 0U; }
void mavlink_rc_dispatch(const mavlink_message_t *message) { (void)message; }
void mavlink_flow_dispatch(const mavlink_message_t *message) { (void)message; }
void vTaskDelay(TickType_t ticks) { (void)ticks; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return NULL; }
TickType_t xTaskGetTickCount(void) { return now; }
BaseType_t xTaskNotifyFromISR(TaskHandle_t task, uint32_t bits, int action, BaseType_t *woken)
{ (void)task; (void)bits; (void)action; (void)woken; return pdTRUE; }
BaseType_t xTaskNotifyWait(uint32_t before, uint32_t after, uint32_t *bits, TickType_t timeout)
{ (void)before; (void)after; (void)bits; (void)timeout; return pdFALSE; }

int main(void)
{
    mavlink_send_telemetry();
    assert(starts == 1U && builds == 1U && s_telemetry_tx_active);
    uint8_t saved[MAVLINK_TELEMETRY_BUFFER_SIZE];
    memcpy(saved, s_telemetry_txbuf, sizeof(saved));
    now = MAVLINK_TELEMETRY_TX_TIMEOUT_MS;
    mavlink_send_telemetry();
    assert(starts == 1U && builds == 1U && g_telemetry_tx_skipped == 1U);
    assert(memcmp(saved, s_telemetry_txbuf, sizeof(saved)) == 0);

    /* 正常 UART TC 完成后才能覆盖 buffer。 */
    huart3.gState = HAL_UART_STATE_READY;
    mavlink_send_telemetry();
    assert(starts == 2U && builds == 2U && aborts == 0U);

    /* TX 超时但 abort 失败：不覆盖尚在使用中的 DMA buffer。 */
    memcpy(saved, s_telemetry_txbuf, sizeof(saved));
    now += MAVLINK_TELEMETRY_TX_TIMEOUT_MS + 1U;
    abort_result = HAL_ERROR;
    mavlink_send_telemetry();
    assert(aborts == 1U && builds == 2U && g_telemetry_tx_errors == 1U);
    assert(memcmp(saved, s_telemetry_txbuf, sizeof(saved)) == 0);

    /* 成功终止后恢复，保持 RX 不受影响。 */
    abort_result = HAL_OK;
    mavlink_send_telemetry();
    assert(aborts == 2U && builds == 3U && starts == 3U);
    assert(receive_aborts == 0U);

    /* DMA 启动失败不标记 active，下一次可重试。 */
    huart3.gState = HAL_UART_STATE_READY;
    start_result = HAL_ERROR;
    mavlink_send_telemetry();
    assert(!s_telemetry_tx_active);
    unsigned errors = g_telemetry_tx_errors;
    start_result = HAL_OK;
    mavlink_send_telemetry();
    assert(s_telemetry_tx_active && g_telemetry_tx_errors == errors);

    /* 外部 TX 占用也不能覆盖 buffer。 */
    s_telemetry_tx_active = false;
    unsigned previous_builds = builds;
    mavlink_send_telemetry();
    assert(builds == previous_builds);

    /* DMA 超时判断跨 Tick 回绕。 */
    huart3.gState = HAL_UART_STATE_READY;
    now = UINT32_MAX - 10U;
    mavlink_send_telemetry();
    now = 0U;
    previous_builds = builds;
    mavlink_send_telemetry();
    assert(builds == previous_builds);
    now = 11U;
    mavlink_send_telemetry();
    assert(builds == previous_builds + 1U && receive_aborts == 0U);
    puts("TX DMA passed: busy buffer ownership, timeout/abort failure, retry, RX isolation, tick wrap");
    return 0;
}
