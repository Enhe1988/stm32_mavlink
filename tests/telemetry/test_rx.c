/* Exercise actual app RX callbacks -> circular DMA drain -> official parser -> flow publication.
 * HAL registers/DMA writes and task notification waits are simulated; this is not a board test. */
#include "../../application/mavlink_app.c"
#include "INS_task.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "flow_wire_vectors.h"

static DMA_HandleTypeDef rc_dma, flow_dma;
UART_HandleTypeDef huart2 = {.hdmarx = &flow_dma};
UART_HandleTypeDef huart3 = {.hdmarx = &rc_dma};
static uint32_t now, pending_events, rc_frames, receive_starts;
static HAL_UART_RxEventTypeTypeDef rx_event;

uint32_t HAL_GetTick(void) { return now; }
bool ins_get_snapshot(IMU_data_t *out)
{
    /* Raw flow reception must work even before INS initialization/calibration. */
    memset(out, 0, sizeof(*out));
    return false;
}
uint16_t mavlink_telemetry_build(uint8_t *buffer, uint16_t capacity, uint32_t timestamp)
{ (void)buffer; (void)capacity; (void)timestamp; return 0U; }
void mavlink_rc_dispatch(const mavlink_message_t *message)
{ assert(message->msgid == MAVLINK_MSG_ID_HEARTBEAT); ++rc_frames; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t count)
{
    assert((uart == &huart2 && buffer == s_flow_rxbuf && count == FLOW_RX_BUF_SIZE) ||
           (uart == &huart3 && buffer == s_rc_rxbuf && count == RC_RX_BUF_SIZE));
    uart->hdmarx->remaining = count;
    ++receive_starts;
    return HAL_OK;
}
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *uart)
{ (void)uart; return rx_event; }
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart) { (void)uart; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *uart) { (void)uart; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size, uint32_t timeout)
{ (void)uart; (void)data; (void)size; (void)timeout; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{ (void)uart; (void)data; (void)size; return HAL_OK; }
void vTaskDelay(TickType_t ticks) { (void)ticks; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (TaskHandle_t)1; }
TickType_t xTaskGetTickCount(void) { return now; }
BaseType_t xTaskNotifyFromISR(TaskHandle_t task, uint32_t bits, int action, BaseType_t *woken)
{ assert(task == (TaskHandle_t)1 && action == eSetBits); pending_events |= bits; *woken = pdFALSE; return pdTRUE; }
BaseType_t xTaskNotifyWait(uint32_t before, uint32_t after, uint32_t *bits, TickType_t timeout)
{ (void)before; (void)after; (void)timeout; *bits = pending_events; pending_events = 0U; return pdTRUE; }

static void drain_notified_ports(void)
{
    uint32_t events = pending_events;
    pending_events = 0U;
    if (events & MAVLINK_EVENT_RC_RX) (void)MavPort_Drain(&g_rc_port);
    if (events & MAVLINK_EVENT_FLOW_RX) (void)MavPort_Drain(&g_flow_port);
}

static void event(MavlinkPort_t *port, HAL_UART_RxEventTypeTypeDef kind, bool drain)
{
    rx_event = kind;
    HAL_UARTEx_RxEventCallback(port->huart, port->buf_size - port->huart->hdmarx->remaining);
    if (drain) drain_notified_ports();
}

static void write_dma(MavlinkPort_t *port, const uint8_t *bytes, unsigned count, bool drain)
{
    DMA_HandleTypeDef *dma = port->huart->hdmarx;
    for (unsigned i = 0; i < count; ++i) {
        unsigned position = port->buf_size - dma->remaining;
        port->dma_buf[position] = bytes[i];
        --dma->remaining;
        if (dma->remaining == 0U) {
            dma->remaining = port->buf_size;
            event(port, HAL_UART_RXEVENT_TC, drain);
        } else if (dma->remaining == port->buf_size / 2U) {
            event(port, HAL_UART_RXEVENT_HT, drain);
        }
    }
    event(port, HAL_UART_RXEVENT_IDLE, drain);
}

static uint16_t flow_bytes(uint8_t *bytes)
{
    mavlink_optical_flow_rad_t packet = {0};
    packet.time_usec = (uint64_t)now * 1000U;
    packet.integration_time_us = 20000U;
    packet.integrated_x = 0.01f;
    packet.integrated_y = -0.02f;
    packet.distance = 1.5f;
    packet.quality = 100U;
    packet.integrated_xgyro = packet.integrated_ygyro = packet.integrated_zgyro = NAN;
    mavlink_message_t message;
    mavlink_msg_optical_flow_rad_encode_chan(2, 88, MAVLINK_COMM_2, &message, &packet);
    return mavlink_msg_to_send_buffer(bytes, &message);
}

static uint16_t range_bytes(uint8_t *bytes)
{
    mavlink_distance_sensor_t packet = {0};
    packet.time_boot_ms = now;
    packet.current_distance = 150U;
    packet.max_distance = 4000U;
    packet.orientation = 25U;
    packet.covariance = 4U;
    /* Zero tail exercises MAVLink v2 payload truncation. */
    mavlink_message_t message;
    mavlink_msg_distance_sensor_encode_chan(2, 88, MAVLINK_COMM_2, &message, &packet);
    assert(message.len < MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN);
    return mavlink_msg_to_send_buffer(bytes, &message);
}

static OpticalFlow_Data_t snapshot(void)
{
    OpticalFlow_Data_t out;
    assert(mavlink_flow_get_snapshot(&out, now));
    return out;
}

int main(void)
{
    s_mavlink_task = (TaskHandle_t)1;
    Mavlink_Init();
    assert(receive_starts == 2U);
    uint8_t flow[MAVLINK_MAX_PACKET_LEN], range[MAVLINK_MAX_PACKET_LEN];
    uint16_t flow_length = flow_bytes(flow), range_length = range_bytes(range);
    for (unsigned batch = 0; batch < 2000U; ++batch) {
        now = batch * 20U;
        write_dma(&g_flow_port, flow, flow_length, true);
        write_dma(&g_flow_port, range, range_length, true);
        OpticalFlow_Data_t out = snapshot();
        assert(out.flow_frame_count == batch + 1U && out.range_frame_count == batch + 1U);
        assert(out.flow_quality == 100U && out.range_cm == 150U);
        assert(fabsf(out.flow_x_rad - 0.01f) < 1e-6f);
        assert(fabsf(out.flow_y_rad + 0.02f) < 1e-6f);
        assert(out.ground_distance_m == 1.5f);
        assert(!out.velocity_valid && !out.height_valid); /* INS unavailable is not RX failure */
    }
    assert(g_flow_port.dma_wraps > 100U && g_flow_port.overrun_count == 0U);
    assert(receive_starts == 2U); /* HT/TC/IDLE must never restart DMA */

    /* COMM_0 full frame between two parts of a COMM_1 frame must not disturb assembly. */
    write_dma(&g_flow_port, flow, 11U, true);
    mavlink_message_t rc;
    uint8_t rc_bytes[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack_chan(9, 1, MAVLINK_COMM_3, &rc, 0, 0, 0, 0, 0);
    unsigned rc_length = mavlink_msg_to_send_buffer(rc_bytes, &rc);
    write_dma(&g_rc_port, rc_bytes, rc_length, true);
    write_dma(&g_flow_port, flow + 11U, flow_length - 11U, true);
    assert(rc_frames == 1U && snapshot().flow_frame_count == 2001U);

    /* Empty/duplicate event does not consume or publish bytes again. */
    uint32_t consumed = g_flow_port.consumed;
    event(&g_flow_port, HAL_UART_RXEVENT_IDLE, true);
    assert(g_flow_port.consumed == consumed && snapshot().flow_frame_count == 2001U);

    flow[flow_length - 1U] ^= 0x80U;
    write_dma(&g_flow_port, flow, flow_length, true);
    assert(snapshot().flow_frame_count == 2001U);
    flow[flow_length - 1U] ^= 0x80U;
    write_dma(&g_flow_port, flow, flow_length, true);
    assert(snapshot().flow_frame_count == 2002U);

    /* Forced task starvation overwrites old bytes; retain valid latest frames and recover. */
    uint8_t junk[FLOW_RX_BUF_SIZE * 3U];
    memset(junk, 0x55, sizeof(junk));
    write_dma(&g_flow_port, junk, sizeof(junk), false);
    write_dma(&g_flow_port, flow, flow_length, false);
    write_dma(&g_flow_port, range, range_length, false);
    drain_notified_ports();
    assert(g_flow_port.overrun_count == 1U && g_flow_port.dropped_bytes > 0U);
    assert(snapshot().flow_frame_count == 2003U && snapshot().range_frame_count == 2001U);

    /* These raw bytes/CRC are generated independently of the C MAVLink pack helpers.
     * Both actual 44B payload and 40B tail-zero-truncated payload must decode exactly. */
    for (unsigned i = 0; i < sizeof(wire_cases) / sizeof(wire_cases[0]); ++i) {
        uint32_t previous_frames = snapshot().flow_frame_count;
        /* Split inside the payload also tests byte assembly before decoding. */
        write_dma(&g_flow_port, wire_cases[i].bytes, 24U, true);
        write_dma(&g_flow_port, wire_cases[i].bytes + 24U, wire_cases[i].size - 24U, true);
        OpticalFlow_Data_t out = snapshot();
        assert(out.flow_frame_count == previous_frames + 1U);
        assert(out.flow_x_rad == wire_cases[i].x && out.flow_y_rad == wire_cases[i].y);
        assert(out.flow_quality == wire_cases[i].quality);
        assert(fabsf(out.integration_time_s - 0.02f) < 1e-6f);
        assert(fabsf(out.ground_distance_m - 0.777f) < 1e-6f);
    }
    puts("RX pipeline passed: 2000 flow/range pairs, DMA wraps, partial/interleaved frames, CRC rejection, overwrite recovery, INS-independent raw RX");
    puts("Independent wire vectors passed: MAVLink v1/v2 and version switching, nonzero/zero flow, quality 0/87/255, 44B/40B payload, raw bytes vs official decode");
    return 0;
}
