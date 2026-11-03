/**
 * Shared ICM acquisition: DRDY ISR -> SPI4 DMA -> completion notification -> task.
 * RAW_PROCESSING buffers stay owned by the parser until decoding is complete.
 * No sensor decoding or floating-point work runs in interrupt context.
 */
#include "icm_acq.h"
#include "icm42688p.h"
#include "icm42688p_regs.h"
#include "icm45686.h"
#include "icm45686_regs.h"
#include "imu_internal.h"
#include "bsp_platform.h"
#include "main.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define SENSOR_BYTES 14U
#define FRAME_BYTES (SENSOR_BYTES + 1U)
#define SPI_TIMEOUT_MS 100U

typedef enum { RAW_FREE, RAW_DMA, RAW_READY, RAW_PROCESSING } raw_state_t;

/* DMA1/2 must not use the task stack in DTCM. MPU makes this D2 region non-cacheable. */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t s_dma_tx[FRAME_BYTES];
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t s_dma_rx[2][FRAME_BYTES];

static volatile raw_state_t s_raw_state[2];
static uint32_t s_timestamp[2];
static volatile bool s_dma_busy;
static volatile bool s_started;
static volatile uint8_t s_dma_index;
static imu_type_t s_type;
static TaskHandle_t s_parse_task;

static void cs_low(void)
{
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
}

void icm_cs_high(void)
{
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
}

bool icm_read_registers(uint8_t reg, uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U || length > SENSOR_BYTES) {
        return false;
    }
    uint8_t tx[FRAME_BYTES] = {0};
    uint8_t rx[FRAME_BYTES] = {0};
    tx[0] = reg | 0x80U;
    cs_low();
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(
        &hspi4, tx, rx, length + 1U, SPI_TIMEOUT_MS);
    icm_cs_high();
    if (result != HAL_OK) {
        return false;
    }
    memcpy(data, rx + 1, length);
    return true;
}

bool icm_write_registers(uint8_t reg, const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U || length > 3U) {
        return false;
    }
    uint8_t tx[4] = {reg, 0U, 0U, 0U};
    memcpy(tx + 1, data, length);
    cs_low();
    HAL_StatusTypeDef result = HAL_SPI_Transmit(&hspi4, tx, length + 1U, SPI_TIMEOUT_MS);
    icm_cs_high();
    return result == HAL_OK;
}

bool icm_write_register(uint8_t reg, uint8_t value)
{
    return icm_write_registers(reg, &value, 1U);
}

bool icm_verify_register(uint8_t reg, uint8_t expected, uint8_t mask)
{
    uint8_t value;
    return icm_read_registers(reg, &value, 1U) && (value & mask) == (expected & mask);
}

/* 中断短、计算放任务
 * DMA 完成通知只负责唤醒；任务一次排空全部 READY 缓冲，而不是一条通知只取一帧。
 * 按时间戳先后处理两份缓冲，PROCESSING 期间不让 ISR 重用；浮点换算和发布都在这里。 */
static void parse_task(void *arg)
{
    (void)arg;
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        for (;;) {
            int index = -1;
            taskENTER_CRITICAL();
            for (int i = 0; i < 2; ++i) {
                if (s_raw_state[i] == RAW_READY &&
                    (index < 0 || (int32_t)(s_timestamp[i] - s_timestamp[index]) < 0)) {
                    index = i;
                }
            }
            if (index >= 0) {
                s_raw_state[index] = RAW_PROCESSING;
            }
            taskEXIT_CRITICAL();
            if (index < 0) {
                break;
            }

            ImuData sample = {0};
            /* A temperature sentinel keeps the last published temperature. */
            (void)imu_read(&sample);
            sample.timestamp = s_timestamp[index];
            const uint8_t *raw = s_dma_rx[index] + 1;
            bool valid = (s_type == IMU_TYPE_ICM45686)
                       ? icm45686_decode(raw, &sample) : icm42688p_decode(raw, &sample);
            if (valid) {
                imu_publish(&sample);
            } else {
                ++g_imu_status.invalid_sample_count;
            }

            taskENTER_CRITICAL();
            s_raw_state[index] = RAW_FREE;
            taskEXIT_CRITICAL();
        }
    }
}

int icm_acq_start(imu_type_t type)
{
    if (s_started) {
        return INIT_OK;
    }
    if (type != IMU_TYPE_ICM45686 && type != IMU_TYPE_ICM42688) {
        return INIT_WHO_AM_I_ERROR;
    }
    s_type = type;
    s_raw_state[0] = s_raw_state[1] = RAW_FREE;
    s_dma_busy = false;
    s_dma_index = 0U;
    memset(s_dma_tx, 0, sizeof(s_dma_tx));
    s_dma_tx[0] = 0x80U | ((type == IMU_TYPE_ICM45686)
                            ? ICM45686_ACCEL_DATA_X1 : ICM42688_TEMP_DATA1);
    if (s_parse_task == NULL &&
        xTaskCreate(parse_task, "icm_parse", 384U, NULL,
                    configMAX_PRIORITIES - 1U, &s_parse_task) != pdPASS) {
        return INIT_TASK_ERROR;
    }

    __HAL_GPIO_EXTI_CLEAR_IT(ICM_INT_Pin);
    bool enabled = (type == IMU_TYPE_ICM45686)
                 ? icm45686_enable_drdy() : icm42688p_enable_drdy();
    if (!enabled) {
        return INIT_SPI_ERROR;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(ICM_INT_Pin);
    s_started = true;
    return INIT_OK;
}

/* DRDY ISR 直接拉低 CS 启动 DMA，不先通知采集任务，省去一次任务切换。
 * 只选择 FREE 缓冲；DMA 忙或两缓冲均占用时统计 missed_drdy，不覆盖待解析帧。
 * 时间戳记在 DRDY 时刻，而非稍后的解析时刻，减少调度延迟对采样时间的影响。 */
void icm_acq_drdy_isr(void)
{
    if (!s_started) {
        return;
    }
    ++g_imu_status.drdy_count;
    if (s_dma_busy) {
        ++g_imu_status.missed_drdy_count;
        return;
    }
    int index = -1;
    for (int i = 0; i < 2; ++i) {
        if (s_raw_state[i] == RAW_FREE) {
            index = i;
            break;
        }
    }
    if (index < 0) {
        ++g_imu_status.missed_drdy_count;
        return;
    }
    s_raw_state[index] = RAW_DMA;
    s_dma_index = (uint8_t)index;
    s_dma_busy = true;
    s_timestamp[index] = (uint32_t)bsp_time_us();
    __DMB();
    cs_low();
    if (HAL_SPI_TransmitReceive_DMA(&hspi4, s_dma_tx, s_dma_rx[index], FRAME_BYTES) != HAL_OK) {
        icm_cs_high();
        s_raw_state[index] = RAW_FREE;
        s_dma_busy = false;
        ++g_imu_status.dma_start_error_count;
    }
}

void icm_acq_done_isr(void)
{
    if (!s_dma_busy) {
        return;
    }
    icm_cs_high();
    __DMB();
    s_raw_state[s_dma_index] = RAW_READY;
    s_dma_busy = false;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_parse_task, &woken);
    portYIELD_FROM_ISR(woken);
}

void icm_acq_error_isr(void)
{
    if (!s_started) {
        return;
    }
    icm_cs_high();
    if (s_dma_busy) {
        s_raw_state[s_dma_index] = RAW_FREE;
        s_dma_busy = false;
        ++g_imu_status.spi_error_count;
    }
}
