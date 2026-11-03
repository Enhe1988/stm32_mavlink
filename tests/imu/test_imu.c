/* Execute production selection, both ICM initializers/decoders, shared DMA state
 * machine and publication. HAL/RTOS and the Bosch adapter are simulated. */
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "imu_api.h"
#include "bsp_config.h"
#include "imu_calib.h"
#include "imu_selftest.h"
#include "icm_acq.h"
#include "icm42688p.h"
#include "icm45686.h"
#include "spi.h"
#include "task.h"

SPI_HandleTypeDef hspi4 = {SPI4};
volatile imu_bias_t g_imu_bias;
static unsigned mask, fail_mask, task_count, notifications, cs;
static bool task_fail, dma_fail, configuring456;
static uint8_t registers[128], raw[14];
static void (*parser)(void *);
static jmp_buf blocked;
static unsigned waits;

void HAL_GPIO_WritePin(void *port, uint16_t pin, unsigned state)
{ (void)port; (void)pin; cs = state; }
void bsp_delay_us(uint32_t us) { (void)us; }
uint64_t bsp_time_us(void) { static uint64_t t; return ++t; }

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, uint8_t *tx, uint16_t n, uint32_t timeout)
{
    (void)spi; (void)timeout;
    assert(cs == GPIO_PIN_RESET && n >= 2 && n <= 4);
    /* Inject failure after identity detection, at sensor power configuration. */
    if ((tx[0] == 0x10 && (fail_mask & 1)) ||
        (tx[0] == 0x4e && (fail_mask & 2))) return HAL_ERROR;
    for (unsigned i = 1; i < n; ++i) registers[tx[0] + i - 1] = tx[i];
    if (tx[0] == 0x7f) { registers[0x7f] = 1; configuring456 = true; } /* IREG_DONE */
    if (tx[0] == 0x11) configuring456 = false;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx, uint8_t *rx,
                                         uint16_t n, uint32_t timeout)
{
    (void)spi; (void)timeout;
    assert(cs == GPIO_PIN_RESET && n >= 2 && n <= 15);
    unsigned addr = tx[0] & 0x7f;
    for (unsigned i = 1; i < n; ++i) rx[i] = registers[addr + i - 1];
    if (addr == 0x72) rx[1] = (mask & 1) ? 0xe9 : 0;
    if (addr == 0x75) rx[1] = (mask & 2) ? 0x47 : 0;
    if (addr == 0x19) rx[1] = 0x80; /* 45686 reset complete */
    if (addr == 0x2d && !configuring456) rx[1] = 0x10; /* 42688 reset complete */
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *spi, uint8_t *tx, uint8_t *rx, uint16_t n)
{
    (void)spi;
    assert(cs == GPIO_PIN_RESET && n == 15);
    assert(tx[0] == (g_imu_active_type == IMU_TYPE_ICM45686 ? 0x80 : 0x9d));
    if (dma_fail) return HAL_ERROR;
    memcpy(rx + 1, raw, sizeof(raw));
    return HAL_OK;
}
BaseType_t xTaskCreate(void (*entry)(void *), const char *name, unsigned stack,
                      void *arg, unsigned priority, TaskHandle_t *handle)
{
    (void)name; (void)arg;
    assert(stack == 384 && priority == configMAX_PRIORITIES - 1);
    if (task_fail) return 0;
    ++task_count;
    parser = entry;
    *handle = (void *)1;
    return pdPASS;
}
uint32_t ulTaskNotifyTake(BaseType_t clear, uint32_t timeout)
{
    assert(clear == pdTRUE && timeout == portMAX_DELAY);
    if (waits++) longjmp(blocked, 1); /* Stop when the task blocks again. */
    return 1;
}
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken)
{ assert(task); *woken = pdFALSE; ++notifications; }

bool bmi088_probe(void) { return (mask & 4) != 0; }
bool bmi088_init(void) { return (fail_mask & 4) == 0; }
bool bmi088_start(void) { return !task_fail; }
uint8_t bmi088_accel_id(void) { return (mask & 4) ? 0x1e : 0; }
uint8_t bmi088_gyro_id(void) { return (mask & 4) ? 0x0f : 0; }

static void near(float actual, float expected)
{ assert(fabsf(actual - expected) < 0.0001f); }
static void be(unsigned offset, int16_t value)
{ raw[offset] = (uint16_t)value >> 8; raw[offset + 1] = (uint8_t)value; }
static void run_parser(void)
{ waits = 0; if (setjmp(blocked) == 0) parser(NULL); }

static void decoder_test(void)
{
    for (unsigned chip = 0; chip < 2; ++chip) {
        bool (*decode)(const uint8_t *, ImuData *) = chip ? icm42688p_decode : icm45686_decode;
        unsigned a = chip ? 2 : 0, g = chip ? 8 : 6, t = chip ? 0 : 12;
        memset(raw, 0, sizeof(raw));
        be(a, 2048); be(a + 2, -4096); be(a + 4, 1024);
        be(g, 164); be(g + 2, -328); be(g + 4, 82); be(t, chip ? 13248 : 12800);
        ImuData out = {0};
        assert(decode(raw, &out));
        near(out.accel[0], 9.80665f); near(out.accel[1], -19.6133f);
        near(out.accel[2], 4.903325f);
        near(out.gyro[0], 0.174532925f); near(out.gyro[1], -0.34906585f);
        near(out.gyro[2], 0.0872664626f); near(out.temp, 125.0f);
        be(t, INT16_MIN); out.temp = 42.0f;
        assert(decode(raw, &out)); near(out.temp, 42.0f);
        for (unsigned offset = 0; offset < 6; offset += 2) {
            be(a + offset, INT16_MIN); assert(!decode(raw, &out)); be(a + offset, 0);
            be(g + offset, INT16_MIN); assert(!decode(raw, &out)); be(g + offset, 0);
        }
    }
}

int main(int argc, char **argv)
{
    assert(argc == 4);
    mask = (unsigned)atoi(argv[1]); fail_mask = (unsigned)atoi(argv[2]);
    task_fail = atoi(argv[3]) != 0;
    decoder_test();
    ImuData out;
    assert(!imu_read(NULL)); assert(!imu_read(&out)); near(out.temp, 0);
    assert(!g_imu_status.running);
    unsigned usable = mask & ~fail_mask;
    imu_type_t expected = (usable & 1) ? IMU_TYPE_ICM45686 :
                          (usable & 2) ? IMU_TYPE_ICM42688 :
                          (usable & 4) ? IMU_TYPE_BMI088 : IMU_TYPE_NONE;
    assert(imu_init() == (expected != IMU_TYPE_NONE));
    assert(imu_get_type() == expected);
    assert(g_imu_detected_mask == mask && g_imu_selftest.done);
    imu_start(); imu_start();
    assert(g_imu_status.running == (expected != IMU_TYPE_NONE && !task_fail));
    if (expected == IMU_TYPE_ICM45686 || expected == IMU_TYPE_ICM42688) {
        if (task_fail) {
            assert(g_imu_status.start_result == INIT_TASK_ERROR && task_count == 0);
        } else {
            assert(task_count == 1 && imu_init()); /* Initialization/start are idempotent. */
            memset(raw, 0, sizeof(raw));
            unsigned a = expected == IMU_TYPE_ICM45686 ? 0 : 2;
            be(a, 2048);
            g_imu_bias.accel[0] = 1.0f; g_imu_bias.valid = 1;
            icm_acq_drdy_isr(); icm_acq_drdy_isr();
            assert(g_imu_status.missed_drdy_count == 1);
            icm_acq_done_isr();
            assert(notifications == 1 && g_imu_status.sample_count == 0);
            /* Second buffer may complete before parsing; third DRDY must drop. */
            icm_acq_drdy_isr(); icm_acq_done_isr(); icm_acq_drdy_isr();
            assert(g_imu_status.missed_drdy_count == 2);
            run_parser();
            assert(g_imu_status.sample_count == 2 && imu_read(&out));
#if defined(BOARD_ORIENT_VERTICAL)
            near(out.accel[0], 0); near(out.accel[1], -10.80665f);
#else
            near(out.accel[0], -10.80665f); /* axis mapping BEFORE bias */
#endif
            assert(out.timestamp == 2 && imu_get_data()->timestamp == out.timestamp);
            ImuData held = out;
            be(a, INT16_MIN); icm_acq_drdy_isr(); icm_acq_done_isr(); run_parser();
            assert(g_imu_status.invalid_sample_count == 1);
            assert(imu_read(&out) && out.timestamp == held.timestamp);
            dma_fail = true; icm_acq_drdy_isr();
            assert(g_imu_status.dma_start_error_count == 1 && cs == GPIO_PIN_SET);
            dma_fail = false; icm_acq_drdy_isr(); icm_acq_error_isr();
            assert(g_imu_status.spi_error_count == 1 && cs == GPIO_PIN_SET);
        }
    }
    puts("PASS");
    return 0;
}
