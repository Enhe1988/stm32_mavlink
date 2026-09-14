/**
 * @file    Icm42688p.cpp
 * @brief   ICM-42688-P 自研最小驱动。
 *
 * 初始化阶段使用阻塞 SPI 完成复位、身份检查和寄存器配置。运行阶段由
 * PE15 DATA_RDY 中断直接启动一次 15 字节 SPI4 TxRx DMA；DMA 完成中断
 * 关闭片选并通知解析任务。解析、单位换算、轴映射和发布均在任务上下文。
 */
#include "Icm42688p.hpp"
#include "icm42688p_regs.h"
#include "icm_platform.h"
#include "imu_calib.h"
#include "imu_frame.h"
#include "imu_orientation.h"

#include <limits.h>

extern "C" {
#include "FreeRTOS.h"
#include "main.h"
#include "spi.h"
#include "task.h"
}

using namespace icm42688p_reg;

namespace {

constexpr uint32_t SPI_TIMEOUT_MS = 100U;
constexpr uint32_t RESET_SETTLE_US = 2000U;
constexpr uint32_t GYRO_STARTUP_US = 60000U;
constexpr uint32_t RESET_POLL_US = 100U;
constexpr uint32_t RESET_POLL_COUNT = 100U;

constexpr float ACCEL_LSB_PER_G = 2048.0f;
constexpr float GYRO_LSB_PER_DPS = 16.4f;
constexpr float GRAVITY_MPS2 = 9.80665f;
constexpr float DEG_TO_RAD = 0.017453292519943295f;
constexpr float TEMP_SENSITIVITY = 132.48f;
constexpr float TEMP_OFFSET_C = 25.0f;

enum RawBufferState : uint8_t {
    RAW_FREE = 0U,
    RAW_DMA,
    RAW_READY,
    RAW_PROCESSING,
};

Icm42688p *s_icm = nullptr;
TaskHandle_t s_parse_task = nullptr;

/* SPI4 DMA 只能访问 RAM_D2；该段由 icm_platform_dma_region_init 配成非缓存。 */
__attribute__((section(".dma_buffer"), aligned(32)))
uint8_t s_dma_tx[SPI_FRAME_BYTES] = {0U};

__attribute__((section(".dma_buffer"), aligned(32)))
uint8_t s_dma_rx[2][SPI_FRAME_BYTES] = {{0U}};

volatile RawBufferState s_raw_state[2] = {RAW_FREE, RAW_FREE};
volatile bool s_dma_busy = false;
volatile uint8_t s_dma_index = 0U;
uint32_t s_raw_timestamp_us[2] = {0U, 0U};

inline void chipSelectLow()
{
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
}

inline void chipSelectHigh()
{
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
}

inline int16_t decodeBigEndian(const uint8_t *p)
{
    return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8U) |
                                static_cast<uint16_t>(p[1]));
}

} /* namespace */

extern "C" {

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == ICM_INT_Pin) {
        Icm42688p::onDrdyISR();
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *spi)
{
    if (spi->Instance == SPI4) {
        Icm42688p::onSpiDmaDoneISR();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi)
{
    if (spi->Instance == SPI4) {
        Icm42688p::onSpiErrorISR();
    }
}

} /* extern "C" */

const ImuData &Icm42688p::getData() const
{
    const uint8_t index = published_index_;
    __DMB();
    return data_[index];
}

bool Icm42688p::readRegisters(uint8_t reg, uint8_t *data, uint16_t length)
{
    if ((data == nullptr) || (length == 0U) || (length > SENSOR_DATA_BYTES)) {
        return false;
    }

    uint8_t tx[SPI_FRAME_BYTES] = {0U};
    uint8_t rx[SPI_FRAME_BYTES] = {0U};
    tx[0] = static_cast<uint8_t>(reg | READ_BIT);

    chipSelectLow();
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi4, tx, rx, static_cast<uint16_t>(length + 1U), SPI_TIMEOUT_MS);
    chipSelectHigh();

    if (status != HAL_OK) {
        return false;
    }

    for (uint16_t i = 0U; i < length; ++i) {
        data[i] = rx[i + 1U];
    }
    return true;
}

bool Icm42688p::writeRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value};
    chipSelectLow();
    const HAL_StatusTypeDef status =
        HAL_SPI_Transmit(&hspi4, tx, sizeof(tx), SPI_TIMEOUT_MS);
    chipSelectHigh();
    return status == HAL_OK;
}

bool Icm42688p::selectBank(uint8_t bank)
{
    return writeRegister(REG_BANK_SEL, static_cast<uint8_t>(bank & 0x07U));
}

bool Icm42688p::verifyRegister(uint8_t reg, uint8_t expected, uint8_t mask)
{
    uint8_t value = 0U;
    return readRegisters(reg, &value, 1U) &&
           ((value & mask) == (expected & mask));
}

bool Icm42688p::configure()
{
    if (!selectBank(BANK0) ||
        !writeRegister(DEVICE_CONFIG, DEVICE_CONFIG_SOFT_RESET)) {
        init_rc_ = INIT_SPI_ERROR;
        return false;
    }

    icm_platform_delay_us(RESET_SETTLE_US);

    bool reset_done = false;
    for (uint32_t i = 0U; i < RESET_POLL_COUNT; ++i) {
        uint8_t status = 0U;
        if (!readRegisters(INT_STATUS, &status, 1U)) {
            init_rc_ = INIT_SPI_ERROR;
            return false;
        }
        if ((status & INT_STATUS_RESET_DONE) != 0U) {
            reset_done = true;
            break;
        }
        icm_platform_delay_us(RESET_POLL_US);
    }
    if (!reset_done) {
        init_rc_ = INIT_RESET_TIMEOUT;
        return false;
    }

    if (!readRegisters(WHO_AM_I, &who_am_i_, 1U)) {
        init_rc_ = INIT_SPI_ERROR;
        return false;
    }
    if (who_am_i_ != WHO_AM_I_VALUE) {
        init_rc_ = INIT_WHO_AM_I_ERROR;
        return false;
    }

    /* 手册“Settings for SPI”建议：4-wire SPI，并保持 I3C 能力位为 1。 */
    uint8_t intf_config4 = 0U;
    uint8_t intf_config6 = 0U;
    if (!selectBank(BANK1) ||
        !readRegisters(INTF_CONFIG4, &intf_config4, 1U) ||
        !readRegisters(INTF_CONFIG6, &intf_config6, 1U)) {
        init_rc_ = INIT_SPI_ERROR;
        return false;
    }
    intf_config4 = static_cast<uint8_t>(
        (intf_config4 & ~INTF_CONFIG4_I3C_BUS_MODE) | INTF_CONFIG4_SPI_4WIRE);
    intf_config6 = static_cast<uint8_t>(
        intf_config6 | INTF_CONFIG6_I3C_EN |
        INTF_CONFIG6_I3C_DDR_EN | INTF_CONFIG6_I3C_SDR_EN);
    if (!writeRegister(INTF_CONFIG4, intf_config4) ||
        !writeRegister(INTF_CONFIG6, intf_config6) ||
        !selectBank(BANK0)) {
        init_rc_ = INIT_SPI_ERROR;
        return false;
    }

    /* 先配置接口、量程、ODR、滤波与中断，最后才开启两个传感器。 */
    const struct {
        uint8_t reg;
        uint8_t value;
    } config[] = {
        {DRIVE_CONFIG,       DRIVE_CONFIG_SPI_SLEW_5},
        {INTF_CONFIG0,       INTF_CONFIG0_SPI_ONLY},
        {FIFO_CONFIG,        0x00U},
        {INT_CONFIG,         INT_CONFIG_INT1},
        {INT_CONFIG0,        0x00U},
        {INT_CONFIG1,        0x00U}, /* 清 INT_ASYNC_RESET，INT 引脚才能正常工作 */
        {INT_SOURCE0,        0x00U}, /* start() 创建解析任务后再开放 DATA_RDY */
        {GYRO_CONFIG0,       GYRO_CONFIG0_2000DPS_1KHZ},
        {ACCEL_CONFIG0,      ACCEL_CONFIG0_16G_1KHZ},
        {GYRO_ACCEL_CONFIG0, GYRO_ACCEL_CONFIG0_DEFAULT},
    };

    for (const auto &entry : config) {
        if (!writeRegister(entry.reg, entry.value)) {
            init_rc_ = INIT_SPI_ERROR;
            return false;
        }
    }

    if (!writeRegister(PWR_MGMT0, PWR_MGMT0_ACCEL_GYRO_LN)) {
        init_rc_ = INIT_SPI_ERROR;
        return false;
    }

    /* PWR_MGMT0 后至少 200 us 不可访问寄存器；60 ms 同时覆盖陀螺启动。 */
    icm_platform_delay_us(GYRO_STARTUP_US);

    if (!verifyRegister(WHO_AM_I, WHO_AM_I_VALUE, 0xFFU) ||
        !verifyRegister(INTF_CONFIG0, INTF_CONFIG0_SPI_ONLY, 0x33U) ||
        !verifyRegister(INT_CONFIG, INT_CONFIG_INT1, 0x07U) ||
        !verifyRegister(INT_CONFIG1, 0x00U, 0x10U) ||
        !verifyRegister(GYRO_CONFIG0, GYRO_CONFIG0_2000DPS_1KHZ, 0xEFU) ||
        !verifyRegister(ACCEL_CONFIG0, ACCEL_CONFIG0_16G_1KHZ, 0xEFU) ||
        !verifyRegister(PWR_MGMT0, PWR_MGMT0_ACCEL_GYRO_LN, 0x3FU)) {
        init_rc_ = INIT_CONFIG_ERROR;
        return false;
    }

    init_rc_ = INIT_OK;
    return true;
}

bool Icm42688p::init()
{
    icm_platform_timer_init();
    icm_platform_dma_region_init();

    s_icm = this;
    started_ = false;
    published_index_ = 0U;
    data_[0] = {};
    data_[1] = {};
    who_am_i_ = 0U;
    init_rc_ = INIT_OK;
    sample_count_ = 0U;
    drdy_count_ = 0U;
    missed_drdy_count_ = 0U;
    dma_start_error_count_ = 0U;
    spi_error_count_ = 0U;
    invalid_sample_count_ = 0U;

    s_dma_busy = false;
    s_dma_index = 0U;
    s_raw_state[0] = RAW_FREE;
    s_raw_state[1] = RAW_FREE;
    s_dma_tx[0] = static_cast<uint8_t>(TEMP_DATA1 | READ_BIT);
    for (uint8_t i = 1U; i < SPI_FRAME_BYTES; ++i) {
        s_dma_tx[i] = 0U;
    }

    chipSelectHigh();
    icm_platform_delay_us(3000U);
    return configure();
}

void Icm42688p::start()
{
    if ((init_rc_ != INIT_OK) || started_) {
        return;
    }

    if (s_parse_task == nullptr) {
        if (xTaskCreate(&Icm42688p::parseTaskTramp, "icm_parse", 384U, this,
                        configMAX_PRIORITIES - 1U, &s_parse_task) != pdPASS) {
            init_rc_ = INIT_TASK_ERROR;
            return;
        }
    }

    __HAL_GPIO_EXTI_CLEAR_IT(ICM_INT_Pin);
    if (!selectBank(BANK0) ||
        !writeRegister(INT_SOURCE0, INT_SOURCE0_UI_DRDY)) {
        init_rc_ = INIT_SPI_ERROR;
        return;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(ICM_INT_Pin);
    started_ = true;
}

void Icm42688p::parseTaskTramp(void *arg)
{
    static_cast<Icm42688p *>(arg)->parseTask();
}

void Icm42688p::parseTask()
{
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 通知只负责唤醒；一次醒来把所有 READY 缓冲都处理完。 */
        for (;;) {
            int index = -1;
            taskENTER_CRITICAL();
            for (int i = 0; i < 2; ++i) {
                if (s_raw_state[i] == RAW_READY) {
                    s_raw_state[i] = RAW_PROCESSING;
                    index = i;
                    break;
                }
            }
            taskEXIT_CRITICAL();

            if (index < 0) {
                break;
            }

            (void)parseAndPublish(&s_dma_rx[index][1], s_raw_timestamp_us[index]);

            taskENTER_CRITICAL();
            s_raw_state[index] = RAW_FREE;
            taskEXIT_CRITICAL();
        }
    }
}

bool Icm42688p::parseAndPublish(const uint8_t *sample, uint32_t timestamp_us)
{
    const int16_t temp_raw = decodeBigEndian(&sample[0]);
    const int16_t accel_raw[3] = {
        decodeBigEndian(&sample[2]),
        decodeBigEndian(&sample[4]),
        decodeBigEndian(&sample[6]),
    };
    const int16_t gyro_raw[3] = {
        decodeBigEndian(&sample[8]),
        decodeBigEndian(&sample[10]),
        decodeBigEndian(&sample[12]),
    };

    for (int i = 0; i < 3; ++i) {
        if ((accel_raw[i] == INT16_MIN) || (gyro_raw[i] == INT16_MIN)) {
            ++invalid_sample_count_;
            return false;
        }
    }

    float accel[3];
    float gyro[3];
    for (int i = 0; i < 3; ++i) {
        accel[i] = (static_cast<float>(accel_raw[i]) / ACCEL_LSB_PER_G) * GRAVITY_MPS2;
        gyro[i] = (static_cast<float>(gyro_raw[i]) / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    }

    imu_axis_apply(&IMU_MAP_ICM_ACCEL, accel);
    imu_axis_apply(&IMU_MAP_ICM_GYRO, gyro);
    imu_calib_apply(gyro, accel);
    imu_orient_apply(gyro);
    imu_orient_apply(accel);

    ImuData next = {};
    for (int i = 0; i < 3; ++i) {
        next.accel[i] = accel[i];
        next.gyro[i] = gyro[i];
    }
    next.temp = (temp_raw == INT16_MIN)
                    ? data_[published_index_].temp
                    : static_cast<float>(temp_raw) / TEMP_SENSITIVITY + TEMP_OFFSET_C;
    next.timestamp = timestamp_us;

    const uint8_t next_index = static_cast<uint8_t>(published_index_ ^ 1U);
    data_[next_index] = next;
    __DMB();
    published_index_ = next_index;
    ++sample_count_;
    return true;
}

void Icm42688p::onDrdyISR()
{
    Icm42688p *const self = s_icm;
    if ((self == nullptr) || !self->started_) {
        return;
    }

    ++self->drdy_count_;
    if (s_dma_busy) {
        ++self->missed_drdy_count_;
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
        ++self->missed_drdy_count_;
        return;
    }

    s_raw_state[index] = RAW_DMA;
    s_dma_index = static_cast<uint8_t>(index);
    s_dma_busy = true;
    s_raw_timestamp_us[index] = static_cast<uint32_t>(icm_platform_get_time_us());
    __DMB();

    chipSelectLow();
    if (HAL_SPI_TransmitReceive_DMA(&hspi4, s_dma_tx, s_dma_rx[index],
                                    SPI_FRAME_BYTES) != HAL_OK) {
        chipSelectHigh();
        s_raw_state[index] = RAW_FREE;
        s_dma_busy = false;
        ++self->dma_start_error_count_;
    }
}

void Icm42688p::onSpiDmaDoneISR()
{
    if (!s_dma_busy) {
        return;
    }

    chipSelectHigh();
    const uint8_t index = s_dma_index;
    __DMB();
    s_raw_state[index] = RAW_READY;
    s_dma_busy = false;

    BaseType_t higher_priority_woken = pdFALSE;
    if (s_parse_task != nullptr) {
        vTaskNotifyGiveFromISR(s_parse_task, &higher_priority_woken);
    }
    portYIELD_FROM_ISR(higher_priority_woken);
}

void Icm42688p::onSpiErrorISR()
{
    chipSelectHigh();
    if (!s_dma_busy) {
        return;
    }

    Icm42688p *const self = s_icm;
    s_raw_state[s_dma_index] = RAW_FREE;
    s_dma_busy = false;
    if (self != nullptr) {
        ++self->spi_error_count_;
    }
}
