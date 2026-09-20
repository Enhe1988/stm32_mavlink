/**
 * @file    Icm45686.cpp
 * @brief   ICM-45686 自研最小驱动。
 *
 * 初始化阶段阻塞完成复位、身份检查和寄存器配置。运行阶段由 PE15
 * DATA_RDY 中断直接启动 15 字节 SPI4 TxRx DMA；DMA 完成中断只关闭片选
 * 并通知解析任务，浮点换算和发布均在任务上下文完成。
 */
#include "Icm45686.hpp"
#include "icm45686_regs.h"
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

using namespace icm45686_reg;

namespace {

constexpr uint32_t SPI_TIMEOUT_MS = 100U;
constexpr uint32_t RESET_SETTLE_US = 2000U;
constexpr uint32_t GYRO_STARTUP_US = 50000U;
constexpr uint32_t RESET_POLL_US = 100U;
constexpr uint32_t RESET_POLL_COUNT = 100U;
constexpr uint32_t IREG_WAIT_US = 5U;

constexpr float ACCEL_LSB_PER_G = 2048.0f;
constexpr float GYRO_LSB_PER_DPS = 16.4f;
constexpr float GRAVITY_MPS2 = 9.80665f;
constexpr float DEG_TO_RAD = 0.017453292519943295f;
constexpr float TEMP_SENSITIVITY = 128.0f;
constexpr float TEMP_OFFSET_C = 25.0f;

enum RawBufferState : uint8_t {
    RAW_FREE = 0U,
    RAW_DMA,
    RAW_READY,
    RAW_PROCESSING,
};

Icm45686 *s_icm = nullptr;
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

const ImuData &Icm45686::getData() const
{
    const uint8_t index = published_index_;
    __DMB();
    return data_[index];
}

bool Icm45686::readRegisters(uint8_t reg, uint8_t *data, uint16_t length)
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

bool Icm45686::writeRegisters(uint8_t reg, const uint8_t *data, uint16_t length)
{
    if ((data == nullptr) || (length == 0U) || (length > 3U)) {
        return false;
    }

    uint8_t tx[4] = {reg, 0U, 0U, 0U};
    for (uint16_t i = 0U; i < length; ++i) {
        tx[i + 1U] = data[i];
    }

    chipSelectLow();
    const HAL_StatusTypeDef status = HAL_SPI_Transmit(
        &hspi4, tx, static_cast<uint16_t>(length + 1U), SPI_TIMEOUT_MS);
    chipSelectHigh();
    return status == HAL_OK;
}

bool Icm45686::writeRegister(uint8_t reg, uint8_t value)
{
    return writeRegisters(reg, &value, 1U);
}

bool Icm45686::verifyRegister(uint8_t reg, uint8_t expected, uint8_t mask)
{
    uint8_t value = 0U;
    return readRegisters(reg, &value, 1U) &&
           ((value & mask) == (expected & mask));
}

bool Icm45686::writeIndirect(uint16_t address, uint8_t value)
{
    const uint8_t request[3] = {
        static_cast<uint8_t>(address >> 8U),
        static_cast<uint8_t>(address & 0xFFU),
        value,
    };
    if (!writeRegisters(IREG_ADDR_15_8, request, sizeof(request))) {
        return false;
    }

    for (uint32_t i = 0U; i < 20U; ++i) {
        icm_platform_delay_us(IREG_WAIT_US);
        uint8_t status = 0U;
        if (!readRegisters(REG_MISC2, &status, 1U)) {
            return false;
        }
        if ((status & REG_MISC2_IREG_DONE) != 0U) {
            return true;
        }
    }
    return false;
}

bool Icm45686::readIndirect(uint16_t address, uint8_t *value)
{
    if (value == nullptr) {
        return false;
    }

    const uint8_t request[2] = {
        static_cast<uint8_t>(address >> 8U),
        static_cast<uint8_t>(address & 0xFFU),
    };
    if (!writeRegisters(IREG_ADDR_15_8, request, sizeof(request))) {
        return false;
    }

    icm_platform_delay_us(IREG_WAIT_US);
    return readRegisters(IREG_DATA, value, 1U);
}

bool Icm45686::probe()
{
    chipSelectHigh();
    who_am_i_ = 0U;
    return readRegisters(WHO_AM_I, &who_am_i_, 1U) &&
           (who_am_i_ == WHO_AM_I_VALUE);
}

bool Icm45686::configure()
{
    if (!writeRegister(REG_MISC2, REG_MISC2_SOFT_RESET)) {
        init_rc_ = INIT_SPI_ERROR;
        return false;
    }

    icm_platform_delay_us(RESET_SETTLE_US);

    bool reset_done = false;
    for (uint32_t i = 0U; i < RESET_POLL_COUNT; ++i) {
        uint8_t status = 0U;
        if (!readRegisters(INT1_STATUS0, &status, 1U)) {
            init_rc_ = INIT_SPI_ERROR;
            return false;
        }
        if ((status & INT1_STATUS0_RESET_DONE) != 0U) {
            reset_done = true;
            break;
        }
        icm_platform_delay_us(RESET_POLL_US);
    }
    if (!reset_done) {
        init_rc_ = INIT_RESET_TIMEOUT;
        return false;
    }

    if (!probe()) {
        init_rc_ = (who_am_i_ == 0U) ? INIT_SPI_ERROR : INIT_WHO_AM_I_ERROR;
        return false;
    }

    /* 明确锁定当前板级连接需要的 4-wire SPI mode 0。 */
    if (!writeRegister(INTF_CONFIG1_OVRD, INTF_CONFIG1_OVRD_4WIRE_MODE03) ||
        !verifyRegister(INTF_CONFIG1_OVRD, INTF_CONFIG1_OVRD_4WIRE_MODE03, 0x0FU)) {
        init_rc_ = INIT_CONFIG_ERROR;
        return false;
    }

    /* 数据手册规定传感器寄存器默认小端；统一切到大端后再做连续 DMA 读取。 */
    if (!writeIndirect(IREG_SREG_CTRL, SREG_CTRL_BIG_ENDIAN)) {
        init_rc_ = INIT_SPI_ERROR;
        return false;
    }
    uint8_t sreg_ctrl = 0U;
    if (!readIndirect(IREG_SREG_CTRL, &sreg_ctrl) ||
        ((sreg_ctrl & SREG_CTRL_BIG_ENDIAN) == 0U)) {
        init_rc_ = INIT_CONFIG_ERROR;
        return false;
    }

    /* 传感器关闭时配置量程、ODR、FIFO 和 INT1；start() 后才开放 DRDY。 */
    const struct {
        uint8_t reg;
        uint8_t value;
    } config[] = {
        {INT1_CONFIG0,  0x00U},
        {INT1_CONFIG1,  0x00U},
        {INT1_CONFIG2,  INT1_CONFIG2_PUSH_PULL_PULSE_HIGH},
        {FIFO_CONFIG0,  0x00U},
        {ACCEL_CONFIG0, ACCEL_CONFIG0_16G_800HZ},
        {GYRO_CONFIG0,  GYRO_CONFIG0_2000DPS_800HZ},
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

    /* 手册给出 gyro 启动时间 35 ms；留到 50 ms 后再检查配置。 */
    icm_platform_delay_us(GYRO_STARTUP_US);

    if (!verifyRegister(WHO_AM_I, WHO_AM_I_VALUE, 0xFFU) ||
        !verifyRegister(INT1_CONFIG0, 0x00U, 0xFFU) ||
        !verifyRegister(INT1_CONFIG2, INT1_CONFIG2_PUSH_PULL_PULSE_HIGH, 0x07U) ||
        !verifyRegister(ACCEL_CONFIG0, ACCEL_CONFIG0_16G_800HZ, 0x7FU) ||
        !verifyRegister(GYRO_CONFIG0, GYRO_CONFIG0_2000DPS_800HZ, 0xFFU) ||
        !verifyRegister(PWR_MGMT0, PWR_MGMT0_ACCEL_GYRO_LN, 0x0FU)) {
        init_rc_ = INIT_CONFIG_ERROR;
        return false;
    }

    init_rc_ = INIT_OK;
    return true;
}

bool Icm45686::init()
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
    s_dma_tx[0] = static_cast<uint8_t>(ACCEL_DATA_X1 | READ_BIT);
    for (uint8_t i = 1U; i < SPI_FRAME_BYTES; ++i) {
        s_dma_tx[i] = 0U;
    }

    chipSelectHigh();
    icm_platform_delay_us(3000U);
    return configure();
}

void Icm45686::start()
{
    if ((init_rc_ != INIT_OK) || started_) {
        return;
    }

    if (s_parse_task == nullptr) {
        if (xTaskCreate(&Icm45686::parseTaskTramp, "icm456_parse", 384U, this,
                        configMAX_PRIORITIES - 1U, &s_parse_task) != pdPASS) {
            init_rc_ = INIT_TASK_ERROR;
            return;
        }
    }

    uint8_t ignored = 0U;
    (void)readRegisters(INT1_STATUS0, &ignored, 1U);
    __HAL_GPIO_EXTI_CLEAR_IT(ICM_INT_Pin);
    if (!writeRegister(INT1_CONFIG0, INT1_CONFIG0_DRDY)) {
        init_rc_ = INIT_SPI_ERROR;
        return;
    }
    __HAL_GPIO_EXTI_CLEAR_IT(ICM_INT_Pin);
    started_ = true;
}

void Icm45686::parseTaskTramp(void *arg)
{
    static_cast<Icm45686 *>(arg)->parseTask();
}

void Icm45686::parseTask()
{
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

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

bool Icm45686::parseAndPublish(const uint8_t *sample, uint32_t timestamp_us)
{
    const int16_t accel_raw[3] = {
        decodeBigEndian(&sample[0]),
        decodeBigEndian(&sample[2]),
        decodeBigEndian(&sample[4]),
    };
    const int16_t gyro_raw[3] = {
        decodeBigEndian(&sample[6]),
        decodeBigEndian(&sample[8]),
        decodeBigEndian(&sample[10]),
    };
    const int16_t temp_raw = decodeBigEndian(&sample[12]);

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

    /* 两颗 ICM 占用同一贴片位置和方向，共用板级轴映射。 */
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

void Icm45686::onDrdyISR()
{
    Icm45686 *const self = s_icm;
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

void Icm45686::onSpiDmaDoneISR()
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

void Icm45686::onSpiErrorISR()
{
    chipSelectHigh();
    if (!s_dma_busy) {
        return;
    }

    Icm45686 *const self = s_icm;
    s_raw_state[s_dma_index] = RAW_FREE;
    s_dma_busy = false;
    if (self != nullptr) {
        ++self->spi_error_count_;
    }
}
