/**
 * @file    Icm45686.hpp
 * @brief   ICM-45686 最小驱动：阻塞初始化，DRDY 触发 SPI4 DMA，任务解析。
 */
#ifndef ICM45686_HPP
#define ICM45686_HPP

#include "imu.hpp"

class Icm45686 : public ImuDriver {
public:
    bool probe() override;
    bool init() override;
    void start() override;
    const ImuData &getData() const override;

    uint8_t whoAmI() const { return who_am_i_; }
    /* 0=成功；-1=SPI；-2=复位超时；-3=ID；-4=配置回读；-5=建任务。 */
    int initRc() const { return init_rc_; }
    uint32_t sampleCount() const { return sample_count_; }
    uint32_t drdyCount() const { return drdy_count_; }
    uint32_t missedDrdyCount() const { return missed_drdy_count_; }
    uint32_t dmaStartErrorCount() const { return dma_start_error_count_; }
    uint32_t spiErrorCount() const { return spi_error_count_; }
    uint32_t invalidSampleCount() const { return invalid_sample_count_; }

    static void onDrdyISR();
    static void onSpiDmaDoneISR();
    static void onSpiErrorISR();

private:
    enum InitResult : int {
        INIT_OK             = 0,
        INIT_SPI_ERROR      = -1,
        INIT_RESET_TIMEOUT  = -2,
        INIT_WHO_AM_I_ERROR = -3,
        INIT_CONFIG_ERROR   = -4,
        INIT_TASK_ERROR     = -5,
    };

    static void parseTaskTramp(void *arg);
    void parseTask();

    bool configure();
    bool readRegisters(uint8_t reg, uint8_t *data, uint16_t length);
    bool writeRegisters(uint8_t reg, const uint8_t *data, uint16_t length);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool verifyRegister(uint8_t reg, uint8_t expected, uint8_t mask);
    bool writeIndirect(uint16_t address, uint8_t value);
    bool readIndirect(uint16_t address, uint8_t *value);
    bool parseAndPublish(const uint8_t *sample, uint32_t timestamp_us);

    ImuData data_[2]{};
    volatile uint8_t published_index_ = 0U;
    uint8_t who_am_i_ = 0U;
    volatile int init_rc_ = INIT_OK;
    volatile bool started_ = false;

    volatile uint32_t sample_count_ = 0U;
    volatile uint32_t drdy_count_ = 0U;
    volatile uint32_t missed_drdy_count_ = 0U;
    volatile uint32_t dma_start_error_count_ = 0U;
    volatile uint32_t spi_error_count_ = 0U;
    volatile uint32_t invalid_sample_count_ = 0U;
};

#endif /* ICM45686_HPP */
