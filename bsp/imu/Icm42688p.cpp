/**
 * @file    Icm42688p.cpp
 * @brief   ICM-42688-P 驱动实现。
 *          阶段(设计文档 §10 第3步):阻塞版 serif(HAL_SPI_TransmitReceive)
 *          + 官方 inv_icm426xx_init + get_who_am_i 自检(读 0x47)。
 *          DMA 后端 / EXTI 采集任务为后续步骤。
 */
#include "Icm42688p.hpp"
#include "icm_platform.h"
#include "imu_orientation.h"
#include "imu_frame.h"
#include "imu_calib.h"

extern "C" {
#include "main.h"   // ICM_CS_Pin / ICM_CS_GPIO_Port
#include "spi.h"    // hspi4
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
}

/* 单次 SPI 传输的最大数据字节数(不含地址字节)。寄存器模式下读写都很小。 */
#define ICM_SPI_MAX_XFER 32U
#define ICM_SPI_TIMEOUT  100U   /* ms */

/* ── 量程标定常数(与 init 里设的 FSR 一致)──
 * ±16g  → 2048 LSB/g ;  ±2000dps → 16.4 LSB/dps ; 温度 raw/132.48 + 25 */
#define ICM_ACCEL_LSB_PER_G   2048.0f
#define ICM_GYRO_LSB_PER_DPS  16.4f
#define ICM_G_TO_MPS2         9.80665f
#define ICM_DPS_TO_RADS       0.017453292519943295f   /* π/180 */
#define ICM_TEMP_SENS         132.48f
#define ICM_TEMP_OFFSET_C     25.0f

/* sensorEventCb 是无 context 的 C 回调,用文件作用域指针指回当前实例。
 * 实际只有一个 ICM,init() 里赋值即可。 */
static Icm42688p *s_icm_self = nullptr;

/* ── SPI4 DMA 后端(设计文档 §3/§5)──
 * buffer 放 .dma_buffer(RAM_D2,MPU 非缓存);read/write 启 DMA 后阻塞等完成信号量。 */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t s_dma_tx[1 + ICM_SPI_MAX_XFER];
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t s_dma_rx[1 + ICM_SPI_MAX_XFER];

static SemaphoreHandle_t s_spi_dma_done = nullptr;   /* 单次 DMA 传输完成 */
static TaskHandle_t      s_acq_task     = nullptr;   /* DRDY 驱动的采集任务 */

/* ── serif 总线读写(C 语言链接,可赋给官方驱动的 C 函数指针)── */
extern "C" {

static inline void icm_cs_low(void)
{
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
}
static inline void icm_cs_high(void)
{
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
}

/* 等单次 DMA 传输完成信号量(在任务上下文调用)。超时返回 false。 */
static bool icm_wait_dma_done(void)
{
    return (xSemaphoreTake(s_spi_dma_done, pdMS_TO_TICKS(ICM_SPI_TIMEOUT)) == pdTRUE);
}

/* 读:发送 (reg|0x80) 后读 len 字节。total = len+1。走 DMA + 完成信号量。 */
static int icm_serif_read_reg(struct inv_icm426xx_serif * /*serif*/, uint8_t reg,
                              uint8_t *buf, uint32_t len)
{
    if (len > ICM_SPI_MAX_XFER) {
        return -1;
    }
    s_dma_tx[0] = (uint8_t)(reg | 0x80U);
    for (uint32_t i = 0; i < len; ++i) {
        s_dma_tx[i + 1] = 0;
    }

    icm_cs_low();
    if (HAL_SPI_TransmitReceive_DMA(&hspi4, s_dma_tx, s_dma_rx, (uint16_t)(len + 1)) != HAL_OK ||
        !icm_wait_dma_done()) {
        icm_cs_high();
        return -1;
    }
    icm_cs_high();

    for (uint32_t i = 0; i < len; ++i) {
        buf[i] = s_dma_rx[i + 1];   /* RAM_D2 非缓存,直接读即最新 DMA 结果 */
    }
    return 0;
}

/* 写:发送 reg 后写 len 字节。total = len+1。走 DMA + 完成信号量。 */
static int icm_serif_write_reg(struct inv_icm426xx_serif * /*serif*/, uint8_t reg,
                               const uint8_t *buf, uint32_t len)
{
    if (len > ICM_SPI_MAX_XFER) {
        return -1;
    }
    s_dma_tx[0] = reg;
    for (uint32_t i = 0; i < len; ++i) {
        s_dma_tx[i + 1] = buf[i];
    }

    icm_cs_low();
    if (HAL_SPI_Transmit_DMA(&hspi4, s_dma_tx, (uint16_t)(len + 1)) != HAL_OK ||
        !icm_wait_dma_done()) {
        icm_cs_high();
        return -1;
    }
    icm_cs_high();
    return 0;
}

/* ── HAL DMA 完成回调:按 Instance 分发(BMI088 走 SPI1 阻塞,不进这里)── */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h)
{
    if (h->Instance == SPI4) {
        Icm42688p::onSpiDmaDoneISR();
    }
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *h)
{
    if (h->Instance == SPI4) {
        Icm42688p::onSpiDmaDoneISR();
    }
}

/* ── EXTI 回调:PE15(ICM DRDY)→ 通知采集任务 ── */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == ICM_INT_Pin) {
        Icm42688p::onDrdyISR();
    }
}

} /* extern "C" */

bool Icm42688p::init()
{
    icm_platform_timer_init();
    icm_platform_dma_region_init();   /* DMA buffer 区配非缓存(用 DMA 前) */
    s_icm_self = this;

    /* 单次 DMA 完成信号量(只建一次) */
    if (s_spi_dma_done == nullptr) {
        s_spi_dma_done = xSemaphoreCreateBinary();
        if (s_spi_dma_done == nullptr) {
            return false;
        }
    }

    /* CS 空闲拉高 */
    icm_cs_high();

    /* 配置 serif */
    serif_.context    = this;
    serif_.read_reg   = icm_serif_read_reg;
    serif_.write_reg  = icm_serif_write_reg;
    serif_.configure  = nullptr;            /* SPI4 路径不会被调用 */
    serif_.max_read   = ICM_SPI_MAX_XFER;
    serif_.max_write  = ICM_SPI_MAX_XFER;
    serif_.serif_type = ICM426XX_UI_SPI4;

    /* 官方 init(内部含上电延时 + 串口配置 + 硬件默认配置) */
    init_rc_ = inv_icm426xx_init(&drv_, &serif_, &Icm42688p::sensorEventCb);

    /* 读 WHO_AM_I 校验通信(期望 0x47) */
    who_am_i_ = 0;
    inv_icm426xx_get_who_am_i(&drv_, &who_am_i_);
    if (who_am_i_ != 0x47) {
        return false;
    }

    /* ── 数据源配置(寄存器模式,非 FIFO)── */
    inv_icm426xx_configure_fifo(&drv_, INV_ICM426XX_FIFO_DISABLED);
    inv_icm426xx_set_accel_fsr(&drv_, ICM426XX_ACCEL_CONFIG0_FS_SEL_16g);
    inv_icm426xx_set_gyro_fsr(&drv_, ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps);
    inv_icm426xx_set_accel_frequency(&drv_, ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ);
    inv_icm426xx_set_gyro_frequency(&drv_, ICM426XX_GYRO_CONFIG0_ODR_1_KHZ);
    inv_icm426xx_enable_accel_low_noise_mode(&drv_);
    inv_icm426xx_enable_gyro_low_noise_mode(&drv_);

    /* 陀螺/加速度上电后需稳定时间(手册:陀螺 ~60ms),首批样本应丢弃。
     * 这里阻塞等到陀螺启动完成,调用方拿到的首帧即有效。 */
    inv_icm426xx_sleep_us(ICM426XX_GYR_STARTUP_TIME_US);

    return true;
}

void Icm42688p::start()
{
    /* 把 DRDY 路由到 INT1(官方 init 默认未路由到 INT1)。
     * 电气(推挽/高有效/脉冲)已由 inv_icm426xx_init 配好。 */
    inv_icm426xx_interrupt_parameter_t int1 = {};   /* 全部字段 = DISABLE */
    int1.INV_ICM426XX_UI_DRDY = INV_ICM426XX_ENABLE;
    inv_icm426xx_set_config_int1(&drv_, &int1);

    /* 起采集任务;PE15 EXTI 已由 CubeMX 在 gpio.c 使能 */
    if (s_acq_task == nullptr) {
        xTaskCreate(&Icm42688p::acqTaskTramp, "icm_acq", 384, this,
                    configMAX_PRIORITIES - 1, &s_acq_task);
    }
}

void Icm42688p::acqTaskTramp(void *arg)
{
    static_cast<Icm42688p *>(arg)->acqTask();
}

void Icm42688p::acqTask()
{
    for (;;) {
        /* 阻塞等 DRDY 通知(EXTI→onDrdyISR)。只在收到中断时读,
         * 这样 sample_count_ 的增长本身即证明 EXTI 链路工作。 */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        inv_icm426xx_get_data_from_registers(&drv_);   /* → sensorEventCb 填 data_ */
    }
}

int Icm42688p::poll()
{
    /* 同步读数据寄存器;官方驱动内部对每帧调用 sensorEventCb 填 data_ */
    return inv_icm426xx_get_data_from_registers(&drv_);
}

void Icm42688p::sensorEventCb(inv_icm426xx_sensor_event_t *event)
{
    if (s_icm_self == nullptr || event == nullptr) {
        return;
    }
    ImuData &d = s_icm_self->data_;

    /* ① 芯片系物理量(raw int16 → m/s² / rad/s)。寄存器读时三者都有效。 */
    float a[3];   /* 加速度 m/s²,芯片轴 */
    float g[3];   /* 角速度 rad/s,芯片轴 */
    for (int i = 0; i < 3; ++i) {
        a[i] = (float)event->accel[i] / ICM_ACCEL_LSB_PER_G * ICM_G_TO_MPS2;
        g[i] = (float)event->gyro[i]  / ICM_GYRO_LSB_PER_DPS * ICM_DPS_TO_RADS;
    }

    /* ② 传感器系 → 机体系(统一映射,标定改 imu_frame.c) */
    imu_axis_apply(&IMU_MAP_ICM_ACCEL, a);
    imu_axis_apply(&IMU_MAP_ICM_GYRO,  g);
    d.accel[0] = a[0]; d.accel[1] = a[1]; d.accel[2] = a[2];
    d.gyro[0]  = g[0]; d.gyro[1]  = g[1]; d.gyro[2]  = g[2];

    d.temp      = (float)event->temperature / ICM_TEMP_SENS + ICM_TEMP_OFFSET_C;
    d.timestamp = (uint32_t)inv_icm426xx_get_time_us();

    /* ③ 板子系零偏扣除(放置旋转之前,与平放/竖放无关;未标定时不动) */
    imu_calib_apply(d.gyro, d.accel);

    /* ④ 板子放置旋转(平放=恒等;竖放见 imu_orientation.h) */
    imu_orient_apply(d.gyro);
    imu_orient_apply(d.accel);

    s_icm_self->sample_count_++;
}

void Icm42688p::onDrdyISR()
{
    BaseType_t hpw = pdFALSE;
    if (s_acq_task != nullptr) {
        vTaskNotifyGiveFromISR(s_acq_task, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}

void Icm42688p::onSpiDmaDoneISR()
{
    BaseType_t hpw = pdFALSE;
    if (s_spi_dma_done != nullptr) {
        xSemaphoreGiveFromISR(s_spi_dma_done, &hpw);
    }
    portYIELD_FROM_ISR(hpw);
}
