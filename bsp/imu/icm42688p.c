/** @file icm42688p.c
 * Chip-specific blocking setup and raw sample decoding. Runtime DMA is in icm_acq.c.
 */

#include "icm42688p.h"
#include "icm42688p_regs.h"
#include "icm_acq.h"
#include "bsp_platform.h"
#include <limits.h>

#define RESET_SETTLE_US (2000U)
#define GYRO_STARTUP_US (60000U)
#define RESET_POLL_US (100U)
#define RESET_POLL_COUNT (100U)
#define ACCEL_LSB_PER_G (2048.0f)
#define GYRO_LSB_PER_DPS (16.4f)
#define GRAVITY_MPS2 (9.80665f)
#define DEG_TO_RAD (0.017453292519943295f)
#define TEMP_SENSITIVITY (132.48f)
#define TEMP_OFFSET_C (25.0f)

static uint8_t s_who_am_i;
static int s_init_rc;

static bool select_bank(uint8_t bank);

bool icm42688p_probe(void)
{
    icm_cs_high();
    s_who_am_i = 0U;
    /* MCU 单独复位时芯片可能仍停在非 0 bank，先显式切回 bank 0。 */
    return icm_write_register(ICM42688_REG_BANK_SEL, ICM42688_BANK0) &&
           icm_read_registers(ICM42688_WHO_AM_I, &s_who_am_i, 1U) &&
           (s_who_am_i == ICM42688_WHO_AM_I_VALUE);
}

static bool select_bank(uint8_t bank)
{
    return icm_write_register(ICM42688_REG_BANK_SEL, (uint8_t)(bank & 0x07U));
}

static bool configure(void)
{
    if (!select_bank(ICM42688_BANK0) ||
        !icm_write_register(ICM42688_DEVICE_CONFIG, ICM42688_DEVICE_CONFIG_SOFT_RESET)) {
        s_init_rc = INIT_SPI_ERROR;
        return false;
    }

    bsp_delay_us(RESET_SETTLE_US);

    bool reset_done = false;
    for (uint32_t i = 0U; i < RESET_POLL_COUNT; ++i) {
        uint8_t status = 0U;
        if (!icm_read_registers(ICM42688_INT_STATUS, &status, 1U)) {
            s_init_rc = INIT_SPI_ERROR;
            return false;
        }
        if ((status & ICM42688_INT_STATUS_RESET_DONE) != 0U) {
            reset_done = true;
            break;
        }
        bsp_delay_us(RESET_POLL_US);
    }
    if (!reset_done) {
        s_init_rc = INIT_RESET_TIMEOUT;
        return false;
    }

    if (!icm_read_registers(ICM42688_WHO_AM_I, &s_who_am_i, 1U)) {
        s_init_rc = INIT_SPI_ERROR;
        return false;
    }
    if (s_who_am_i != ICM42688_WHO_AM_I_VALUE) {
        s_init_rc = INIT_WHO_AM_I_ERROR;
        return false;
    }

    /* 手册“Settings for SPI”建议：4-wire SPI，并保持 I3C 能力位为 1。 */
    uint8_t intf_config4 = 0U;
    uint8_t intf_config6 = 0U;
    if (!select_bank(ICM42688_BANK1) ||
        !icm_read_registers(ICM42688_INTF_CONFIG4, &intf_config4, 1U) ||
        !icm_read_registers(ICM42688_INTF_CONFIG6, &intf_config6, 1U)) {
        s_init_rc = INIT_SPI_ERROR;
        return false;
    }
    intf_config4 = (uint8_t)(
        (intf_config4 & ~ICM42688_INTF_CONFIG4_I3C_BUS_MODE) | ICM42688_INTF_CONFIG4_SPI_4WIRE);
    intf_config6 = (uint8_t)(
        intf_config6 | ICM42688_INTF_CONFIG6_I3C_EN |
        ICM42688_INTF_CONFIG6_I3C_DDR_EN | ICM42688_INTF_CONFIG6_I3C_SDR_EN);
    if (!icm_write_register(ICM42688_INTF_CONFIG4, intf_config4) ||
        !icm_write_register(ICM42688_INTF_CONFIG6, intf_config6) ||
        !select_bank(ICM42688_BANK0)) {
        s_init_rc = INIT_SPI_ERROR;
        return false;
    }

    /* 先配置接口、量程、ODR、滤波与中断，最后才开启两个传感器。 */
    const struct {
        uint8_t reg;
        uint8_t value;
    } config[] = {
        {ICM42688_DRIVE_CONFIG,       ICM42688_DRIVE_CONFIG_SPI_SLEW_5},
        {ICM42688_INTF_CONFIG0,       ICM42688_INTF_CONFIG0_SPI_ONLY},
        {ICM42688_FIFO_CONFIG,        0x00U},
        {ICM42688_INT_CONFIG,         ICM42688_INT_CONFIG_INT1},
        {ICM42688_INT_CONFIG0,        0x00U},
        {ICM42688_INT_CONFIG1,        0x00U}, /* 清 INT_ASYNC_RESET，INT 引脚才能正常工作 */
        {ICM42688_INT_SOURCE0,        0x00U}, /* icm_acq_start() 创建解析任务后再开放 DATA_RDY */
        {ICM42688_GYRO_CONFIG0,       ICM42688_GYRO_CONFIG0_2000DPS_1KHZ},
        {ICM42688_ACCEL_CONFIG0,      ICM42688_ACCEL_CONFIG0_16G_1KHZ},
        {ICM42688_GYRO_ACCEL_CONFIG0, ICM42688_GYRO_ACCEL_CONFIG0_DEFAULT},
    };

    for (unsigned i = 0U; i < sizeof(config) / sizeof(config[0]); ++i) {
        if (!icm_write_register(config[i].reg, config[i].value)) {
            s_init_rc = INIT_SPI_ERROR;
            return false;
        }
    }

    if (!icm_write_register(ICM42688_PWR_MGMT0, ICM42688_PWR_MGMT0_ACCEL_GYRO_LN)) {
        s_init_rc = INIT_SPI_ERROR;
        return false;
    }

    /* ICM42688_PWR_MGMT0 后至少 200 us 不可访问寄存器；60 ms 同时覆盖陀螺启动。 */
    bsp_delay_us(GYRO_STARTUP_US);

    if (!icm_verify_register(ICM42688_WHO_AM_I, ICM42688_WHO_AM_I_VALUE, 0xFFU) ||
        !icm_verify_register(ICM42688_INTF_CONFIG0, ICM42688_INTF_CONFIG0_SPI_ONLY, 0x33U) ||
        !icm_verify_register(ICM42688_INT_CONFIG, ICM42688_INT_CONFIG_INT1, 0x07U) ||
        !icm_verify_register(ICM42688_INT_CONFIG1, 0x00U, 0x10U) ||
        !icm_verify_register(ICM42688_GYRO_CONFIG0, ICM42688_GYRO_CONFIG0_2000DPS_1KHZ, 0xEFU) ||
        !icm_verify_register(ICM42688_ACCEL_CONFIG0, ICM42688_ACCEL_CONFIG0_16G_1KHZ, 0xEFU) ||
        !icm_verify_register(ICM42688_PWR_MGMT0, ICM42688_PWR_MGMT0_ACCEL_GYRO_LN, 0x3FU)) {
        s_init_rc = INIT_CONFIG_ERROR;
        return false;
    }

    s_init_rc = INIT_OK;
    return true;
}

bool icm42688p_init(void)
{
    s_init_rc = INIT_OK;
    icm_cs_high();
    bsp_delay_us(3000U);
    return configure();
}

bool icm42688p_enable_drdy(void)
{
    return select_bank(ICM42688_BANK0) &&
           icm_write_register(ICM42688_INT_SOURCE0, ICM42688_INT_SOURCE0_UI_DRDY);
}

uint8_t icm42688p_who_am_i(void) { return s_who_am_i; }
int icm42688p_init_result(void) { return s_init_rc; }

bool icm42688p_decode(const uint8_t *sample, ImuData *out)
{
    const int16_t temp_raw = icm_decode_be16(&sample[0]);
    const int16_t accel_raw[3] = {
        icm_decode_be16(&sample[2]),
        icm_decode_be16(&sample[4]),
        icm_decode_be16(&sample[6]),
    };
    const int16_t gyro_raw[3] = {
        icm_decode_be16(&sample[8]),
        icm_decode_be16(&sample[10]),
        icm_decode_be16(&sample[12]),
    };

    for (int i = 0; i < 3; ++i) {
        if ((accel_raw[i] == INT16_MIN) || (gyro_raw[i] == INT16_MIN)) {
            return false;
        }
    }

    for (int i = 0; i < 3; ++i) {
        out->accel[i] = ((float)(accel_raw[i]) / ACCEL_LSB_PER_G) * GRAVITY_MPS2;
        out->gyro[i] = ((float)(gyro_raw[i]) / GYRO_LSB_PER_DPS) * DEG_TO_RAD;
    }

    /* Preserve the previous temperature when the raw temperature is invalid. */
    if (temp_raw != INT16_MIN) {
        out->temp = (float)temp_raw / TEMP_SENSITIVITY + TEMP_OFFSET_C;
    }
    return true;
}
