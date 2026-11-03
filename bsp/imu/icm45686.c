/** @file icm45686.c
 * Chip-specific blocking setup and raw sample decoding. Runtime DMA is in icm_acq.c.
 */

#include "icm45686.h"
#include "icm45686_regs.h"
#include "icm_acq.h"
#include "bsp_platform.h"
#include <limits.h>

#define RESET_SETTLE_US (2000U)
#define GYRO_STARTUP_US (50000U)
#define RESET_POLL_US (100U)
#define RESET_POLL_COUNT (100U)
#define IREG_WAIT_US (5U)
#define ACCEL_LSB_PER_G (2048.0f)
#define GYRO_LSB_PER_DPS (16.4f)
#define GRAVITY_MPS2 (9.80665f)
#define DEG_TO_RAD (0.017453292519943295f)
#define TEMP_SENSITIVITY (128.0f)
#define TEMP_OFFSET_C (25.0f)

static uint8_t s_who_am_i;
static int s_init_rc;

static bool write_indirect(uint16_t address, uint8_t value)
{
    const uint8_t request[3] = {
        (uint8_t)(address >> 8U),
        (uint8_t)(address & 0xFFU),
        value,
    };
    if (!icm_write_registers(ICM45686_IREG_ADDR_15_8, request, sizeof(request))) {
        return false;
    }

    for (uint32_t i = 0U; i < 20U; ++i) {
        bsp_delay_us(IREG_WAIT_US);
        uint8_t status = 0U;
        if (!icm_read_registers(ICM45686_REG_MISC2, &status, 1U)) {
            return false;
        }
        if ((status & ICM45686_REG_MISC2_IREG_DONE) != 0U) {
            return true;
        }
    }
    return false;
}

static bool read_indirect(uint16_t address, uint8_t *value)
{
    if (value == NULL) {
        return false;
    }

    const uint8_t request[2] = {
        (uint8_t)(address >> 8U),
        (uint8_t)(address & 0xFFU),
    };
    if (!icm_write_registers(ICM45686_IREG_ADDR_15_8, request, sizeof(request))) {
        return false;
    }

    bsp_delay_us(IREG_WAIT_US);
    return icm_read_registers(ICM45686_IREG_DATA, value, 1U);
}

bool icm45686_probe(void)
{
    icm_cs_high();
    s_who_am_i = 0U;
    return icm_read_registers(ICM45686_WHO_AM_I, &s_who_am_i, 1U) &&
           (s_who_am_i == ICM45686_WHO_AM_I_VALUE);
}

static bool configure(void)
{
    if (!icm_write_register(ICM45686_REG_MISC2, ICM45686_REG_MISC2_SOFT_RESET)) {
        s_init_rc = INIT_SPI_ERROR;
        return false;
    }

    bsp_delay_us(RESET_SETTLE_US);

    bool reset_done = false;
    for (uint32_t i = 0U; i < RESET_POLL_COUNT; ++i) {
        uint8_t status = 0U;
        if (!icm_read_registers(ICM45686_INT1_STATUS0, &status, 1U)) {
            s_init_rc = INIT_SPI_ERROR;
            return false;
        }
        if ((status & ICM45686_INT1_STATUS0_RESET_DONE) != 0U) {
            reset_done = true;
            break;
        }
        bsp_delay_us(RESET_POLL_US);
    }
    if (!reset_done) {
        s_init_rc = INIT_RESET_TIMEOUT;
        return false;
    }

    if (!icm45686_probe()) {
        s_init_rc = (s_who_am_i == 0U) ? INIT_SPI_ERROR : INIT_WHO_AM_I_ERROR;
        return false;
    }

    /* 明确锁定当前板级连接需要的 4-wire SPI mode 0。 */
    if (!icm_write_register(ICM45686_INTF_CONFIG1_OVRD, ICM45686_INTF_CONFIG1_OVRD_4WIRE_MODE03) ||
        !icm_verify_register(ICM45686_INTF_CONFIG1_OVRD, ICM45686_INTF_CONFIG1_OVRD_4WIRE_MODE03, 0x0FU)) {
        s_init_rc = INIT_CONFIG_ERROR;
        return false;
    }

    /* 数据手册规定传感器寄存器默认小端；统一切到大端后再做连续 DMA 读取。 */
    if (!write_indirect(ICM45686_IREG_SREG_CTRL, ICM45686_SREG_CTRL_BIG_ENDIAN)) {
        s_init_rc = INIT_SPI_ERROR;
        return false;
    }
    uint8_t sreg_ctrl = 0U;
    if (!read_indirect(ICM45686_IREG_SREG_CTRL, &sreg_ctrl) ||
        ((sreg_ctrl & ICM45686_SREG_CTRL_BIG_ENDIAN) == 0U)) {
        s_init_rc = INIT_CONFIG_ERROR;
        return false;
    }

    /* 传感器关闭时配置量程、ODR、FIFO 和 INT1；icm_acq_start() 后才开放 DRDY。 */
    const struct {
        uint8_t reg;
        uint8_t value;
    } config[] = {
        {ICM45686_INT1_CONFIG0,  0x00U},
        {ICM45686_INT1_CONFIG1,  0x00U},
        {ICM45686_INT1_CONFIG2,  ICM45686_INT1_CONFIG2_PUSH_PULL_PULSE_HIGH},
        {ICM45686_FIFO_CONFIG0,  0x00U},
        {ICM45686_ACCEL_CONFIG0, ICM45686_ACCEL_CONFIG0_16G_800HZ},
        {ICM45686_GYRO_CONFIG0,  ICM45686_GYRO_CONFIG0_2000DPS_800HZ},
    };

    for (unsigned i = 0U; i < sizeof(config) / sizeof(config[0]); ++i) {
        if (!icm_write_register(config[i].reg, config[i].value)) {
            s_init_rc = INIT_SPI_ERROR;
            return false;
        }
    }

    if (!icm_write_register(ICM45686_PWR_MGMT0, ICM45686_PWR_MGMT0_ACCEL_GYRO_LN)) {
        s_init_rc = INIT_SPI_ERROR;
        return false;
    }

    /* 手册给出 gyro 启动时间 35 ms；留到 50 ms 后再检查配置。 */
    bsp_delay_us(GYRO_STARTUP_US);

    if (!icm_verify_register(ICM45686_WHO_AM_I, ICM45686_WHO_AM_I_VALUE, 0xFFU) ||
        !icm_verify_register(ICM45686_INT1_CONFIG0, 0x00U, 0xFFU) ||
        !icm_verify_register(ICM45686_INT1_CONFIG2, ICM45686_INT1_CONFIG2_PUSH_PULL_PULSE_HIGH, 0x07U) ||
        !icm_verify_register(ICM45686_ACCEL_CONFIG0, ICM45686_ACCEL_CONFIG0_16G_800HZ, 0x7FU) ||
        !icm_verify_register(ICM45686_GYRO_CONFIG0, ICM45686_GYRO_CONFIG0_2000DPS_800HZ, 0xFFU) ||
        !icm_verify_register(ICM45686_PWR_MGMT0, ICM45686_PWR_MGMT0_ACCEL_GYRO_LN, 0x0FU)) {
        s_init_rc = INIT_CONFIG_ERROR;
        return false;
    }

    s_init_rc = INIT_OK;
    return true;
}

bool icm45686_init(void)
{
    s_init_rc = INIT_OK;
    icm_cs_high();
    bsp_delay_us(3000U);
    return configure();
}

bool icm45686_enable_drdy(void)
{
    uint8_t ignored;
    (void)icm_read_registers(ICM45686_INT1_STATUS0, &ignored, 1U);
    return icm_write_register(ICM45686_INT1_CONFIG0, ICM45686_INT1_CONFIG0_DRDY);
}

uint8_t icm45686_who_am_i(void) { return s_who_am_i; }
int icm45686_init_result(void) { return s_init_rc; }

bool icm45686_decode(const uint8_t *sample, ImuData *out)
{
    const int16_t accel_raw[3] = {
        icm_decode_be16(&sample[0]),
        icm_decode_be16(&sample[2]),
        icm_decode_be16(&sample[4]),
    };
    const int16_t gyro_raw[3] = {
        icm_decode_be16(&sample[6]),
        icm_decode_be16(&sample[8]),
        icm_decode_be16(&sample[10]),
    };
    const int16_t temp_raw = icm_decode_be16(&sample[12]);

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
