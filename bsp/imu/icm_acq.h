/** Shared SPI4 transport and DRDY/DMA acquisition for the selected ICM. */
#ifndef ICM_ACQ_H
#define ICM_ACQ_H
#include "imu_api.h"
#include <stddef.h>

enum {
    INIT_OK = 0,
    INIT_SPI_ERROR = -1,
    INIT_RESET_TIMEOUT = -2,
    INIT_WHO_AM_I_ERROR = -3,
    INIT_CONFIG_ERROR = -4,
    INIT_TASK_ERROR = -5,
};

void icm_cs_high(void);
bool icm_read_registers(uint8_t reg, uint8_t *data, uint16_t length);
bool icm_write_registers(uint8_t reg, const uint8_t *data, uint16_t length);
bool icm_write_register(uint8_t reg, uint8_t value);
bool icm_verify_register(uint8_t reg, uint8_t expected, uint8_t mask);
int icm_acq_start(imu_type_t type);
void icm_acq_drdy_isr(void);
void icm_acq_done_isr(void);
void icm_acq_error_isr(void);

static inline int16_t icm_decode_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8U) | p[1]);
}
#endif
