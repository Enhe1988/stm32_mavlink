/**
 * @file    icm45686_regs.h
 * @brief   ICM-45686 本工程使用到的最小寄存器与位定义。
 *
 * 地址和位定义来自 ICM-45686 datasheet DS-000489 rev 1.1。
 */
#ifndef ICM45686_REGS_H
#define ICM45686_REGS_H

#include <stdint.h>

namespace icm45686_reg {

constexpr uint8_t READ_BIT = 0x80U;

/* User bank 0 direct registers. */
constexpr uint8_t ACCEL_DATA_X1   = 0x00U;
constexpr uint8_t PWR_MGMT0       = 0x10U;
constexpr uint8_t INT1_CONFIG0    = 0x16U;
constexpr uint8_t INT1_CONFIG1    = 0x17U;
constexpr uint8_t INT1_CONFIG2    = 0x18U;
constexpr uint8_t INT1_STATUS0    = 0x19U;
constexpr uint8_t ACCEL_CONFIG0   = 0x1BU;
constexpr uint8_t GYRO_CONFIG0    = 0x1CU;
constexpr uint8_t FIFO_CONFIG0    = 0x1DU;
constexpr uint8_t INTF_CONFIG1_OVRD = 0x2DU;
constexpr uint8_t WHO_AM_I        = 0x72U;
constexpr uint8_t IREG_ADDR_15_8  = 0x7CU;
constexpr uint8_t IREG_ADDR_7_0   = 0x7DU;
constexpr uint8_t IREG_DATA       = 0x7EU;
constexpr uint8_t REG_MISC2       = 0x7FU;

constexpr uint8_t WHO_AM_I_VALUE = 0xE9U;

constexpr uint8_t REG_MISC2_SOFT_RESET = 0x02U;
constexpr uint8_t REG_MISC2_IREG_DONE  = 0x01U;
constexpr uint8_t INT1_STATUS0_RESET_DONE = 0x80U;

/* AP host interface: force 4-wire SPI, mode 0/3. */
constexpr uint8_t INTF_CONFIG1_OVRD_4WIRE_MODE03 = 0x0EU;

/* INT1: enable only UI DRDY; push-pull, pulse, active high. */
constexpr uint8_t INT1_CONFIG0_DRDY = 0x04U;
constexpr uint8_t INT1_CONFIG2_PUSH_PULL_PULSE_HIGH = 0x01U;

/* UI output: +/-16 g, +/-2000 dps, both at 800 Hz (ICM-45686 has no 1 kHz ODR). */
constexpr uint8_t ACCEL_CONFIG0_16G_800HZ = 0x16U;
constexpr uint8_t GYRO_CONFIG0_2000DPS_800HZ = 0x16U;
constexpr uint8_t PWR_MGMT0_ACCEL_GYRO_LN = 0x0FU;

/* IPREG_TOP1:SREG_CTRL (0xA200 + 0x67), bit1 selects big-endian sensor data. */
constexpr uint16_t IREG_SREG_CTRL = 0xA267U;
constexpr uint8_t SREG_CTRL_BIG_ENDIAN = 0x02U;

/* ACCEL XYZ (6) + GYRO XYZ (6) + TEMP (2), starting at 0x00. */
constexpr uint8_t SENSOR_DATA_BYTES = 14U;
constexpr uint8_t SPI_FRAME_BYTES = SENSOR_DATA_BYTES + 1U;

} /* namespace icm45686_reg */

#endif /* ICM45686_REGS_H */
