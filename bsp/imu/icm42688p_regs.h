/**
 * @file    icm42688p_regs.h
 * @brief   ICM-42688-P 本工程实际使用的寄存器与位定义。
 *
 * 只保留初始化、INT1 DATA_RDY 和 14 字节传感器突发读取所需内容。
 * 地址与位定义来自 ICM-42688-P datasheet rev 1.2。
 */
#ifndef ICM42688P_REGS_H
#define ICM42688P_REGS_H

#include <stdint.h>

namespace icm42688p_reg {

constexpr uint8_t READ_BIT = 0x80U;

/* Bank 0 */
constexpr uint8_t DEVICE_CONFIG      = 0x11U;
constexpr uint8_t DRIVE_CONFIG       = 0x13U;
constexpr uint8_t INT_CONFIG         = 0x14U;
constexpr uint8_t FIFO_CONFIG        = 0x16U;
constexpr uint8_t TEMP_DATA1         = 0x1DU;
constexpr uint8_t INT_STATUS         = 0x2DU;
constexpr uint8_t INTF_CONFIG0       = 0x4CU;
constexpr uint8_t PWR_MGMT0          = 0x4EU;
constexpr uint8_t GYRO_CONFIG0       = 0x4FU;
constexpr uint8_t ACCEL_CONFIG0      = 0x50U;
constexpr uint8_t GYRO_ACCEL_CONFIG0 = 0x52U;
constexpr uint8_t INT_CONFIG0        = 0x63U;
constexpr uint8_t INT_CONFIG1        = 0x64U;
constexpr uint8_t INT_SOURCE0        = 0x65U;
constexpr uint8_t WHO_AM_I           = 0x75U;
constexpr uint8_t REG_BANK_SEL       = 0x76U;

/* Bank 1 */
constexpr uint8_t INTF_CONFIG4       = 0x7AU;
constexpr uint8_t INTF_CONFIG6       = 0x7CU;

constexpr uint8_t BANK0 = 0U;
constexpr uint8_t BANK1 = 1U;

constexpr uint8_t DEVICE_CONFIG_SOFT_RESET = 0x01U;
constexpr uint8_t INT_STATUS_RESET_DONE    = 0x10U;
constexpr uint8_t WHO_AM_I_VALUE           = 0x47U;

/* SPI 4-wire；INT1 为高有效、推挽、脉冲；仅路由 UI DATA_RDY。 */
constexpr uint8_t DRIVE_CONFIG_SPI_SLEW_5 = 0x05U;
constexpr uint8_t INT_CONFIG_INT1         = 0x03U;
constexpr uint8_t INT_SOURCE0_UI_DRDY     = 0x08U;

/* 大端传感器数据，关闭 I2C 从接口；复位值中的大端位保持为 1。 */
constexpr uint8_t INTF_CONFIG0_SPI_ONLY = 0x33U;

constexpr uint8_t INTF_CONFIG4_I3C_BUS_MODE = 0x40U;
constexpr uint8_t INTF_CONFIG4_SPI_4WIRE     = 0x02U;
constexpr uint8_t INTF_CONFIG6_I3C_EN        = 0x10U;
constexpr uint8_t INTF_CONFIG6_I3C_DDR_EN    = 0x02U;
constexpr uint8_t INTF_CONFIG6_I3C_SDR_EN    = 0x01U;

/* ±2000 dps / ±16 g，均为 1 kHz；UI 滤波器带宽使用复位默认值。 */
constexpr uint8_t GYRO_CONFIG0_2000DPS_1KHZ  = 0x06U;
constexpr uint8_t ACCEL_CONFIG0_16G_1KHZ     = 0x06U;
constexpr uint8_t GYRO_ACCEL_CONFIG0_DEFAULT = 0x11U;

constexpr uint8_t PWR_MGMT0_ACCEL_GYRO_LN = 0x0FU;

constexpr uint8_t SENSOR_DATA_BYTES = 14U;
constexpr uint8_t SPI_FRAME_BYTES   = SENSOR_DATA_BYTES + 1U;

} /* namespace icm42688p_reg */

#endif /* ICM42688P_REGS_H */
