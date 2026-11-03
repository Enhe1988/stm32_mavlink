#ifndef TEST_SPI_H
#define TEST_SPI_H
#include "main.h"
typedef struct { void *Instance; } SPI_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define SPI4 ((void *)4)
extern SPI_HandleTypeDef hspi4;
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t);
#endif
