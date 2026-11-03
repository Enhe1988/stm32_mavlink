#ifndef TELEMETRY_TEST_MAIN_H
#define TELEMETRY_TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
uint32_t HAL_GetTick(void);
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef enum { HAL_UART_RXEVENT_TC, HAL_UART_RXEVENT_HT, HAL_UART_RXEVENT_IDLE } HAL_UART_RxEventTypeTypeDef;
typedef struct { uint16_t remaining; } DMA_HandleTypeDef;
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
typedef struct {
    DMA_HandleTypeDef *hdmarx;
    unsigned gState;
} UART_HandleTypeDef;
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_TX 1U
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
#endif
