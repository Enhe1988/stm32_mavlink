#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
#define ICM_CS_GPIO_Port ((void *)1)
#define ICM_CS_Pin 1U
#define ICM_INT_Pin 2U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define __DMB() ((void)0)
#define __HAL_GPIO_EXTI_CLEAR_IT(pin) ((void)(pin))
void HAL_GPIO_WritePin(void *port, uint16_t pin, unsigned state);
#endif
