#ifndef TELEMETRY_TEST_TASK_H
#define TELEMETRY_TEST_TASK_H
#include "FreeRTOS.h"
#include <assert.h>
typedef uint32_t TickType_t;
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define configASSERT(value) assert(value)
#define pdMS_TO_TICKS(value) (value)
#define eSetBits 0
void vTaskDelay(TickType_t);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
TickType_t xTaskGetTickCount(void);
BaseType_t xTaskNotifyFromISR(TaskHandle_t, uint32_t, int, BaseType_t *);
BaseType_t xTaskNotifyWait(uint32_t, uint32_t, uint32_t *, TickType_t);
#endif
