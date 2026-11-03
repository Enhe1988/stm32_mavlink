#ifndef TEST_TASK_H
#define TEST_TASK_H
#include "FreeRTOS.h"
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
BaseType_t xTaskCreate(void (*entry)(void *), const char *, unsigned, void *, unsigned, TaskHandle_t *);
uint32_t ulTaskNotifyTake(BaseType_t, uint32_t);
void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t *);
#endif
