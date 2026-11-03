/**
  ********************************************************
  * @file       mavlink_app.h
  * @brief      MAVLink 应用层装配:创建各串口端口实例、初始化、解析任务。
  *             - USART3 / COMM_0:遥控+上位机(mavlink_rc)
  *             - USART2 / COMM_1:光流+测距(mavlink_flow),需 CubeMX 生成 huart2
  ********************************************************
  */
#ifndef __MAVLINK_APP_H__
#define __MAVLINK_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 光流端口(USART2)开关:CubeMX 生成 huart2 + DMA(circular) 后改为 1。
 * 置 0 时整套光流接收不参与编译,工程可在 USART2 就绪前正常构建。 */
#ifndef MAVLINK_FLOW_PORT_ENABLE
#define MAVLINK_FLOW_PORT_ENABLE 1
#endif

/**
  * @brief  初始化所有 MAVLink 端口并启动 DMA+IDLE/HT/TC 接收。
  * @note   由 mavlink_task 在调度器启动、任务句柄就绪后调用。
  */
void Mavlink_Init(void);

/**
  * @brief  MAVLink 解析任务:等待 RX/错误通知，排空对应端口，并维持 20Hz 遥测 DMA。
  */
void mavlink_task(void const *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* __MAVLINK_APP_H__ */
