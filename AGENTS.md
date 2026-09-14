# 项目：fly_control_stm（stm32_mavlink 主工程）

STM32H743VIH6 飞控，480MHz，FreeRTOS/CMSIS-OS v1。构建使用 CMake + Ninja + arm-none-eabi-gcc（C11/C++17），工具链由 STM32 VS Code 扩展提供。

## 分层与接口
- application：INS 姿态任务、飞行控制状态机、MAVLink 双串口接收与派发。
- algorithm：PID 与二阶低通滤波；当前使用的 Madgwick 实现在 application/INS_task.c。
- bsp/imu：C++ ImuDriver 抽象，对外暴露 extern "C" 的 imu_init/imu_start/imu_get_data。
- IMU 统一输出 rad/s、m/s²；INS_angle 顺序为 yaw/pitch/roll，单位为度。
- 编译期 IMU 选型、平/竖放和上电标定开关集中于 bsp/bsp_config.h。

## IMU 架构约束
- 各驱动自管采集节拍：ICM 使用 DRDY 驱动的 DMA 链路，BMI088 使用独立轮询任务。
- ICM 使用本工程最小驱动：初始化阶段阻塞读写寄存器；运行时 DRDY ISR 直接启动传感器寄存器突发 DMA，DMA 完成 ISR 通知解析任务。ISR 不做浮点解析。
- 数据转换顺序：物理量换算 → imu_frame 轴映射 → 板子系零偏扣除 → imu_orientation 放置旋转。
- INS 通过统一接口取数，不增加按芯片分支。
- SPI/UART DMA 缓冲置于 RAM_D2 的 .dma_buffer 段；MPU region1 覆盖 0x30000000 起始 2KB，新增缓冲需检查容量。
- 调用 FreeRTOS FromISR API 的中断使用逻辑优先级 5 或更低紧急度。

## 硬件
- ICM-42688-P：SPI4，PE12/PE13/PE14，CS=PE11，INT=PE15，WHO_AM_I=0x47。
- BMI088：SPI1，PA5/PA6/PD7，两路 CS=PC15/PB2；中断未引出。片选角色以实际驱动与板上验证为准。
- USART3：460800，遥控与上位机；USART2：115200，光流与测距。
- TIM3 CH3/CH4：电机；TIM5 CH2/CH3/CH4：舵机。
- SPL06：I2C2，当前读取使用 HAL_I2C_Mem_Read_IT，函数名仍保留 DMA 字样。

## 修改注意
- 尊重工作区已有未提交修改；当前 PWM 启动、心跳等调试开关以实际源码为准。
- BMI088 为受支持的备选实现，不因当前选用 ICM 而删除。
- 光流/测距与气压高度尚未接入控制闭环；角速度 PID 当前未参与控制输出。
- CubeMX 生成代码的手工修改放 USER CODE 区；保留 .ioc、生成配置和厂商依赖。
- README 与历史注释可能滞后，应核对真实调用链。

## 关联资料
- 光流测距_串口数据协议说明.md：光流模块协议。
- 传感器数据手册/：ICM 规格书。
- 姊妹工程 E:/2024/project/stm_fly_control/icm42686：芯片通信验证。
- E:/2024/project/stm_fly_control/飞控原理图/：硬件原理图。
