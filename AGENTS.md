# 项目：fly_control_stm（stm32_mavlink 主工程）

STM32H743VIH6 飞控，480MHz，FreeRTOS/CMSIS-OS v1。构建使用 CMake + Ninja + arm-none-eabi-gcc（C11 + 启动汇编），工具链由 STM32 VS Code 扩展提供。

## 分层与接口
- application：INS 姿态任务、飞行控制状态机、MAVLink 双串口接收与派发。
- algorithm：PID 与二阶低通滤波；当前使用的 Madgwick 实现在 application/INS_task.c。
- bsp/imu：纯 C 驱动；imu_api.c 负责探测、选择、统一处理与快照发布。上层使用 imu_init/imu_start/imu_read，imu_get_data 保留兼容。
- IMU 统一输出 rad/s、m/s²；INS_angle 顺序为 yaw/pitch/roll，单位为度。
- IMU 在启动时按 ICM-45686 > ICM-42688-P > BMI088 自动探测选择；bsp/bsp_config.h 的 AIRCRAFT_MODEL 统一选择样机、安装方向与混控，标定开关也在该文件。

## IMU 架构约束
- 各驱动自管采集节拍：ICM 使用 DRDY 驱动的 DMA 链路，BMI088 使用独立轮询任务。
- ICM 使用本工程最小驱动：初始化阶段阻塞读写寄存器；运行时 DRDY ISR 直接启动传感器寄存器突发 DMA，DMA 完成 ISR 通知解析任务。ISR 不做浮点解析。
- icm_acq.c 统一管理两款 ICM 的 SPI4 传输、DMA 双缓冲与解析任务；icm42688p.c / icm45686.c 只保留各自探测、配置、DRDY 使能与原始数据解码。
- 三款芯片通过 imu_publish 统一做轴映射、零偏与放置旋转。INS 与标定使用 imu_read 复制完整快照，不持有跨采样周期的数据指针。
- 数据转换顺序：物理量换算 → imu_frame 轴映射 → 板子系零偏扣除 → imu_orientation 放置旋转。
- INS 通过统一接口取数，不增加按芯片分支。
- SPI/UART DMA 缓冲置于 RAM_D2 的 .dma_buffer 段；MPU region1 覆盖 0x30000000 起始 2KB，新增缓冲需检查容量。
- bsp/bsp_platform.c 提供 DWT 微秒计时与 MPU 配置，在 main 的 USER CODE SysInit 中初始化，先于任何 DMA 和任务启动。main 在 MPU 配置后开启 DCache，再初始化外设/任务；ICache 未主动开启。
- 调用 FreeRTOS FromISR API 的中断使用逻辑优先级 5 或更低紧急度。

## MAVLink 架构约束
- USART3（遥控/上位机）与 USART2（光流/测距）使用 circular RX DMA；IDLE、HT、TC 回调只记录事件并通知 MAVLink 任务，不在 ISR 中解析协议。
- MAVLink 任务根据 DMA TC 回绕次数和 NDTR 计算累计生产量，从 DMA 缓冲的 `tail` 排空新字节；发生一整圈覆盖时统计丢字节并重置残帧解析状态。
- USART3 与 USART2 分别使用 MAVLINK_COMM_0 / MAVLINK_COMM_1，不能让两路物理字节流共用同一个 MAVLink parser channel。
- 遥控和上位机指令在整帧解码完成后按完整快照原子发布；飞控任务复制本地快照使用，不直接读取正在更新的共享结构。
- 遥控快照超过 `RC_LINK_TIMEOUT_MS`（当前 100ms）未更新时，飞控状态机必须进入 `Aircraft_ZERO_FORCE`。
- MAVLink 周期发送由任务通知等待的超时时间和绝对截止时间驱动，不使用解析循环次数充当发送分频器。

## 硬件
- ICM-45686 / ICM-42688-P：共用 SPI4 与贴片位置，PE12/PE13/PE14，CS=PE11，INT=PE15；WHO_AM_I 分别为 0xE9 / 0x47。
- BMI088：SPI1，PA5/PA6/PD7，两路 CS=PC15/PB2；中断未引出。片选角色以实际驱动与板上验证为准。
- USART3：460800，遥控与上位机；USART2：115200，光流与测距。
- TIM3 CH3/CH4：电机；TIM5 CH2/CH3/CH4：舵机。
- 样机 A：竖放，TIM3 CH3/CH4 左右电机差速 Roll，TIM5 CH2/CH3 双舵机混控 Pitch/Yaw（左=Pitch+Yaw，右=-Pitch+Yaw），不使用 TIM5 CH4。
- 样机 B：竖放；C：平放。B/C 均使用 TIM3 CH3 单电机和 TIM5 CH2/CH3/CH4 独立 Yaw/Pitch/Roll 舵机，不使用 TIM3 CH4。
- algorithm/aircraft_mixer.c 按编译期构型混控；遥控和上位机模式共用。throttle 为 0..1，三轴输出为 us 增量。方向、限幅和中位在 bsp_config.h；A 的差速受对称油门余量限制。
- AIRCRAFT_PWM_ENABLE 默认 0；开启后由飞控任务先写停机/中位再启动所选通道，main 不再单独启动 PWM。
- SPL06：I2C2，当前读取使用 HAL_I2C_Mem_Read_IT，函数名仍保留 DMA 字样。

## 修改注意
- 尊重工作区已有未提交修改；当前 PWM 启动、心跳等调试开关以实际源码为准。
- BMI088 为受支持的备选实现，不因当前选用 ICM 而删除。
- 光流/测距与气压高度尚未接入控制闭环；三轴角度/角速度串级 PID 已接入，单位依次为 deg、deg/s、us。
- CubeMX 生成代码的手工修改放 USER CODE 区；保留 .ioc、生成配置和厂商依赖。
- README 与历史注释可能滞后，应核对真实调用链。
- 新任务先以本文件确定长期约束，再阅读 README 的总体数据流和目标模块源码；实现细节冲突时以当前源码、CubeMX 配置和链接结果为准。
- IMU 主机回归测试：python tests/imu/run_tests.py，需要本机 GCC；HAL/RTOS 与 BMI088 适配层使用模拟，不能替代传感器实板验证。
- 样机混控/安装旋转测试：python tests/control/run_tests.py，覆盖 A/B/C 与非法配置。

## 关联资料
- 光流测距_串口数据协议说明.md：光流模块协议。
- 传感器数据手册/：ICM 规格书。
- 姊妹工程 E:/2024/project/stm_fly_control/icm42686：芯片通信验证。
- E:/2024/project/stm_fly_control/飞控原理图/：硬件原理图。

## 姿态控制
- algorithm/attitude_control.c：三轴串级 PID，遥控目标变化率前馈叠加到外环角速度目标；配置在 bsp_config.h 的 CONTROL_*。
- 前馈只在新遥控帧更新，按实际帧间隔计算；首帧/模式切换抑制，带限幅和一阶平滑。上位机模式不启用遥控前馈。
- INS/飞控任务用 vTaskDelayUntil 以 1ms 调度，PID 与 Madgwick 用 DWT 实测 dt；FreeRTOSConfig.h 的 USER CODE Defines 覆盖启用 INCLUDE_vTaskDelayUntil。
- INS 用 ins_get_snapshot 发布完整姿态/角速度及有效标志；初始化/标定前或 IMU 超时不控制，上位机模式检查指令有效期。
- 参数在 bsp_config.h 的 MODEL_A/B/C_ANGLE_*_PID 与 MODEL_A/B/C_RATE_*_PID 分组，CONTROL_* 为选型别名；三组初值相同且 Ki/Kd 默认零，不能描述为分别完成实机整定。
- 角速度内环反馈已接入三轴独立二阶 Butterworth（filter.c），fs 为名义控制频率；未改变 Madgwick 输入。前馈的一阶平滑与这个二阶滤波分别保存状态。
- 动态截止频率按基础油门在当前样机的 MODEL_*_GYRO_LPF_MIN/MAX_HZ 间线性调度，默认 30–80Hz，10ms 更新/50ms 平滑；更换系数保留历史，不是 FFT/振动频谱自适应。
- CONTROL_GYRO_LPF_ENABLE 与 CONTROL_GYRO_LPF_DYNAMIC_ENABLE 分别控制旁路和固定带宽模式；control 主机测试包含频响和这些分支。
