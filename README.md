# fly_control_stm — STM32H743 飞控主工程

基于 **STM32H743VIH6** + **FreeRTOS（CMSIS-OS）** 的自研飞行控制器固件。采用分层架构，IMU 抽象层仿 PX4 设计，遥控/上位机/光流均通过 **MAVLink v2** 协议接入。构建系统为 **CMake + Ninja + arm-none-eabi-gcc**，工程结构由 STM32CubeMX 生成。

> 本工程为持续开发中的学习/实验性飞控，硬件为自制飞控板。配套的姿态/标定参考代码见 [gitee.com/Enhe1988/flight-commander](https://gitee.com/Enhe1988/flight-commander)。

---

## 目录

- [功能特性](#功能特性)
- [硬件平台](#硬件平台)
- [软件架构](#软件架构)
- [目录结构](#目录结构)
- [外设与引脚映射](#外设与引脚映射)
- [FreeRTOS 任务](#freertos-任务)
- [数据流](#数据流)
- [MAVLink 接收子系统](#mavlink-接收子系统)
- [IMU 抽象层](#imu-抽象层)
- [编译期配置](#编译期配置)
- [构建与烧录](#构建与烧录)
- [已知问题与待办](#已知问题与待办)

---

## 功能特性

- **IMU 抽象层**：仿 PX4 的 `ImuDriver` 基类（C++ 实现，对外暴露 C 接口），编译期选用 ICM-42688-P 或 BMI088，上层任务与具体芯片完全解耦。
- **姿态解算**：Madgwick AHRS（陀螺仪 + 加速度计融合），输出欧拉角。
- **MAVLink 双口接收**：
  - USART3 — 遥控器（ELRS/MAVLink RC_CHANNELS_OVERRIDE）+ 上位机姿态指令（SET_ATTITUDE_TARGET）。
  - USART2 — 光流 + 测距模块（OPTICAL_FLOW_RAD + DISTANCE_SENSOR）。
  - 接收链路基于 **DMA(circular) + IDLE/HT/TC 中断 + 任务通知**，DMA 缓冲直接作为零拷贝环形区。
- **串级 PID 控制**：Roll 双环（角度环 + 角速度环），Pitch/Yaw 单环；状态机切换飞行模式。
- **气压计**：SPL06 经 I2C2 + DMA 读取（高度估计，解算逻辑待接入）。
- **H743 Cache 一致性处理**：所有 DMA 缓冲集中放在 RAM_D2 的 MPU 非缓存区，规避 D-Cache 与 DMA 的数据一致性陷阱。

---

## 硬件平台

| 项目 | 规格 |
|------|------|
| MCU | STM32H743VIH6（Cortex-M7 @ 480MHz，2MB Flash，1MB RAM） |
| 主 IMU | ICM-42688-P（SPI4，中断 + DMA），WHO_AM_I=0x47 |
| 备选 IMU | BMI088（SPI1，轮询） |
| 气压计 | SPL06（I2C2 + DMA） |
| 光流模块 | PMW3901（光流）+ VL53L1X（ToF 测距），经 AT32F421 转 MAVLink v2 串口输出 |
| 遥控链路 | ELRS 接收机（MAVLink RC over UART） |
| 执行机构 | 2× 电机（差速推力，TIM3）+ 3× 舵机（推力矢量/Roll，TIM5） |

原理图与数据手册见 `飞控原理图/` 与 `传感器数据手册/` 目录。

---

## 软件架构

分四层，依赖自上而下单向：

```
┌──────────────────────────────────────────────────────────┐
│ application/   任务层(FreeRTOS task)                       │
│   INS_task            姿态解算(consumer，纯取数+AHRS)      │
│   Aircraft_ctrl_task  飞行控制状态机 + 串级PID + 执行输出  │
│   mavlink_*           MAVLink 双口接收与派发               │
├──────────────────────────────────────────────────────────┤
│ algorithm/     算法层                                      │
│   pid                            位置式/串级 PID           │
│   filter                         二阶低通滤波              │
├──────────────────────────────────────────────────────────┤
│ bsp/           板级支持/驱动层                             │
│   imu/         IMU 抽象层(C++) + ICM/BMI 驱动 + 官方SDK    │
│   spl06        气压计驱动                                  │
│   bmi08*       Bosch BMI088 官方 C 驱动                    │
├──────────────────────────────────────────────────────────┤
│ Third_party/   MAVLink C 库(c_library_v2，common dialect) │
│ Core/ Drivers/ Middlewares/   CubeMX 生成的 HAL/FreeRTOS  │
└──────────────────────────────────────────────────────────┘
```

---

## 目录结构

```
stm32_mavlink/
├── application/                飞控应用任务
│   ├── INS_task.{c,h}              姿态解算任务(IMU consumer + Madgwick)
│   ├── Aircraft_ctrl_task.{c,h}    飞行控制任务(状态机 + 串级PID + 电机/舵机)
│   ├── mavlink_port.{c,h}          通用 MAVLink 端口(DMA+IDLE 收，逐字节喂 parser)
│   ├── mavlink_rc.{c,h}            USART3 派发: 遥控器/上位机 → ELRS_Data/PC_Data_t
│   ├── mavlink_flow.{c,h}          USART2 派发: 光流/测距 → OpticalFlow_Data_t
│   └── mavlink_app.{c,h}           端口装配 + 初始化 + 解析任务 + 心跳/发送钩子
├── algorithm/                  算法
│   ├── pid.{c,h}                   PID 控制器
│   └── filter.{c,h}                二阶低通滤波
├── bsp/                        板级驱动
│   ├── imu/                        IMU 抽象层(详见下文)
│   ├── spl06.{c,h}                 SPL06 气压计(I2C+DMA)
│   ├── bmi08*.{c,h}                Bosch BMI088 官方驱动
│   └── bsp_config.h                编译期选型(IMU/放置/标定开关)
├── Third_party/mavlink_c/      MAVLink v2 C 库(common dialect)
├── Core/                       CubeMX: main / freertos / 外设初始化 / 中断
├── Drivers/                    STM32H7 HAL + CMSIS
├── Middlewares/                FreeRTOS + ARM DSP
├── cmake/                      工具链文件(gcc-arm-none-eabi.cmake)
├── 光流测距_串口数据协议说明.md  光流模块串口协议
├── CMakeLists.txt / CMakePresets.json
├── STM32H743XX_FLASH.ld        链接脚本(含 .dma_buffer 段)
└── fly_control_stm.ioc         CubeMX 工程文件
```

---

## 外设与引脚映射

| 外设 | 用途 | 引脚 | 备注 |
|------|------|------|------|
| **SPI4** | ICM-42688-P | SCK=PE12, MISO=PE13, MOSI=PE14, CS=PE11, **INT=PE15** | 15MHz, Mode0, 中断+DMA |
| **SPI1** | BMI088 | SCK=PA5, MISO=PA6, MOSI=PD7, gyroCS=PC15, accelCS=PB2 | 中断未引出 → 轮询 |
| **I2C2** | SPL06 气压计 | SCL=PB10, SDA=PB11 | DMA(DMA1_Stream0/1) |
| **USART3** | MAVLink 遥控/上位机 | TX=PD8, RX=PD9 | **460800** 8N1, DMA(Stream5/6) |
| **USART2** | MAVLink 光流/测距 | RX=PD6 (TX=PD5 未用) | 115200 8N1, DMA(Stream2, circular) |
| **TIM3** | 推力电机 ×2 | CH3=PB0, CH4=PB1 | PWM, 1000–1900 |
| **TIM5** | 舵机 ×3 | CH2=PA1, CH3=PA2, CH4=PA3 | PWM, 1100–1900(中位1500) |
| **EXTI15_10** | ICM 数据就绪中断 | PE15 | 触发 IMU 采集 |

> 控制映射：`TIM5 CH2/CH3` = 左/右舵机(由 Pitch+Yaw 串级输出驱动)，`TIM5 CH4` = Roll 舵机；`TIM3 CH3/CH4` = 两路推力电机(油门)。

---

## FreeRTOS 任务

| 任务 | 函数 | 优先级 | 栈 | 周期 | 职责 |
|------|------|--------|----|------|------|
| INS | `INS_task` | High | 256 | 1ms | IMU 取数 + Madgwick 姿态解算 + 气压计(50Hz 分频) |
| FLY_CTRL | `Aircraft_ctrl_task` | Normal | 256 | 1ms | 模式状态机 + 串级 PID + 电机/舵机输出 |
| MAV_LINK | `mavlink_task` | AboveNormal | 256 | 事件驱动 | 等待 UART RX 通知并解析；超时唤醒维持 2Hz 测试发送 |
| defaultTask | `StartDefaultTask` | Normal | 128 | 1ms | 空闲占位 |

任务创建见 `Core/Src/freertos.c`。MAV_LINK 任务受 `CURRENT_RC_PROTOCOL == RC_PROTOCOL_MAVLINK` 编译期开关控制。

---

## 数据流

```
ICM-42688-P ──SPI4 中断+DMA──► imu_get_data() ─┐
                                                ├─► INS_task ─Madgwick─► INS_angle ─┐
气压计 SPL06 ──I2C2+DMA──────► (高度，待接入) ──┘                                  │
                                                                                    ▼
遥控器/上位机 ─USART3─MAVLink─► 完整帧快照（RC / PC）───────────────────────────► Aircraft_ctrl_task
                                                                                    │  (状态机 + 串级PID)
光流/测距 ────USART2─MAVLink─► get_optical_flow_point() (待接入控制)               ▼
                                                                          TIM3 电机 / TIM5 舵机
```

IMU 与光流继续通过只读指针接口取最新数据；遥控和上位机指令按完整快照发布，飞控任务每轮原子复制一份本地数据，避免读到一帧更新到一半的混合值。

---

## MAVLink 接收子系统

采用 **DMA(circular) + IDLE/HT/TC 中断 + FreeRTOS 任务通知**。核心抽象 `MavlinkPort_t` 把“一条物理串口流”封装为一个端口实例，每个端口使用独立的 `mavlink_channel_t`，保证解析状态机互不污染。

| 端口 | 串口 | channel | 消息 | 产出 |
|------|------|---------|------|------|
| `g_rc_port` | USART3 | COMM_0 | RC_CHANNELS_OVERRIDE / SET_ATTITUDE_TARGET | `ELRS_Data` / `PC_Data_t` |
| `g_flow_port` | USART2 | COMM_1 | OPTICAL_FLOW_RAD(106) / DISTANCE_SENSOR(132) | `OpticalFlow_Data_t` |

**接收原理**：DMA 硬件持续循环写入非缓存缓冲区。IDLE 用于短帧或帧间空闲时的低延迟唤醒，HT/TC 保证连续字节流即使没有空闲也能定期唤醒；TC 同时累计 DMA 回绕次数。中断只记录事件并通知任务，任务根据 `回绕次数 × 缓冲大小 + (缓冲大小 - NDTR)` 算出累计生产量，再从软件读指针 `tail` 排空新字节并逐个喂给 `mavlink_parse_char`。若任务落后超过一整圈，会统计覆盖字节并重置残帧解析状态。

USART3 缓冲为 1024B，USART2 缓冲为 256B，均位于 RAM_D2 的 `.dma_buffer` 非缓存区。遥控帧发布时携带 `update_ms` 和 `frame_count`；超过 `RC_LINK_TIMEOUT_MS`（当前 100ms）没有新遥控帧，飞控状态机会进入 `Aircraft_ZERO_FORCE`。周期发送不再依赖解析循环次数，而是使用 500ms 的绝对截止时间，因此 RX 通知频率不会改变 2Hz 发送节拍。

光流模块协议（来自 AT32F421，50Hz 成对发送）详见 [`光流测距_串口数据协议说明.md`](光流测距_串口数据协议说明.md)。

---

## IMU 抽象层

位于 `bsp/imu/`，仿 PX4 设计：

| 文件 | 说明 |
|------|------|
| `imu_api.{h,cpp}` | 对外 C 接口：`imu_init` / `imu_start` / `imu_get_data`，统一数据 `ImuData`（rad/s, m/s²）+ 编译期选型 |
| `imu.hpp` | `ImuDriver` 抽象基类 |
| `Icm42688p.{hpp,cpp}` | ICM-42688-P 自研最小驱动（阻塞初始化；DRDY→SPI4 DMA→解析任务） |
| `icm42688p_regs.h` | 本工程使用的 ICM-42688-P 寄存器和配置位 |
| `Bmi088.{hpp,cpp}` | BMI088 驱动（SPI1 轮询，封装 Bosch C 驱动） |
| `icm_platform.{c,h}` | 平台服务：DWT 微秒计时 + MPU 非缓存区配置 |
| `imu_frame.{c,h}` / `imu_orientation.h` | 传感器系 → 机体系旋转映射（机体系：X前/Y右/Z下） |
| `imu_calib.{c,h}` | 零偏标定（上电实测或写死值） |
| `imu_selftest.{cpp,h}` | 双芯片通信自检（POST） |

切换 IMU 只需改 `bsp/bsp_config.h` 一行 `#define`，上层无感知。当前接口与配置以 `bsp/imu/` 的实现为准。

ICM 运行期链路为：`PE15 DRDY ISR → 启动 15 字节 SPI4 TxRx DMA → DMA 完成 ISR → 一次任务通知 → 解析/换算/标定 → 双缓冲发布`。DRDY ISR 不再先唤醒采集任务，也不在中断中做浮点运算。

---

## 编译期配置

集中在 `bsp/bsp_config.h`：

```c
#define IMU_USE_ICM42688      // IMU 选型: ICM-42688-P / BMI088 二选一
//#define IMU_USE_BMI088

#define BOARD_ORIENT_FLAT     // 板子放置: 平放 / 竖放
//#define BOARD_ORIENT_VERTICAL

#define IMU_CALIB_ON_BOOT     // 上电实测零偏(需静止水平~1s); 注释则用写死值
#define IMU_CALIB_SAMPLES 500U
```

其他：
- `Core/Inc/main.h` — `CURRENT_RC_PROTOCOL`（MAVLink / CRSF）。
- `application/mavlink_app.h` — `MAVLINK_FLOW_PORT_ENABLE`（光流口开关）。

---

## 构建与烧录

工具链由 STM32 VS Code 扩展捆绑（cmake / ninja 不在系统 PATH）。命令行构建（PowerShell）：

```powershell
$env:Path = "C:\Users\<user>\AppData\Local\stm32cube\bundles\cmake\<ver>\bin;" +
            "C:\Users\<user>\AppData\Local\stm32cube\bundles\ninja\<ver>\bin;" + $env:Path

cmake --preset Debug          # 改了 CMakeLists / 加了源文件后需重跑
cmake --build build/Debug      # 产物: build/Debug/fly_control_stm.elf
```

- 推荐使用 **STM32 VS Code 扩展（cube-cmake）** 一键构建/调试/烧录。
- 也可用 STM32CubeIDE 导入 `.ioc`。
- C++ 工具链配置见 `cmake/gcc-arm-none-eabi.cmake`（C++17，`-fno-rtti / -fno-exceptions / -fno-threadsafe-statics`）。

---

## 已知问题与待办

- **IMU 轴映射需随硬件复核**：ICM 与 BMI088 的当前映射集中在 `bsp/imu/imu_frame.c`，换板或改安装方向后需重新验证。
- **光流数据未接入控制**：`get_optical_flow_point()` 已可取到 `range_cm` / `flow_x_rad` 等，尚未接入 `Aircraft_ctrl_task` 做定高/水平速度控制。
- **气压计高度未接入**：SPL06 已读取，解算高度尚未写入 INS 数据结构供控制使用。
- **加速度零偏/温补、IMU 温度读取**等为可选优化项（BMI088 温度暂填 0）。
- MAVLink 发送钩子目前统一走 USART3，未按 channel 区分。

---

## 协议与依赖

- **MAVLink**：c_library_v2（common dialect），见 `Third_party/mavlink_c/`。
- **FreeRTOS** + **CMSIS-RTOS v1**。
- **CMSIS-DSP**（ARM Math）。
- **Bosch BMI08x 驱动**；ICM-42688-P 由本工程按数据手册直接驱动。

---

*本工程仍在持续开发中，欢迎参考学习。如用于实飞，请充分测试并自负风险。*
