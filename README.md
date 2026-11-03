# fly_control_stm — STM32H743 飞控主工程

基于 **STM32H743VIH6** + **FreeRTOS（CMSIS-OS）** 的自研飞行控制器固件。采用纯 C 分层架构，IMU 通过统一接口接入，遥控/上位机/光流均通过 **MAVLink v2** 协议接入。构建系统为 **CMake + Ninja + arm-none-eabi-gcc**（C11 + 启动汇编），工程结构由 STM32CubeMX 生成。

> 本工程为持续开发中的学习/实验性飞控，硬件为自制飞控板。配套的姿态/标定参考代码见 [gitee.com/Enhe1988/flight-commander](https://gitee.com/Enhe1988/flight-commander)。

---

## 目录

- [功能特性](#功能特性)
- [硬件平台](#硬件平台)
- [当前实现状态](#当前实现状态)
- [软件架构](#软件架构)
- [目录结构](#目录结构)
- [外设与引脚映射](#外设与引脚映射)
- [FreeRTOS 任务](#freertos-任务)
- [数据流](#数据流)
- [MAVLink 接收子系统](#mavlink-接收子系统)
- [IMU 抽象层](#imu-抽象层)
- [IMU 与编译期配置](#imu-与编译期配置)
- [构建与烧录](#构建与烧录)
- [已知问题与待办](#已知问题与待办)
- [协议与依赖](#协议与依赖)

---

## 功能特性

- **IMU 统一接口**：纯 C 实现，上电自动探测 ICM-45686、ICM-42688-P 和 BMI088；两款 ICM 共用一套 DMA 采集和解析任务，上层通过完整快照取数。
- **姿态解算**：Madgwick AHRS（陀螺仪 + 加速度计融合），输出欧拉角。
- **MAVLink 双口接收**：
  - USART3 — 遥控器（ELRS/MAVLink RC_CHANNELS_OVERRIDE）+ 上位机姿态指令（SET_ATTITUDE_TARGET）。
  - USART2 — 光流 + 测距模块（OPTICAL_FLOW_RAD + DISTANCE_SENSOR）。
  - 接收链路基于 **DMA(circular) + IDLE/HT/TC 中断 + 任务通知**，DMA 缓冲直接作为零拷贝环形区。
- **飞行控制框架**：三轴角度外环、角速度内环、遥控目标变化率前馈和样机混控已接入；PID/前馈初始参数集中在配置文件，仍需实机整定。
- **气压计**：SPL06 经 I2C2 中断方式读取（`HAL_I2C_Mem_Read_IT`），高度解算尚未接入控制闭环。
- **H743 Cache 一致性处理**：main 先配置 RAM_D2 的 MPU 非缓存区，再开启 D-Cache，然后初始化外设和任务；SPI4/UART DMA 缓冲统一在该区。当前仅验证代码和链接布局，未进行硬件一致性验证。

---

## 硬件平台

| 项目 | 规格 |
|------|------|
| MCU | STM32H743VIH6（Cortex-M7 @ 480MHz，2MB Flash，1MB RAM） |
| SPI4 IMU | ICM-45686 或 ICM-42688-P（同一贴片位置，中断 + DMA），WHO_AM_I=0xE9 / 0x47 |
| SPI1 IMU | BMI088（轮询） |
| 气压计 | SPL06（I2C2 中断读取） |
| 光流模块 | PMW3901（光流）+ VL53L1X（ToF 测距），经 AT32F421 转 MAVLink v2 串口输出 |
| 遥控链路 | ELRS 接收机（MAVLink RC over UART） |
| 执行机构 | A：2× 电机 + 2× 舵机；B/C：1× 电机 + 3× 舵机，由样机配置选择 |

原理图与数据手册见 `飞控原理图/` 与 `传感器数据手册/` 目录。

---

## 当前实现状态

| 模块 | 当前状态 | 说明 |
|------|----------|------|
| IMU | 已接入，待实板覆盖测试 | 三类 IMU 上电自动探测；ICM-45686 与 ICM-42688-P 使用 DRDY + DMA，BMI088 使用轮询任务 |
| 姿态解算 | 已接入 | `INS_task` 每 1ms 读取统一 IMU 快照并执行 Madgwick 解算 |
| 遥控与上位机 | 已接入 | USART3 MAVLink 帧解码后按完整快照发布，遥控超时进入零力状态 |
| 飞行控制 | 开发中 | 三轴串级 PID + 遥控变化率前馈进入混控；参数为初始值 |
| 光流与测距 | 实板接收已通过，提供遥测估计，未进闭环 | 下视光流估计机体 XY 速度，测距结合姿态投影为离地高度；方向、比例及倾斜高度准确性待实测 |
| 气压高度 | 已读取，未进闭环 | SPL06 原始数据和高度计算链路已建立，尚未写入控制所用状态量 |

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
│   imu/         IMU 抽象层 + ICM 最小驱动 + BMI088 适配层  │
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
│   ├── mavlink_app.{c,h}           端口装配 + 初始化 + 解析任务 + 遥测 TX DMA
│   └── mavlink_telemetry.{c,h}     姿态/机体速度/离地高度 MAVLink 打包
├── algorithm/                  算法
│   ├── pid.{c,h}                   PID 控制器
│   ├── filter.{c,h}                二阶低通滤波
│   └── optical_flow.{c,h}          下视光流旋转补偿及速度/高度估计
├── bsp/                        板级驱动
│   ├── imu/                        IMU 抽象层(详见下文)
│   ├── bsp_platform.{c,h}          公共 DWT 计时与 DMA 非缓存 MPU 区
│   ├── spl06.{c,h}                 SPL06 气压计(I2C 中断读取)
│   ├── bmi08*.{c,h}                Bosch BMI088 官方驱动
│   └── bsp_config.h                放置方式/标定开关（IMU 型号上电自动探测）
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
| **SPI4** | ICM-45686 / ICM-42688-P | SCK=PE12, MISO=PE13, MOSI=PE14, CS=PE11, **INT=PE15** | 同一贴片位置，15MHz, Mode0, 中断+DMA |
| **SPI1** | BMI088 | SCK=PA5, MISO=PA6, MOSI=PD7, gyroCS=PC15, accelCS=PB2 | 中断未引出 → 轮询 |
| **I2C2** | SPL06 气压计 | SCL=PB10, SDA=PB11 | `HAL_I2C_Mem_Read_IT`，I2C 事件/错误中断 |
| **USART3** | MAVLink 遥控/上位机 | TX=PD8, RX=PD9 | **460800** 8N1, DMA(Stream5/6) |
| **USART2** | MAVLink 光流/测距 | RX=PD6 (TX=PD5 未用) | 115200 8N1, DMA(Stream2, circular) |
| **TIM3** | 推力电机 ×2 | CH3=PB0, CH4=PB1 | PWM, 1000–1900 |
| **TIM5** | 舵机 ×3 | CH2=PA1, CH3=PA2, CH4=PA3 | PWM, 1100–1900(中位1500) |
| **EXTI15_10** | ICM 数据就绪中断 | PE15 | 触发 IMU 采集 |

执行器映射由 `bsp/bsp_config.h` 的 `AIRCRAFT_MODEL` 选择：

| 样机 | 板子安装 | TIM3 CH3 | TIM3 CH4 | TIM5 CH2 | TIM5 CH3 | TIM5 CH4 |
|------|----------|-----------|-----------|-----------|-----------|-----------|
| A | 竖放 | 左电机：油门 + Roll 差速 | 右电机：油门 − Roll 差速 | 左舵机：Pitch + Yaw | 右舵机：−Pitch + Yaw | 未使用 |
| B | 竖放 | 单电机油门 | 未使用 | Yaw 舵机 | Pitch 舵机 | Roll 舵机 |
| C | 平放 | 单电机油门 | 未使用 | Yaw 舵机 | Pitch 舵机 | Roll 舵机 |

三轴控制输出先送入 `algorithm/aircraft_mixer.c`，再统一写入 PWM；遥控和上位机模式共用混控。角速度内环使用二阶低通后的反馈。控制器输出按脉宽增量处理，油门统一为 0..1（遥控的 0..100 除以 100，上位机直接使用 thrust）。电机范围 1000–1900us，舵机范围 1100–1900us，中位默认 1500us；方向增益、中位和范围集中在配置文件。

串级控制由 `algorithm/attitude_control.c` 实现，内部轴序为 Roll/Pitch/Yaw：

```text
目标角(deg) -> 角度 PID -> 角速度目标(deg/s) + 遥控前馈
                                               |
陀螺仪(rad/s) -> 转 deg/s -> 动态二阶低通 -> 角速度 PID -> us 增量 -> 样机混控
```

遥控前馈为 `Kff × Δ目标角 / Δ遥控帧时间`，默认 Kff=0.5，限幅 ±100deg/s，一阶平滑时间常数 20ms；新遥控帧才更新差分，其余控制周期保持并平滑，首帧和模式切换时清零。Yaw 使用最短角差，避免 ±180° 跳变。前馈只在遥控模式启用，上位机仍使用同一三轴串级 PID。所有参数位于 `bsp_config.h` 的 `CONTROL_*` 配置。

角度 PID 输出与前馈之和限幅 ±200deg/s，角速度 PID 输出限幅 ±400us；积分按实际 dt 累加，微分按实际 dt 计算，包含积分限幅和条件抗饱和。调试器观察 `g_attitude_control` 的 `feedforward`、`rate_target`、`output` 和两级 PID。初始 Ki/Kd 为零，结构支持后续整定。

姿态任务用 `ins_get_snapshot()` 原子发布姿态与角速度；上电初始化/标定完成前不进入控制，IMU 数据超过 10ms 或遥控超过 100ms 未更新进入零力，上位机模式另检查 100ms 指令有效期。退出控制时清空 PID 积分、历史差分和前馈状态。

A 的单侧差速默认是 `0.3 × Roll`，最大 ±200us，并受上下油门余量限制以保持平均油门。零油门和满油门时对称差速余量为零；需要最大 Roll 权限时应预留油门余量。B/C 第二路电机固定停机，A 第三路舵机固定回中且不启动对应 PWM 通道。

`AIRCRAFT_PWM_ENABLE` 默认 **0**，保留当前工程关闭物理 PWM 的调试状态；改为 **1** 后，飞控任务先写停机/回中值，再启动当前样机实际使用的通道。零力与仅调参状态每周期都输出停机/回中值。

---

## FreeRTOS 任务

| 任务 | 函数 | 优先级 | 栈 | 周期 | 职责 |
|------|------|--------|----|------|------|
| INS | `INS_task` | High | 256 | 1ms | IMU 取数 + Madgwick 姿态解算 + 气压计(50Hz 分频) |
| FLY_CTRL | `Aircraft_ctrl_task` | Normal | 256 | 1ms | 模式状态机 + 串级 PID + 电机/舵机输出 |
| MAV_LINK | `mavlink_task` | AboveNormal | 512 | 事件驱动 | 等待 UART RX 通知并解析；绝对截止时间驱动 20Hz 遥测 TX DMA |
| defaultTask | `StartDefaultTask` | Normal | 128 | 1ms | 空闲占位 |

任务创建见 `Core/Src/freertos.c`。MAV_LINK 任务受 `CURRENT_RC_PROTOCOL == RC_PROTOCOL_MAVLINK` 编译期开关控制。

---

## 数据流

```
上电探测 ─► ICM-45686 / ICM-42688-P / BMI088 ─► 选择可初始化的最高优先级设备
                         │
                         ├─ ICM: DRDY + SPI4 DMA ─┐
                         └─ BMI: SPI1 轮询任务 ───┴─► imu_read() ─► INS_task
                                                                        │
                                                                        └─ Madgwick ─► INS_angle ─┐
                                                                                                   │
遥控器/上位机 ─USART3─MAVLink─► 完整帧快照（RC / PC）───────────────────────────────► Aircraft_ctrl_task
                                                                                                   │
光流/测距 ────USART2─MAVLink─► 光流快照 → 速度/高度估计 → USART3 遥测（未进控制）   │
气压计 SPL06 ─I2C2 中断读取─► 高度计算（待接入控制）                               ▼
                                                                                         TIM3 电机 / TIM5 舵机
```

IMU 由 `imu_read()` 复制最新完整快照，返回值说明是否已发布过有效采样，`timestamp` 可用于判断是否为新帧；INS 和飞控均通过 `vTaskDelayUntil()` 按 1ms 绝对周期调度；Madgwick 和 PID 使用 DWT 实测 dt。`imu_get_data()` 保留兼容，返回指针不可跨采样周期保存。光流通过 `mavlink_flow_get_snapshot()` 原子复制并检查超时，旧指针接口仅保留兼容；遥控和上位机指令按完整快照发布，飞控任务每轮原子复制一份本地数据。

---

## MAVLink 接收子系统

采用 **DMA(circular) + IDLE/HT/TC 中断 + FreeRTOS 任务通知**。核心抽象 `MavlinkPort_t` 把“一条物理串口流”封装为一个端口实例，每个端口使用独立的 `mavlink_channel_t`，保证解析状态机互不污染。

| 端口 | 串口 | channel | 消息 | 产出 |
|------|------|---------|------|------|
| `g_rc_port` | USART3 | COMM_0 | RC_CHANNELS_OVERRIDE / SET_ATTITUDE_TARGET | `ELRS_Data` / `PC_Data_t` |
| `g_flow_port` | USART2 | COMM_1 | OPTICAL_FLOW_RAD(106) / DISTANCE_SENSOR(132) | `OpticalFlow_Data_t` |

**接收原理**：DMA 硬件持续循环写入非缓存缓冲区。IDLE 用于短帧或帧间空闲时的低延迟唤醒，HT/TC 保证连续字节流即使没有空闲也能定期唤醒；TC 同时累计 DMA 回绕次数。中断只记录事件并通知任务，任务根据 `回绕次数 × 缓冲大小 + (缓冲大小 - NDTR)` 算出累计生产量，再从软件读指针 `tail` 排空新字节并逐个喂给 `mavlink_parse_char`。若任务落后超过一整圈，会统计覆盖字节并重置残帧解析状态。

USART3 RX 缓冲为 1024B，USART2 RX 缓冲为 256B，遥测 TX 缓冲为 160B，均位于 RAM_D2 的 `.dma_buffer` 非缓存区。遥控帧发布时携带 `update_ms` 和 `frame_count`；超过 `RC_LINK_TIMEOUT_MS`（当前 100ms）没有新遥控帧，飞控状态机会进入 `Aircraft_ZERO_FORCE`。遥测默认使用 50ms 的绝对截止时间，RX 通知频率不会改变 20Hz 发送节拍；USART3 TX 使用 normal DMA，忙时跳过本批，超时仅终止 TX 并恢复，不停止 RX。

原 AT32F421 光流模块的协议说明见 [`光流测距_串口数据协议说明.md`](光流测距_串口数据协议说明.md)。接收端支持 MAVLink v1/v2 的 106/132 消息，输入周期以实际模块为准，与 20Hz 遥测周期独立。

### 上位机遥测

地面电脑侧完整对接说明见 [地面电脑 MAVLink 遥测接收说明](地面电脑_MAVLink遥测接收说明.md)，部署接收入口为 **地面电脑 UDP 14550**，由串口/网络链路转发飞控输出。

交给 Ubuntu 端 Codex 实现 ROS2 接收工程时，同时提供 [Ubuntu Codex ROS2 工程构建说明](Ubuntu_Codex_ROS2工程构建说明.md)，其中包含包结构、接口、依赖、构建启动和离线验收要求。

上位机接 USART3 TX（PD8），**460800 / 8N1**，MAVLink v2，SYSID=1、COMPID=1。每 50ms 发送以下五帧，各消息/命名值均为 **20Hz**，整批共用 `time_boot_ms`。仅发送表中数据及有效位，不发送心跳包或固定测试数据。频率由 `bsp/bsp_config.h` 的 `MAVLINK_TELEMETRY_PERIOD_MS` 配置。

| 消息 | 字段/名称 | 单位与约定 |
|------|-----------|------------|
| `ATTITUDE` (30) | `roll` / `pitch` / `yaw` | rad；上位机显示度时乘 180/π，角速度字段为 rad/s |
| `NAMED_VALUE_FLOAT` (251) | `vx_body` | m/s，机体 X 向前为正 |
| `NAMED_VALUE_FLOAT` (251) | `vy_body` | m/s，机体 Y 向右为正 |
| `NAMED_VALUE_FLOAT` (251) | `height` | m，距地面向上为正，不是 NED z，也不是气压海拔 |
| `NAMED_VALUE_INT` (252) | `telem_ok` | 有效位：bit0=姿态、bit1=XY速度、bit2=高度；7=全部有效 |

无效量发送 **NaN**，上位机应结合 `telem_ok` 和自身接收超时判断，不能把无效量当作零速度/零高度。INS 未初始化或超过 10ms 未更新时全部无效；光流/距离估计超过 100ms 无新数据时对应量无效。低质量光流只撤销速度，不撤销有效测距高度。两种输入报文分别记录接收时间与帧计数。

下视光流解算先把 `integrated_x/y` 映射为机体角轴光流并除以积分时间，再减去本机 INS 的 X/Y 角速度，得到 `vx_body=(flow_y-gyro_y)×distance`、`vy_body=-(flow_x-gyro_x)×distance`。距离使用光流帧自带的米值；高度可由任一种报文中的距离计算 `height=distance×cos(roll)×cos(pitch)`。这是近悬停观测估计，假定下视光轴沿机体 +Z、测距方向一致且地面近似水平，未融合 IMU 加速度、未积分位置、未加入控制。旋转补偿使用**接收时角速度近似积分窗口均值**，没有硬件时间同步和杆臂补偿，快速转动时仍可能产生假速度。

安装轴/符号在 `bsp/bsp_config.h` 配置：`FLOW_BODY_X/Y_AXIS`（0=X、1=Y）和 `FLOW_BODY_X/Y_SCALE`（符号及比例）先于旋转补偿生效。当前实测轴对应正确、两轴速度符号相反，保留同名轴映射并将两项 SCALE 均设为 -1；烧录后复核前移 vx>0、右移 vy>0。标准光流角轴约定下，映射后前移应产生 +flow_y，右移应产生 -flow_x。比例和原地 Roll/Pitch 旋转补偿效果仍需实测。可参考 [MAVLink 光流字段定义](https://mavlink.io/en/messages/common#OPTICAL_FLOW_RAD) 和 [PX4 光流方向说明](https://docs.px4.io/main/en/sensor/optical_flow)。

默认仅接受 0.08–4m 距离、5–50ms 积分时间、quality≥10、合成倾角约≤30°，并拒绝非有限数及任一轴超过 10m/s 的速度；均集中在 `FLOW_*` 配置中。发送端协议文档中“4000cm=4米”和速度量纲描述存在矛盾，因此不把其 `max_distance` 字段作为可信物理量程；实际比例和工作范围需实测。

### 光流方向与倾斜高度验证

2026-11-03 用户确认更换发送端后的光流、测距实板接收测试已通过。本轮删除了原始载荷诊断结构和重复解码检查，以及针对旧发送端的临时互操作测试；正常帧计数、时间戳、质量、原始数据和估计有效位继续保留。

调试器观察 `flow_data`（有重名时使用 `'mavlink_flow.c'::flow_data`）和 `imu_data`。验证步骤、观察变量、轴向调整方式与定高倾斜的几何预期见 [光流方向与倾斜高度实测说明](docs/光流方向与倾斜高度实测说明.md)。

原 AT32 发送端的历史源码分析保留在 [光流收发对照检查（2026-11-02）](docs/光流收发对照检查_20261102.md)，其中的旧发送端问题不代表新模块的状态。主机回归仍覆盖 v1/v2、版本切换及尾零截断，不编入飞控固件。

---

## IMU 抽象层

位于 `bsp/imu/`，使用普通 C 函数和固定型号分发，不依赖 C++ 类或虚函数：

| 文件 | 说明 |
|------|------|
| `imu_api.{c,h}` | 统一接口、探测选择、坐标转换、双缓冲发布与完整快照读取 |
| `icm_acq.{c,h}` | 两款 ICM 共用的阻塞 SPI 传输、DRDY→DMA→解析任务、原始双缓冲与错误统计 |
| `icm45686.{c,h}` | ICM-45686 探测、初始化、DRDY 使能与解码（±16g/±2000dps/800Hz） |
| `icm45686_regs.h` | 本工程使用的 ICM-45686 直接/间接寄存器和配置位 |
| `icm42688p.{c,h}` | ICM-42688-P 探测、初始化、DRDY 使能与解码（±16g/±2000dps/1kHz） |
| `icm42688p_regs.h` | 本工程使用的 ICM-42688-P 寄存器和配置位 |
| `bmi088.{c,h}` | BMI088 驱动（SPI1 独立轮询任务，封装 Bosch C 驱动） |
| `../bsp_platform.{c,h}` | 板级公共 DWT 计时与 MPU 配置，在 main 中先于 DMA/任务初始化 |
| `imu_internal.h` | 驱动内部公共发布入口 `imu_publish()` |
| `imu_frame.{c,h}` / `imu_orientation.h` | 传感器系 → 机体系旋转映射（机体系：X前/Y右/Z下） |
| `imu_calib.{c,h}` | 零偏标定（上电实测或写死值） |
| `imu_selftest.h` | 三类 IMU 上电诊断信息声明，定义与兼容探测入口合并在 imu_api.c |

启动时会读取三类芯片的身份寄存器，并按 `ICM-45686 > ICM-42688-P > BMI088` 尝试完整初始化；高优先级初始化失败时继续尝试下一款。`g_imu_detected_mask` 和 `g_imu_active_type` 可在调试器中查看，上层无需按芯片分支。

ICM 运行期链路为：`PE15 DRDY ISR → 启动 15 字节 SPI4 TxRx DMA → DMA 完成 ISR → 一次任务通知 → 解析/换算/标定 → 双缓冲发布`。DRDY ISR 不再先唤醒采集任务，也不在中断中做浮点运算。

三款 IMU 的当前运行参数：

| 型号 | 总线与采集方式 | 加速度计 | 陀螺仪 | 输出速率 |
|------|----------------|----------|--------|----------|
| ICM-45686 | SPI4，DRDY + DMA | ±16g | ±2000dps | 800Hz |
| ICM-42688-P | SPI4，DRDY + DMA | ±16g | ±2000dps | 1kHz |
| BMI088 | SPI1，独立轮询任务 | ±6g，800Hz | ±1000dps，1kHz | 由轮询任务发布 |

ICM-45686 与 ICM-42688-P 共用同一贴片位置和 SPI4 引脚。两者的寄存器配置与运行期读数均由本工程最小驱动完成，不依赖厂商 SDK；BMI088 适配层继续使用 Bosch 驱动。

调试器可观察以下全局量：

| 变量 | 含义 |
|------|------|
| `g_imu_detected_mask` | 探测位图：bit0=ICM-45686、bit1=ICM-42688-P、bit2=BMI088 |
| `g_imu_active_type` | 当前设备：0=None、1=ICM-45686、2=ICM-42688-P、3=BMI088 |
| `g_imu_selftest` | 各芯片 ID、探测结果、初始化返回值和最终选择结果 |
| `g_imu_status` | 采集启动结果、运行标记、采样数、DRDY 数、丢采样/DMA/SPI/无效样本计数 |

数据在发布前统一执行：物理量换算 → `imu_frame` 轴映射 → 板子系零偏扣除 → `imu_orientation` 放置旋转。统一输出单位为陀螺仪 rad/s、加速度计 m/s²。

角速度内环的三轴二阶 Butterworth 位于 `algorithm/filter.c`，由 `attitude_control.c` 实际调用。采用带预畸变的双线性变换计算系数，默认按 1kHz 控制采样率工作，三个轴保存独立历史状态；首帧用当前测量预填历史，退出控制/模式切换清空状态。

默认动态模式以 `fc_target = fc_min + throttle × (fc_max-fc_min)` 调度截止频率，三款初值均为 30–80Hz；每 10ms 更新，50ms 时间常数平滑截止频率，并保留历史状态更换系数。这是**油门调度带宽**，不是 FFT/频谱自适应。滤波仅进入角速度反馈路径，未改变 INS 的 Madgwick 输入；前馈仍保留独立的一阶平滑。

`CONTROL_GYRO_LPF_ENABLE=0` 旁路滤波；`CONTROL_GYRO_LPF_DYNAMIC_ENABLE=0` 使用固定 `CONTROL_GYRO_LPF_FIXED_HZ`。截止频率必须大于 0 且小于控制采样率的一半，非法配置会使控制器初始化失败。调试器可观察 `g_attitude_control.filtered_rate` 和 `lpf_cutoff_hz`。

原始 DMA 缓冲有 `FREE / DMA / READY / PROCESSING` 四种状态；解析期间不会被下一次 DMA 覆盖。两帧同时待解析时按时间戳先后处理。转换后的数据由统一发布函数写入另一份缓冲后切换索引；`imu_read()` 在短临界区中复制一帧，BMI088 也使用同一发布路径。

---

## IMU 与编译期配置

`bsp/bsp_config.h` 统一选择样机，自动推导安装方向与混控构型。默认选择 A；切换 B/C 时只改 `AIRCRAFT_MODEL`，不要另行定义 `BOARD_ORIENT_*`。IMU 型号仍在上电自动探测：

```c
#define AIRCRAFT_MODEL AIRCRAFT_MODEL_A // 可选 AIRCRAFT_MODEL_B / AIRCRAFT_MODEL_C
#define AIRCRAFT_PWM_ENABLE 0           // 1=启动所选样机的实际 PWM 通道

#define IMU_CALIB_ON_BOOT     // 上电实测零偏(需静止水平~1s); 注释则用写死值
#define IMU_CALIB_SAMPLES 500U
```

A/B 的竖放沿用 `imu_orientation.h` 现有旋转：`V=(-Bz, Bx, -By)`；C 使用恒等旋转。gyro/accel 使用同一安装旋转。这里的“静止水平”指飞行器机体系水平，A/B 的板子仍竖放。

PID 参数在同一个配置文件的“三款样机 PID 参数修改区”分别保存：

| 样机 | 角度外环参数 | 角速度内环参数 |
|------|--------------|----------------|
| A | `MODEL_A_ANGLE_ROLL/PITCH/YAW_PID` | `MODEL_A_RATE_ROLL/PITCH/YAW_PID` |
| B | `MODEL_B_ANGLE_ROLL/PITCH/YAW_PID` | `MODEL_B_RATE_ROLL/PITCH/YAW_PID` |
| C | `MODEL_C_ANGLE_ROLL/PITCH/YAW_PID` | `MODEL_C_RATE_ROLL/PITCH/YAW_PID` |

例如 `MODEL_A_ANGLE_ROLL_PID {5.0f, 0.0f, 0.0f}` 中三个数依次为 Kp、Ki、Kd。修改对应样机组后重新编译；下方 `CONTROL_*_PID` 是自动选择的别名，不必修改。三组目前保留相同初值，尚未分别实机整定。运行时遥控调参仅修改 RAM，重启后重新加载配置文件值。

三款滤波范围也在各自组内配置：`MODEL_A/B/C_GYRO_LPF_MIN_HZ` 与 `MODEL_A/B/C_GYRO_LPF_MAX_HZ`。源文件注释说明任务、协议接收、MPU、IMU、姿态融合、串级控制、前馈、滤波和混控的实际调用关系。

其他：
- `Core/Inc/main.h` — `CURRENT_RC_PROTOCOL`（MAVLink / CRSF）。
- `application/mavlink_app.h` — `MAVLINK_FLOW_PORT_ENABLE`（光流口开关）。

---

## 构建与烧录

工具链由 STM32 VS Code 扩展捆绑（cmake / ninja 不在系统 PATH）。命令行构建（PowerShell）：

```powershell
$env:Path = "C:\Users\<user>\AppData\Local\stm32cube\bundles\cmake\<ver>\bin;" +
            "C:\Users\<user>\AppData\Local\stm32cube\bundles\ninja\<ver>\bin;" +
            "C:\Users\<user>\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\<ver>\bin;" +
            $env:Path

cmake --preset Debug          # 改了 CMakeLists / 加了源文件后需重跑
cmake --build build/Debug      # 产物: build/Debug/fly_control_stm.elf
```

- 推荐使用 **STM32 VS Code 扩展（cube-cmake）** 一键构建/调试/烧录。
- 也可用 STM32CubeIDE 导入 `.ioc`。
- 工具链配置见 `cmake/gcc-arm-none-eabi.cmake`；项目仅启用 C 和 ASM，使用 GCC 编译与链接。
- `.dma_buffer` 位于 RAM_D2，链接脚本限制该非缓存区最大为 2KB；超过容量时链接会直接失败。

IMU 主机回归测试（需要本机 GCC；可通过环境变量 `HOST_CC` 指定编译器）：

```powershell
python tests/imu/run_tests.py
python tests/control/run_tests.py
python tests/telemetry/run_tests.py
```

测试直接编译生产代码中的自动选择、两款 ICM 初始化/解码、共享 DMA 状态机和发布接口，每款样机覆盖 128 组设备存在、初始化失败与任务创建失败组合（三款共 384 组），并检查双缓冲占用、数据换算、无效样本和错误恢复。HAL、FreeRTOS 与 BMI088 适配层使用模拟，实板 SPI/DMA 时序仍需上板验证。

控制测试分别编译 A/B/C 三种配置，验证差速正反方向、双舵机混控、独立舵机输出、饱和限制、零油门、停机值、未使用通道和安装旋转；非法样机编号必须在编译期报错。串级控制测试覆盖三轴内外环、角速度反馈、遥控前馈、Yaw 回绕、复位、积分 dt 和抗饱和，并验证输出进入当前样机混控；滤波测试检查低频通过、截止处 -3dB、高频衰减、动态调频直流连续性、参数拒绝、固定带宽和旁路分支。

遥测测试编译真实光流解码/解算/打包代码，覆盖 A/B/C 和交换轴/反号/比例配置；通过官方 parser 重新解析发送字节，检查消息、CRC、单位、姿态轴序、命名字段、有效位、旋转补偿、无效输入、独立超时及 Tick 回绕。另检查真实发送函数在 DMA 忙时不覆盖缓冲、超时/终止失败/启动失败后的恢复，以及不终止 RX。HAL/INS 使用模拟，仍需验证实板发送、光流方向与速度比例。

接收链路回归使用实际 UART RX 回调、`MavPort_Drain()`、官方 parser 和光流派发代码，模拟 2000 对光流/测距帧，覆盖 DMA 多圈回绕、半帧、双串口交错、CRC 错误与覆盖后恢复，并验证 INS 未初始化时仍发布原始观测和帧计数。DMA 寄存器和任务通知由主机模拟，不能替代接线和实板中断验证。

另使用 Python 标准库按协议偏移及独立的 CRC 算法生成原始报文字节，测试非零光流、静止光流、quality=0/87/255、44 字节完整载荷和 40 字节截断载荷，避免仅靠同一套 MAVLink 库的打包/解包互测。原始载荷与解码结果必须一致。

---

## 已知问题与待办

- **ICM-45686 需完成实板验证**：重点检查上电初始化、DRDY 频率、DMA 连续采样、温度值和静止时三轴方向。
- **IMU 轴映射需随硬件复核**：ICM 与 BMI088 的当前映射集中在 `bsp/imu/imu_frame.c`，换板或改安装方向后需重新验证。
- **控制参数待整定**：串级 PID 和遥控前馈已接入，默认参数不能替代样机实测整定。
- **动态滤波范围待实测整定**：三轴角速度内环已接入二阶 Butterworth，默认油门调度 30–80Hz；没有自动识别扑翼频谱或实飞抑振效果记录。
- **光流数据未接入控制**：机体 XY 速度与离地高度已接入遥测，方向/比例及旋转补偿效果待实测，尚未接入 `Aircraft_ctrl_task` 做定高/水平速度控制。
- **气压计高度未接入**：SPL06 已读取，解算高度尚未写入 INS 数据结构供控制使用。
- **加速度零偏/温补、IMU 温度读取**等为可选优化项（BMI088 温度暂填 0）。
- MAVLink 遥测仅从 USART3 下行，光流 USART2 保持单向输入；未实现消息频率请求/参数协商。

---

## 协议与依赖

- **MAVLink**：c_library_v2（common dialect），见 `Third_party/mavlink_c/`。
- **FreeRTOS** + **CMSIS-RTOS v1**。
- **CMSIS-DSP**（ARM Math）。
- **Bosch BMI08x 驱动**；ICM-45686 与 ICM-42688-P 由本工程按各自数据手册直接驱动。

---

*本工程仍在持续开发中，欢迎参考学习。如用于实飞，请充分测试并自负风险。*
