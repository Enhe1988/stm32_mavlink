# IMU 抽象层重构设计文档（ICM-42688-P 中断+DMA 接入 + PX4 式抽象）

> 本文档是实现规格。新对话照此实现即可。讨论已完成，下面是锁定的方案。

## 0. 背景与目标

主工程 stm32_mavlink（STM32H743VIH6 + FreeRTOS/CMSIS-OS）当前用 **BMI088（SPI1，轮询）** 做姿态，INS_task 直接耦合 Bosch `bmi08` 数据结构。本次要：

1. 接入 **ICM-42688-P（SPI4，中断+DMA）** 作为可选 IMU
2. 仿 **PX4** 做一层 IMU 抽象（C++ 实现，对外暴露 C 接口）
3. **编译期**选择当前使用哪个 IMU
4. **重构 INS_task** 成纯 consumer，只认统一数据结构，与具体芯片解耦

## 1. 硬件事实（来自原理图 + 已在 icm42686 测试工程验证）

| IMU | 总线 | 引脚 | 中断 |
|---|---|---|---|
| **ICM-42688-P** | SPI4 | SCK=PE12, MISO=PE13, MOSI=PE14, **CS=PE11** | **INT=PE15** |
| **BMI088** | SPI1 | SCK=PA5, MISO=PA6, MOSI=PD7, gyro CS=PC15(CS1), accel CS=PB2(CS2) | **未引出 → 只能轮询** |

- 原理图把 PB2 误标成 GYRO1_INT，实际 PB2 是 BMI088 的 accel CS（已与用户确认）。
- SPI4 时钟：PCLK2(D2PCLK1)=120MHz，预分频 8 → **SCLK=15MHz**（≤24MHz 上限）。
- SPI4 模式：**Mode 0**（CPOL=0/CPHA=0）、8-bit、MSB first。SPI4 三个复用脚速率必须 **Very High**（默认 Low 在 15MHz 下边沿变形）。
- ICM 已用手写 ReadReg 验证过 WHO_AM_I=0x47 通信正常（见 icm42686 测试工程）。

## 2. ICM-42688-P 寄存器与流程

WHO_AM_I=**0x75**，期望 **0x47**（Bank0 直读）。读：地址字节 `addr | 0x80`。

### 初始化序列（上电一次）
| 步骤 | 寄存器 | 操作 |
|---|---|---|
| 软复位 | DEVICE_CONFIG (0x11) | bit0=1，等 ~1ms |
| 校验 ID | WHO_AM_I (0x75) | ==0x47 |
| INT 电气 | INT_CONFIG (0x14) | INT1 推挽 / 高有效 / 脉冲模式 |
| 中断路由 | INT_SOURCE0 (0x65) | UI_DRDY_INT1_EN=1 |
| 陀螺量程+ODR | GYRO_CONFIG0 (0x4F) | ±2000dps + 1kHz |
| 加速度量程+ODR | ACCEL_CONFIG0 (0x50) | ±16g + 1kHz |
| 滤波(可选) | GYRO_ACCEL_CONFIG0 (0x52) | UI filter 带宽 |
| 开电源 | PWR_MGMT0 (0x4E) | GYRO_MODE=LN, ACCEL_MODE=LN |

⚠️ 手册警告：开 Low Noise 后等 ~200µs；**开陀螺仪后 45ms 内不要写其它寄存器**。

### 运行期（每次 DRDY）
- burst read 从 **0x1D 连续 14 字节**：TEMP(2)+ACCEL_XYZ(6)+GYRO_XYZ(6)，加 1 地址字节 → **DMA 15 字节**
- 解析：`int16 = (hi<<8)|lo`；`accel_g=raw/2048`（±16g）；`gyro_dps=raw/16.4`（±2000dps）；`temp_C=raw/132.48+25`

### 配置参数（默认值，可调）
ODR=1kHz，陀螺 ±2000dps，加速度 ±16g，含温度。

## 3. 数据读取方案：DMA 后端的同步 read_reg（锁定，已按官方驱动调整）

> **重要**：因为采用官方 Icm426xx 驱动（见 §9），其数据读取函数 `inv_icm426xx_get_data_from_registers()` 内部是**同步调用 serif 的 `read_reg` 并期望返回时数据已就绪**。所以"中断里直接启动 DMA、完成回调里再处理"那套**不适用**了——改成"中断通知任务 → 任务调驱动 → 驱动的 read_reg 内部用 DMA 阻塞读"。这其实就是之前讨论的"方案B + DMA 后端"。

```
t0  PE15上升沿 → EXTI15_10_IRQHandler → HAL_GPIO_EXTI_Callback()
      【ICM驱动】vTaskNotifyGiveFromISR(icm_task)   // 中断只发通知,极轻
    中断退出
t1  调度切到 icm_task:
      ulTaskNotifyTake() 返回
      → inv_icm426xx_get_data_from_registers(&icm_driver)   // 官方驱动
          → 内部调 我们的 read_reg(reg, buf, len):
                CS拉低
                HAL_SPI_TransmitReceive_DMA(&hspi4, tx, rx, len+1)
                xSemaphoreTake(spi_dma_done)   // 阻塞等DMA,期间CPU让给别的任务
                CS拉高
                返回数据
          → 驱动解析后回调 sensor_event_cb(accel, gyro, temp)
                → 缩放标定 + 应用板子旋转 → 填 ImuData
   ── DMA完成中断 HAL_SPI_TxRxCpltCallback(SPI4):
          xSemaphoreGiveFromISR(spi_dma_done)   // 唤醒上面的阻塞读
```

要点：
- **两层信号**：① EXTI 任务通知（DRDY→唤醒采集任务）；② DMA 完成信号量（在 read_reg 内部用，等单次 SPI 传输完成）
- read_reg 对官方驱动呈现为"同步返回"，但底层走 DMA，等待期间任务让出 CPU
- DMA normal 模式，传完自停；read_reg 每次重新调用启动函数，**无需手动恢复**
- 1kHz 下传输约 8µs，远小于 1ms 周期，安全
- ISR 内只做 `GiveFromISR`，绝不在中断里调官方驱动（驱动函数会阻塞）

### ⚠️ H743 DMA + Cache（必处理）
DMA1/DMA2 **访问不到 DTCM**，且 D-Cache 会导致 CPU 读到旧值。措施：
- tx/rx buffer 放 **D2 域 SRAM**（如 SRAM1/AXI SRAM），不能放默认 DTCM
- 用 **MPU 划一块该区域为 non-cacheable**（工程已有 MPU_Config，加一个 region）
- 或退而求其次：DMA 完成回调里 `SCB_InvalidateDCache_by_Addr(rx, len)`（buffer 需 32 字节对齐）。**优先用 MPU 非缓存区**

### 中断优先级
EXTI15_10 与 SPI4 DMA 流中断，抢占优先级数值 **≥ configMAX_SYSCALL_INTERRUPT_PRIORITY**（即优先级低于阈值，如设 5~6），否则 ISR 调 FreeRTOS API 触发断言。

## 4. 节拍归属：做法 A（各驱动自管节拍，锁定）

- **ICM 驱动**：`start()` 起**自己的采集任务** + 使能 PE15 EXTI。采集任务等 DRDY 通知 → 调官方驱动同步读（read_reg 内部 DMA）→ 填内部 latest，`get_data()` 返回最新帧
- **BMI088 驱动**：`start()` 起**自己的轮询任务**（SPI1 按节拍读，填 latest），包住现有 `bmi08a/g` C 驱动
- **INS_task**：只调 `active_imu->get_data()`，对两者零分支
- 两个 IMU 各有一个采集任务（ICM 中断驱动 / BMI088 定时驱动），互不干扰

## 5. C++ 抽象层设计（对外 C 接口）

语言：抽象层用 **C++**；对外（INS_task 等）暴露 **C 接口**（`extern "C"`）。后续可统一为 C++。

### 统一数据结构与接口
```cpp
// bsp/imu/imu.hpp
struct ImuData {
    float gyro[3];   // rad/s, 机体坐标系(已应用旋转)
    float accel[3];  // m/s²
    float temp;      // °C
    uint32_t timestamp;
};

class ImuDriver {
public:
    virtual bool init() = 0;                 // 软复位+配置+自检(读WHO_AM_I)
    virtual void start() = 0;                // ICM:武装EXTI+DMA / BMI088:起轮询任务
    virtual const ImuData& getData() const = 0;
    virtual ~ImuDriver() = default;
};
```

### 派生驱动
- `bsp/imu/Icm42688p.hpp/.cpp`：包官方 **Icm426xx 驱动**（见 §9 对接），采集任务 + DRDY + DMA 后端 read_reg；填 ImuData
- `bsp/imu/Bmi088.hpp/.cpp`：包现有 `bmi08a.c/bmi08g.c/common.c`；轮询任务；填 ImuData
- 驱动对象 **静态构造，在调度器启动前 init**（`-fno-threadsafe-statics` 已设）

### 官方驱动对接（serif + ExtFunc）
`Icm42688p` 类要向官方驱动提供 4 个 C 回调（用 static 成员或文件内 static 函数，经实例指针访问 SPI4/PE11）：
```c
// serif (struct inv_icm426xx_serif) —— 总线读写
int icm_read_reg (serif, reg, buf, len);   // CS低; DMA TransmitReceive(reg|0x80 + len); 等信号量; CS高
int icm_write_reg(serif, reg, buf, len);   // CS低; DMA Transmit(reg + len);            等信号量; CS高
// ExtFunc (Icm426xxExtFunc.h) —— 平台服务
void     inv_icm426xx_sleep_us(uint32_t us);   // DWT 或 定时器 微秒延时
uint64_t inv_icm426xx_get_time_us(void);       // DWT 或 自由运行定时器 微秒时戳
```
serif 配置：`serif_type = ICM426XX_UI_SPI4`，`max_read/max_write` 设足够大（如 1024 / 258）。

### 官方驱动标准调用流程（来自 example-raw-data-registers）
```c
// init():
inv_icm426xx_init(&drv, &serif, sensor_event_cb);           // 注册 serif + 数据回调
inv_icm426xx_configure_fifo(&drv, INV_ICM426XX_FIFO_DISABLED); // 不用FIFO,读寄存器
inv_icm426xx_get_who_am_i(&drv, &id);                       // 校验 0x47
inv_icm426xx_set_accel_fsr(&drv, ICM426XX_ACCEL_CONFIG0_FS_SEL_16g);
inv_icm426xx_set_gyro_fsr (&drv, ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps);
inv_icm426xx_set_accel_frequency(&drv, ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ);
inv_icm426xx_set_gyro_frequency (&drv, ICM426XX_GYRO_CONFIG0_ODR_1_KHZ);
inv_icm426xx_enable_accel_low_noise_mode(&drv);
inv_icm426xx_enable_gyro_low_noise_mode(&drv);
// 还需:配置 INT1 输出 DRDY(驱动有 inv_icm426xx_set_config_int1 / 中断源API)
// 采集任务每次 DRDY:
inv_icm426xx_get_data_from_registers(&drv);   // 内部调 read_reg → 触发 sensor_event_cb
```
`sensor_event_cb(inv_icm426xx_sensor_event_t* e)` 里拿到 raw accel/gyro/temp → 缩放标定 → 填 ImuData。

### ISR 跳板
HAL 的 EXTI/DMA 回调是 C 的，集中在一处按 pin/Instance 分发进对应 C++ 对象：
```cpp
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin){
    if (pin==GPIO_PIN_15) Icm42688p::onDrdyISR();   // → vTaskNotifyGiveFromISR(采集任务)
}
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* h){
    if (h->Instance==SPI4) Icm42688p::onSpiDmaDoneISR();  // → GiveFromISR(spi_dma_done)
}
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* h){  // 写路径也要
    if (h->Instance==SPI4) Icm42688p::onSpiDmaDoneISR();
}
```
> 注意：BMI088 在 SPI1、轮询不用这些回调；若其它外设也用，务必按 Instance 分发，集中维护。

### 编译期选型
```c
// bsp_config.h
#define IMU_USE_ICM42688    // 或 IMU_USE_BMI088
```
```cpp
// bsp/imu/imu_api.cpp —— 对外C接口 + 选型
#include "bsp_config.h"
#if defined(IMU_USE_ICM42688)
  static Icm42688p g_imu;
#else
  static Bmi088 g_imu;
#endif
static ImuDriver* active = &g_imu;

extern "C" {
  bool imu_init(void){ return active->init(); }
  void imu_start(void){ active->start(); }
  const ImuData* imu_get_data(void){ return &active->getData(); } // C侧用同布局struct
}
```
对外 C 头 `imu_api.h` 暴露 `ImuData`（C struct，与 C++ 同内存布局）、`imu_init/imu_start/imu_get_data`。

## 6. INS_task 重构

从"BMI088驱动+融合"拆成纯 consumer：
```c
void INS_task(void const *p){
    imu_init();
    imu_start();
    gyro_bias_calibrate();           // 走统一接口做零偏
    for(;;){
        const ImuData *d = imu_get_data();
        MahonyAHRS_update(d->gyro, d->accel);   // 只认统一数据
        osDelay(1);
    }
}
```
- BMI088 专属的 `struct bmi08_sensor_data`、`BMI088_*_SEN` 灵敏度宏 **移进 Bmi088 驱动内部**，INS_task 不再出现
- 灵敏度换算、零偏标定、板子旋转都在驱动里做完，INS_task 拿到的是标定后的 rad/s、m/s²
- 切换 IMU = 改一行 `#define`，INS_task 不动

## 7. CubeMX 需要配置的（生成代码）

1. **SPI4**：Full-Duplex Master，NSS Disable(软件)，8-bit，MSB，CPOL Low，CPHA 1Edge，Prescaler 8。引脚 PE12/13/14，Max output speed = **Very High**
2. **PE11**：GPIO_Output，Label `ICM_CS`，初始 High，推挽，No pull，Very High
3. **PE15**：GPIO_EXTI15，**上升沿**触发，建议 pull-down（INT 脉冲高有效，空闲低）
4. **SPI4 DMA**：TX + RX 两个流，Normal 模式，数据宽度 Byte，外设/存储器地址按默认
5. **NVIC**：使能 EXTI15_10、SPI4 的 TX/RX DMA 流中断；抢占优先级设 5~6（低于 FreeRTOS 阈值）
6. **DMA buffer 内存**：通过 MPU 给一块 SRAM region 配 non-cacheable（见 §3）

> 重新生成只覆盖 Core/，bsp/application/algorithm 不受影响。

## 8. CMakeLists.txt 改动

顶部：
```cmake
enable_language(C CXX ASM)        # 原为 C ASM
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

`target_sources` 增加（我们写的 + 官方驱动 9 个 .c）：
```cmake
    # 抽象层(我们写)
    bsp/imu/imu_api.cpp
    bsp/imu/Icm42688p.cpp
    bsp/imu/Bmi088.cpp
    # 官方 Icm426xx 驱动(已迁入 bsp/imu/icm426xx_sdk/)
    bsp/imu/icm426xx_sdk/Invn/Drivers/Icm426xx/Icm426xxDriver_HL.c
    bsp/imu/icm426xx_sdk/Invn/Drivers/Icm426xx/Icm426xxDriver_HL_apex.c
    bsp/imu/icm426xx_sdk/Invn/Drivers/Icm426xx/Icm426xxSelfTest.c
    bsp/imu/icm426xx_sdk/Invn/Drivers/Icm426xx/Icm426xxTransport.c
    bsp/imu/icm426xx_sdk/Invn/EmbUtils/DataConverter.c
    bsp/imu/icm426xx_sdk/Invn/EmbUtils/ErrorHelper.c
    bsp/imu/icm426xx_sdk/Invn/EmbUtils/InvBasicMath.c
    bsp/imu/icm426xx_sdk/Invn/EmbUtils/Message.c
    bsp/imu/icm426xx_sdk/Invn/Helpers/Icm426xx/helperClockCalib.c
```
`target_include_directories` 增加：
```cmake
    bsp/imu
    bsp/imu/icm426xx_sdk        # 让官方驱动里 #include "Invn/..." 能解析
```
> 注：`Icm426xxDriver_HL_apex.c`(计步等APEX) 和 `helperClockCalib.c`(外部时钟校准) 我们用不到，可暂时不加进 sources 以减小体积；若编译报未定义符号再补回。`Icm426xxSelfTest.c` 同理(只有调自检才需要)。最小集其实是 `Icm426xxDriver_HL.c` + `Icm426xxTransport.c` + EmbUtils 4个。

## 9. 官方驱动来源（已迁入工程 ✅）

ICM-42688-P 用 InvenSense 官方 **Icm426xx 系列驱动**（covers 42688-P/42605/42622…）。来源：SmartMotion eMD 例程包 `smartmotion-dkexamples-atmel-cm4-fpu-2.0.9`。

**已迁入 `bsp/imu/icm426xx_sdk/Invn/`**（保留 `Invn/` 前缀以匹配驱动内部 include）：
```
Invn/Drivers/Icm426xx/   Icm426xxDriver_HL.c/.h  Icm426xxDriver_HL_apex.c/.h
                         Icm426xxSelfTest.c/.h   Icm426xxTransport.c/.h
                         Icm426xxDefs.h  Icm426xxExtFunc.h  Icm426xxVersion.h
Invn/EmbUtils/           DataConverter / ErrorHelper / InvBasicMath / Message (.c/.h) + RingBuffer.h
Invn/Helpers/Icm426xx/   helperClockCalib.c/.h
Invn/InvError.h  Invn/InvExport.h
```
未迁入（用不到）：磁力计 `Ak0991x`、融合库 `LibAGM`（我们用自己的 Mahony）。

**对接点**（详见 §5）：实现 serif 的 `read_reg/write_reg`(SPI4+PE11) + ExtFunc 的 `inv_icm426xx_sleep_us/get_time_us`。

> 参考例程：原包 `sources/examples/example-raw-data-registers/` 是最贴近我们用法的范例（读数据寄存器，非 FIFO）。

## 10. 实现顺序建议

1. CubeMX 配 §7 全部外设，生成代码，确认编译通过（先不加 C++）
2. CMakeLists 开 CXX，加空的 imu 抽象层骨架 + 官方驱动源，确认 C/C++ 混编链接通过
3. 实现 ExtFunc（DWT 微秒计时）+ serif（先用**阻塞** `HAL_SPI_TransmitReceive`，不上DMA），跑官方 init + `get_who_am_i` 读到 **0x47**
4. 跑通 `get_data_from_registers` + sensor_event_cb，静止验证 accel≈(0,0,9.8)、gyro≈0
5. serif 升级为 **DMA 后端**（DMA + 完成信号量），配 MPU 非缓存区放 DMA buffer，再验数据
6. 接 PE15 EXTI → 采集任务（DRDY 驱动），形成完整中断+DMA 链路
7. 实现 Bmi088 驱动（包现有 C 驱动 + 轮询任务），填 ImuData
8. 重构 INS_task 为 consumer，编译期切换两个 IMU 各验证一遍

> 注意第 3→5 步是**先阻塞跑通、再上 DMA**，降低一次性调试难度。
