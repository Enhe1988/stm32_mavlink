# 项目:fly_control_stm (stm32_mavlink 主工程)

STM32H743VIH6 飞控主工程,FreeRTOS(CMSIS-OS)。分层:application(任务) / algorithm(AHRS/PID/滤波) / bsp(传感器驱动) / Third_party(mavlink_c)。构建:CMake(Ninja) + arm-none-eabi-gcc,STM32 VS Code 扩展(cube-cmake)。

## 当前任务:IMU 抽象层重构 + 接入 ICM-42688-P —— 设计文档 §10 八步已全部完成 ✅(板上验到第6步,数据/中断链路 OK)

**完整设计规格见 [docs/IMU_Refactor_Design.md](docs/IMU_Refactor_Design.md)。**

一句话:仿 PX4 做 IMU 抽象层,接入 ICM-42688-P(SPI4,中断+DMA),编译期选用哪个 IMU,重构 INS_task 为纯 consumer。

### 现状(已实现)
- 抽象层 `bsp/imu/`:`imu_api.*`(C 接口+编译期选型)/`imu.hpp`(ImuDriver 基类)/`Icm42688p.*`(SPI4 EXTI+DMA,官方 Icm426xx 驱动)/`Bmi088.*`(SPI1 轮询,包 Bosch C 驱动)/`icm_platform.*`(DWT 计时+MPU 非缓存区)/`imu_selftest.*`(双芯片通信 POST)。
- INS_task 已是纯 consumer:`imu_init/imu_start/imu_get_data`,内部统一 **rad/s、m/s²**。下游 Aircraft_ctrl 只用 `INS_angle`(度),未受影响。
- 选型在 `bsp/bsp_config.h`(`IMU_USE_ICM42688` / `IMU_USE_BMI088`),两种都已验证可构建链接。
- DMA buffer 在 `RAM_D2`(链接脚本 `.dma_buffer` 段)+ MPU region1 非缓存。

### 待办(下一步)
- **ICM 板子旋转未标定**:Icm42688p 驱动目前输出芯片原始轴,姿态角可能不对。BMI088 的旋转已在其驱动里(沿用原 INS_task 映射,可飞)。需实测 ICM 各轴朝向后在 `Icm42688p::sensorEventCb` 里补旋转矩阵。
- 加速度零偏/温补、温度读取(BMI temp 暂填 0)等可选项。

### 已锁定的架构决策
- 抽象层用 **C++**(抽象基类 `ImuDriver`),对外暴露 **C 接口**(`extern "C"`),后续再统一为 C++
- **做法A**:各驱动自管节拍 —— ICM 用中断+DMA,BMI088 用自己的轮询任务
- **中断+DMA(按官方驱动调整)**:EXTI(PE15)只发任务通知→采集任务调官方驱动→驱动的 read_reg 内部用 **DMA阻塞读**(DMA+完成信号量)。官方驱动是同步读架构,故 DMA 藏在 read_reg 里,不在ISR里直接启动
- **编译期选型** `bsp_config.h` 的 `#define IMU_USE_ICM42688/BMI088`
- INS_task 重构为只认统一 `ImuData`(rad/s, m/s²),与芯片解耦
- DMA buffer 放 **MPU 非缓存 SRAM 区**(H743 cache 坑)
- ODR 1kHz / 陀螺±2000dps / 加速度±16g / 带温度(可调)
- 官方驱动 = **Icm426xx 系列**(InvenSense eMD),serif 回调架构(我们实现 read_reg/write_reg + sleep_us/get_time_us)

### 硬件(原理图+已验证)
- **ICM-42688-P**:SPI4,SCK=PE12/MISO=PE13/MOSI=PE14,CS=PE11,**INT=PE15**。WHO_AM_I=0x75 期望 0x47(已在 icm42686 测试工程验证通信)
- **BMI088**:SPI1,SCK=PA5/MISO=PA6/MOSI=PD7,gyro CS=PC15,accel CS=PB2,**中断未引出→保持轮询**

### 待办前置
- ✅ 官方 Icm426xx 驱动已迁入 `bsp/imu/icm426xx_sdk/Invn/`(设计文档 §9)
- CubeMX 尚未配置 SPI4/PE11/PE15/DMA/NVIC(清单见设计文档 §7)——**这是实现第一步**

### 实现顺序
见设计文档 §10。简述:CubeMX配外设→CMake开CXX加骨架→实现ICM(自检+DMA)→MPU非缓存区验数据→实现BMI088驱动→重构INS_task两个IMU各验一遍。

## 关联
- 测试验证来自姊妹工程 icm42686(E:/2024/project/stm_fly_control/icm42686),那里手写驱动读到了 ICM 的 0x47。
- 数据手册:icm42686 文件夹有 ICM-42688-P 规格书;飞控原理图在 stm_fly_control/飞控原理图/。
