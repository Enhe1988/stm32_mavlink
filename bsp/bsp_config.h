#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

/*
 * IMU 不再编译期手选。启动时读取身份寄存器并按以下优先级自动选择：
 * ICM-45686 > ICM-42688-P > BMI088。
 */

/* 只修改 AIRCRAFT_MODEL 一处，安装方向和混控随样机自动选择。
 * A：双电机差速 Roll + 双舵机 Pitch/Yaw，竖放。
 * B：单电机 + 独立 Yaw/Pitch/Roll 舵机，竖放。
 * C：控制构型与 B 相同，平放。
 * 测试/独立构建也可用 -DAIRCRAFT_MODEL=AIRCRAFT_MODEL_B 覆盖。 */
#define AIRCRAFT_MODEL_A 1
#define AIRCRAFT_MODEL_B 2
#define AIRCRAFT_MODEL_C 3

#ifndef AIRCRAFT_MODEL
#define AIRCRAFT_MODEL AIRCRAFT_MODEL_A
#endif

#if defined(BOARD_ORIENT_FLAT) || defined(BOARD_ORIENT_VERTICAL)
#error "安装方向由 AIRCRAFT_MODEL 自动选择，不要单独定义 BOARD_ORIENT_*"
#endif

#if AIRCRAFT_MODEL == AIRCRAFT_MODEL_A
#define BOARD_ORIENT_VERTICAL
#define AIRCRAFT_DUAL_MOTOR 1
#elif AIRCRAFT_MODEL == AIRCRAFT_MODEL_B
#define BOARD_ORIENT_VERTICAL
#define AIRCRAFT_DUAL_MOTOR 0
#elif AIRCRAFT_MODEL == AIRCRAFT_MODEL_C
#define BOARD_ORIENT_FLAT
#define AIRCRAFT_DUAL_MOTOR 0
#else
#error "AIRCRAFT_MODEL 必须选择 AIRCRAFT_MODEL_A / B / C"
#endif

#if defined(BOARD_ORIENT_FLAT) && defined(BOARD_ORIENT_VERTICAL)
#error "bsp_config.h: BOARD_ORIENT_FLAT 与 BOARD_ORIENT_VERTICAL 只能二选一"
#endif
#if !defined(BOARD_ORIENT_FLAT) && !defined(BOARD_ORIENT_VERTICAL)
#error "bsp_config.h: 必须定义 BOARD_ORIENT_FLAT 或 BOARD_ORIENT_VERTICAL 之一"
#endif

/* PWM 调试开关：沿用当前工程未开启输出的状态；改为 1 才启动所选样机的通道。 */
#ifndef AIRCRAFT_PWM_ENABLE
#define AIRCRAFT_PWM_ENABLE 0
#endif

/* 执行器脉宽单位 us，与 TIM3/TIM5 当前 1MHz 计数频率对应。 */
#define MOTOR_STOP_US 1000U
#define MOTOR_MAX_US 1900U
#define SERVO_MIN_US 1100U
#define SERVO_MAX_US 1900U
#define SERVO1_CENTER_US 1500U
#define SERVO2_CENTER_US 1500U
#define SERVO3_CENTER_US 1500U

/* 方向/增益可按实机调整；三轴输入是姿态控制器输出，单位为脉宽增量。 */
#define A_MOTOR_ROLL_GAIN 0.3f
#define A_MOTOR_DIFF_MAX_US 200.0f /* 单侧相对基础油门的最大增量 */
#define A_SERVO1_PITCH_GAIN 1.0f
#define A_SERVO1_YAW_GAIN 1.0f
#define A_SERVO2_PITCH_GAIN (-1.0f)
#define A_SERVO2_YAW_GAIN 1.0f
#define BC_SERVO_YAW_GAIN 1.0f
#define BC_SERVO_PITCH_GAIN 1.0f
#define BC_SERVO_ROLL_GAIN (-1.0f)

/* 控制单位：角度 deg、角速度 deg/s、控制输出 us。参数是初始值，后续实机整定。 */
#define CONTROL_PERIOD_MS 1U
#define CONTROL_MAX_ANGLE_DEG 20.0f
#define CONTROL_YAW_RATE_DPS 30.0f
#define CONTROL_MAX_RATE_DPS 200.0f
#define CONTROL_MAX_OUTPUT_US 400.0f
#define CONTROL_MAX_INTEGRAL_US 100.0f
#define CONTROL_IMU_TIMEOUT_MS 10U
#define CONTROL_PC_TIMEOUT_MS 100U

/* 上位机遥测：USART3 / COMM_0，20Hz，仅姿态/速度/高度及有效位，不发心跳。 */
#define MAVLINK_TELEMETRY_PERIOD_MS 50U
#define MAVLINK_TELEMETRY_TX_TIMEOUT_MS 20U

/* 下视光流：将原始 integrated_x/y 映射到机体绕 X/Y 的光流角后，
 * 减本机 gyro，再按 vx=flow_y*距离、vy=-flow_x*距离换算 m/s。
 * 轴编号 0=X、1=Y；SCALE 同时调整符号和比例。
 * 当前安装实测轴序对应正确，两轴反号以使前移 vx>0、右移 vy>0。 */
#ifndef FLOW_BODY_X_AXIS
#define FLOW_BODY_X_AXIS 0
#endif
#ifndef FLOW_BODY_Y_AXIS
#define FLOW_BODY_Y_AXIS 1
#endif
#ifndef FLOW_BODY_X_SCALE
#define FLOW_BODY_X_SCALE (-1.0f)
#endif
#ifndef FLOW_BODY_Y_SCALE
#define FLOW_BODY_Y_SCALE (-1.0f)
#endif
#if (FLOW_BODY_X_AXIS < 0 || FLOW_BODY_X_AXIS > 1 || \
     FLOW_BODY_Y_AXIS < 0 || FLOW_BODY_Y_AXIS > 1 || FLOW_BODY_X_AXIS == FLOW_BODY_Y_AXIS)
#error "光流 X/Y 必须选择不同的轴编号 0 或 1"
#endif
#define FLOW_TIMEOUT_MS 100U
#define FLOW_MIN_QUALITY 10U
#define FLOW_MIN_DISTANCE_M 0.08f
#define FLOW_MAX_DISTANCE_M 4.0f
#define FLOW_MIN_INTEGRATION_S 0.005f
#define FLOW_MAX_INTEGRATION_S 0.05f
#define FLOW_MIN_TILT_COS 0.8660254f /* 合成倾角约 30deg，近悬停观测使用范围 */
#define FLOW_MAX_SPEED_MPS 10.0f

/* ===== 三款样机 PID 参数修改区 =====
 * 每一行的三个数依次是 {Kp, Ki, Kd}，A/B/C 分开保存。
 * ANGLE_* 是外环：deg 误差 -> deg/s 目标；RATE_* 是内环：deg/s 误差 -> us 输出。

 */

/* 样机 A：在这一组修改其三轴外环和内环参数。 */
#define MODEL_A_ANGLE_ROLL_PID {5.0f, 0.0f, 0.0f}
#define MODEL_A_ANGLE_PITCH_PID {2.0f, 0.0f, 0.0f}
#define MODEL_A_ANGLE_YAW_PID {2.0f, 0.0f, 0.0f}
#define MODEL_A_RATE_ROLL_PID {0.5f, 0.0f, 0.0f}
#define MODEL_A_RATE_PITCH_PID {0.5f, 0.0f, 0.0f}
#define MODEL_A_RATE_YAW_PID {0.5f, 0.0f, 0.0f}
#define MODEL_A_GYRO_LPF_MIN_HZ 30.0f
#define MODEL_A_GYRO_LPF_MAX_HZ 80.0f

/* 样机 B：在这一组修改其三轴外环和内环参数。 */
#define MODEL_B_ANGLE_ROLL_PID {2.0f, 0.0f, 0.0f}
#define MODEL_B_ANGLE_PITCH_PID {1.0f, 0.0f, 0.0f}
#define MODEL_B_ANGLE_YAW_PID {2.5f, 0.0f, 0.0f}
#define MODEL_B_RATE_ROLL_PID {0.2f, 0.0f, 0.0f}
#define MODEL_B_RATE_PITCH_PID {0.2f, 0.0f, 0.0f}
#define MODEL_B_RATE_YAW_PID {0.2f, 0.0f, 0.0f}
#define MODEL_B_GYRO_LPF_MIN_HZ 30.0f
#define MODEL_B_GYRO_LPF_MAX_HZ 80.0f

/* 样机 C：在这一组修改其三轴外环和内环参数。 */
#define MODEL_C_ANGLE_ROLL_PID {8.0f, 0.0f, 0.0f}
#define MODEL_C_ANGLE_PITCH_PID {1.0f, 0.0f, 0.0f}
#define MODEL_C_ANGLE_YAW_PID {1.0f, 0.0f, 0.0f}
#define MODEL_C_RATE_ROLL_PID {0.2f, 0.0f, 0.0f}
#define MODEL_C_RATE_PITCH_PID {0.2f, 0.0f, 0.0f}
#define MODEL_C_RATE_YAW_PID {0.2f, 0.0f, 0.0f}
#define MODEL_C_GYRO_LPF_MIN_HZ 30.0f
#define MODEL_C_GYRO_LPF_MAX_HZ 80.0f

#if AIRCRAFT_MODEL == AIRCRAFT_MODEL_A
#define CONTROL_ANGLE_ROLL_PID MODEL_A_ANGLE_ROLL_PID
#define CONTROL_ANGLE_PITCH_PID MODEL_A_ANGLE_PITCH_PID
#define CONTROL_ANGLE_YAW_PID MODEL_A_ANGLE_YAW_PID
#define CONTROL_RATE_ROLL_PID MODEL_A_RATE_ROLL_PID
#define CONTROL_RATE_PITCH_PID MODEL_A_RATE_PITCH_PID
#define CONTROL_RATE_YAW_PID MODEL_A_RATE_YAW_PID
#define CONTROL_GYRO_LPF_MIN_HZ MODEL_A_GYRO_LPF_MIN_HZ
#define CONTROL_GYRO_LPF_MAX_HZ MODEL_A_GYRO_LPF_MAX_HZ
#elif AIRCRAFT_MODEL == AIRCRAFT_MODEL_B
#define CONTROL_ANGLE_ROLL_PID MODEL_B_ANGLE_ROLL_PID
#define CONTROL_ANGLE_PITCH_PID MODEL_B_ANGLE_PITCH_PID
#define CONTROL_ANGLE_YAW_PID MODEL_B_ANGLE_YAW_PID
#define CONTROL_RATE_ROLL_PID MODEL_B_RATE_ROLL_PID
#define CONTROL_RATE_PITCH_PID MODEL_B_RATE_PITCH_PID
#define CONTROL_RATE_YAW_PID MODEL_B_RATE_YAW_PID
#define CONTROL_GYRO_LPF_MIN_HZ MODEL_B_GYRO_LPF_MIN_HZ
#define CONTROL_GYRO_LPF_MAX_HZ MODEL_B_GYRO_LPF_MAX_HZ
#elif AIRCRAFT_MODEL == AIRCRAFT_MODEL_C
#define CONTROL_ANGLE_ROLL_PID MODEL_C_ANGLE_ROLL_PID
#define CONTROL_ANGLE_PITCH_PID MODEL_C_ANGLE_PITCH_PID
#define CONTROL_ANGLE_YAW_PID MODEL_C_ANGLE_YAW_PID
#define CONTROL_RATE_ROLL_PID MODEL_C_RATE_ROLL_PID
#define CONTROL_RATE_PITCH_PID MODEL_C_RATE_PITCH_PID
#define CONTROL_RATE_YAW_PID MODEL_C_RATE_YAW_PID
#define CONTROL_GYRO_LPF_MIN_HZ MODEL_C_GYRO_LPF_MIN_HZ
#define CONTROL_GYRO_LPF_MAX_HZ MODEL_C_GYRO_LPF_MAX_HZ
#endif

/* 二阶 Butterworth 只过滤角速度内环反馈，不过滤目标或 PID 输出。
 * 动态模式：截止频率随基础油门从 MIN 线性增加到 MAX，避免每周期跳变。
 * 这是油门调度，不是 FFT/频谱自适应；具体范围需按扑翼频率与响应延迟整定。
 * 关闭动态模式后使用 FIXED_HZ；ENABLE=0 可旁路滤波做对照。
 */
#ifndef CONTROL_GYRO_LPF_ENABLE
#define CONTROL_GYRO_LPF_ENABLE 1
#endif
#ifndef CONTROL_GYRO_LPF_DYNAMIC_ENABLE
#define CONTROL_GYRO_LPF_DYNAMIC_ENABLE 1
#endif
#define CONTROL_GYRO_LPF_FIXED_HZ 30.0f
#define CONTROL_GYRO_LPF_UPDATE_S 0.01f
#define CONTROL_GYRO_LPF_TAU_S 0.05f

/* 遥控前馈：Kff × Δ目标角 / Δ遥控帧时间，叠加到外环角速度目标。
 * 重复读取同一帧不再求差分；首帧/模式切换不产生前馈冲击。
 * 20ms 一阶平滑和角速度限幅避免遥控量化产生单周期尖峰。 */
#define CONTROL_RC_FF_ENABLE 1
#define CONTROL_RC_FF_GAIN 0.5f
#define CONTROL_RC_FF_MAX_DPS 100.0f
#define CONTROL_RC_FF_TAU_S 0.02f

/*
 * 上电零偏(零飘)校正开关:
 *   定义   → 本次启动跑一次静止采样实测标定,结果存 g_imu_bias(板子系,与放置无关)。
 *            ⚠️ 期间板子必须【静止且水平】约 1 秒。debug 读出 g_imu_bias 抄进
 *               imu_calib.c 的 s_stored_bias,之后就可关掉本开关用写死值。
 *   不定义 → 用 imu_calib.c 里按芯片写死的零偏(s_stored_bias)。
 */
#define IMU_CALIB_ON_BOOT

#define IMU_CALIB_SAMPLES   500U   /* 实测标定采样帧数(~2ms/帧 ≈ 1s) */

#endif /* BSP_CONFIG_H */
