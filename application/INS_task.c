/**
 * 统一 IMU 快照 -> Madgwick -> 原子发布姿态/角速度。
 * vTaskDelayUntil 按 1ms 调度，积分时间步长使用 DWT 实测值。
 * 初始化、标定未完成或采样失效时不发布有效姿态。
 */
#include "INS_task.h"
#include "cmsis_os.h"
#include "bsp_platform.h"
#include "task.h"
#include <math.h>
#include "bmi088_mm.h"
#include "common.h"
#include "string.h"
#include "main.h"
#include "pid.h"
#include "struct_typedef.h"
#include "spl06.h"
#include "imu_api.h"        // 统一 IMU 抽象层:imu_init/imu_start/imu_get_data + ImuData
#include "bsp_config.h"     // IMU_CALIB_ON_BOOT / IMU_CALIB_SAMPLES
#include "imu_calib.h"      // imu_calib_run / g_imu_bias
#define GRAVITY_EARTH  (9.80665f)

/* ── 调试观测(调试器 watch)── */
volatile uint8_t  g_imu_init_ok     = 0;  /* imu_init 成功=1 */
volatile uint32_t g_imu_loop_count  = 0;  /* INS_task 主循环计数 */
volatile ImuData  g_imu_live        = {0};/* 最新一帧统一数据:gyro rad/s, accel m/s² */

IMU_data_t imu_data;//IMU数据结构体实例化
SPL06_t spl06_inst;//气压计实例化
extern I2C_HandleTypeDef hi2c2; // 引用 CubeMX 生成的 I2C 句柄
float accgyroval[6];
int test_speed=0;
uint32_t current_time=0;
uint32_t prev_time =0;
float dt =0;
float timing_time =0.001;
static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
fp32 angle[3] = {0.0f, 0.0f, 0.0f}; 
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
volatile float beta = 0.1f;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
 float g_roll = 0.0f;  // 物理横滚角 (绕X轴, 左)
 float g_pitch = 0.0f; // 物理俯仰角 (绕Y轴, 前)
 float g_yaw = 0.0f;   // 物理偏航角 (绕Z轴, 下) (!!! 这个会漂移 !!!)
/*********************************************************************/
/*                       Function Declarations                       */
/*********************************************************************/
// 快速反平方根
float InvSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}
/* Madgwick 六轴姿态融合
 * 陀螺仪提供快速角运动，归一化加速度提供重力方向；梯度下降修正项由 beta 调节。
 * 四元数按实测 deltat 积分，最后重新归一化，避免积分造成模长漂移。
 * 没有磁力计/外部航向修正，Yaw 不是绝对航向，长期会漂移。 */
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float deltat) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // deltat 由任务实测时间间隔传入。
    // 四元数导数 (来自陀螺仪)
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // 仅在加速度有效时才进行修正 (防止除以0)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 归一化加速度计读数
        recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;   

        // 辅助变量
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // 梯度下降算法 (目标函数)，基于重力向量 [0, 0, 1]
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        // 归一化梯度
        recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // 应用修正 (用梯度减去陀螺仪漂移)
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // 积分四元数导数
    q0 += qDot1 * deltat;
    q1 += qDot2 * deltat;
    q2 += qDot3 * deltat;
    q3 += qDot4 * deltat;

    // 归一化四元数
    recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
void attitude_update_madgwick(const float* gyro_rads, const float* accel_mps2, float dt)
{
    /* 统一接口已是机体系 rad/s、m/s²(传感器→机体映射在 imu_frame.c,板子放置在
     * imu_orientation.h,均已标定),直接喂 AHRS。机体系:X前/Y右/Z下。 */
    MadgwickAHRSupdateIMU(gyro_rads[0], gyro_rads[1], gyro_rads[2],
                          accel_mps2[0], accel_mps2[1], accel_mps2[2], dt);

   

  
    // Roll (横滚角，绕 X 轴旋转)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    g_roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

    // Pitch (俯仰角，绕 Y 轴旋转)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        // 使用 copysign 处理 90 度死锁情况
        g_pitch = copysignf(M_PI / 2.0f, sinp) * (180.0f / M_PI);
    } else {
        g_pitch = asinf(sinp) * (180.0f / M_PI);
    }

    // Yaw (偏航角，绕 Z 轴旋转)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    g_yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);
}
/* 姿态任务与控制任务解耦
 * 写方先填局部 next，临界区内整份发布；读方同样在短临界区内整份复制。
 * 控制任务得到同一轮的欧拉角、角速度、有效标记和时间，避免混合两轮数据。 */
bool ins_get_snapshot(IMU_data_t *out)
{
    if (out == NULL) return false;
    taskENTER_CRITICAL();
    *out = imu_data;
    taskEXIT_CRITICAL();
    return out->IMU_init_flag;
}

const IMU_data_t*get_imu_data_point(void)
{
	return &imu_data;
}
/* 注:BMI088 的初始化/量程换算/零偏/中断配置已全部迁入 bsp/imu/bmi088 驱动。
 *     ICM-42688-P 在 bsp/imu/icm42688p 驱动。INS_task 只经统一接口取数。 */
/*********************************************************************/
/*                          Functions                                */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initializes the bmi08 sensor
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
void IMU_data_init(IMU_data_t *imu_data_init)
{
	//初始化姿态解算相关变量
	memset(imu_data_init->accel_offset,0,sizeof(imu_data_init->accel_offset));
	memset(imu_data_init->gyro_offset,0,sizeof(imu_data_init->gyro_offset));
	memset(imu_data_init->INS_gyro,0,sizeof(imu_data_init->INS_gyro));
	memset(imu_data_init->INS_accel,0,sizeof(imu_data_init->INS_accel));
	memset(imu_data_init->accel_offset,0,sizeof(imu_data_init->accel_offset));
	memset(imu_data_init->INS_angle_gyro,0,sizeof(imu_data_init->INS_angle_gyro));
	memset(imu_data_init->INS_angle_accel,0,sizeof(imu_data_init->INS_angle_accel));
	imu_data_init->alpha=0.98;
	imu_data_init->IMU_init_flag=false;
}
/* IMU -> 1kHz 姿态输出
 * 自动选型、启动采集、上电标定完成后才进入周期计算，INS 不按芯片分支。
 * 42688 为 1kHz、45686 为 800Hz；1kHz INS 在无新帧的周期保持上一帧有效测量。
 * 原始 IMU 超过 10ms/非有限数时撤销姿态有效标记，飞控状态机据此停机。
 * .INS_angle 按 yaw/pitch/roll 发布，单位是度，陀螺仪仍按 xyz rad/s 发布。 */
void INS_task(void const *pvParameters)
{
	  vTaskDelay(1000);

	  /* 统一抽象层:自动探测并按 45686 > 42688 > BMI088 选择，然后启动采集。 */
	  g_imu_init_ok = imu_init() ? 1U : 0U;
	  imu_start();

#if defined(IMU_CALIB_ON_BOOT)
	  /* 实测零偏标定(需板子静止且水平,约 1s);结果存 g_imu_bias,
	   * debug 时抄进 imu_calib.c 的写死表 */
	  if (g_imu_init_ok) {
	      imu_calib_run(IMU_CALIB_SAMPLES);
	  }
#else
	  /* 不实测:用 imu_calib.c 里按芯片写死的零偏 */
	  imu_calib_load_stored();
#endif

	  SPL06_Init(&spl06_inst, &hi2c2);
	  IMU_data_init(&imu_data);
	  uint8_t baro_divider = 0;
      const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
      configASSERT(period > 0);
      TickType_t wake = xTaskGetTickCount();
      uint32_t previous_us = (uint32_t)bsp_time_us();
      bool first = true;

	  while (1)
    {
      baro_divider++;
        if (baro_divider >= 20)
        {
            baro_divider = 0;

            // [A] 处理上一轮的数据 (如果有)
            if (spl06_inst.data_ready) {
                // 这里进行浮点解算，耗时在任务里，不影响中断
                SPL06_Process_Data(&spl06_inst);

                // --- 此时 spl06_inst.altitude 是最新的高度 ---
                // 你可以将它赋值给你的 INS 数据结构
                // imu_data.baro_alt = spl06_inst.altitude;
                // imu_data.baro_pressure = spl06_inst.pressure;
            }

            // [B] 发起新一轮 I2C 中断读取（历史函数名保留 DMA）
            SPL06_Start_Read_DMA(&spl06_inst);
        }

      /* ── 取统一数据(机体系 rad/s、m/s²,与具体芯片解耦)── */
      ImuData sample;
      bool valid = imu_read(&sample);
      uint32_t now_us = (uint32_t)bsp_time_us();
      float dt = first ? CONTROL_PERIOD_MS * 0.001f : (uint32_t)(now_us - previous_us) * 1.0e-6f;
      first = false;
      previous_us = now_us;
      if (dt <= 0.0f || dt > 0.02f) dt = CONTROL_PERIOD_MS * 0.001f;
      valid = valid && g_imu_status.running &&
          (uint32_t)(now_us - sample.timestamp) <= CONTROL_IMU_TIMEOUT_MS * 1000U;
      for (int i = 0; i < 3; ++i) {
          valid = valid && isfinite(sample.gyro[i]) && isfinite(sample.accel[i]);
      }
      if (!valid) {
          taskENTER_CRITICAL();
          imu_data.IMU_init_flag = false;
          taskEXIT_CRITICAL();
          vTaskDelayUntil(&wake, period);
          continue;
      }
      IMU_data_t next = imu_data;
      const ImuData *d = &sample;
      g_imu_live = *d;   /* 调试器观测 */

      for (int i = 0; i < 3; ++i) {
          INS_gyro[i]          = d->gyro[i];   /* rad/s */
          INS_accel[i]         = d->accel[i];  /* m/s²  */
          next.INS_gyro[i] = d->gyro[i];
          next.INS_accel[i]= d->accel[i];
      }

      attitude_update_madgwick(INS_gyro, INS_accel, dt);
	  next.INS_angle[0]=g_yaw;
	  next.INS_angle[1]=g_pitch;
	  next.INS_angle[2]=g_roll;

      next.update_ms = HAL_GetTick();
      next.IMU_init_flag = isfinite(g_yaw) && isfinite(g_pitch) && isfinite(g_roll);
      taskENTER_CRITICAL();
      imu_data = next;
      taskEXIT_CRITICAL();
      g_imu_loop_count++;
		vTaskDelayUntil(&wake, period);
	}

}
