/**
  ********************************************************
  * @file       INS_TASK.c/h
  * @brief      IMU数据处理任务.板载有两个IMU，目前配置的是BMI088，SPI读取数据
  *             只做了数据接收，没有姿态解算和去零飘
  *             目前代码存在一些问题，加速度计解算角度异常，但陀螺仪正常，使用角度时使用
  *             INS_angle_gyro[2]这个数组里的姿态，因为没有加速度计辅助校准，角度积分从启动状态开始
  *             关于校准：https://gitee.com/Enhe1988/flight-commander参考这个代码的readme
  *             
	*             1.0.1版本更新：找到了加速度计出问题的原因，在数据转换时对重力加速度处理少了两个轴导致异常
	*             修复了这个问题
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.1     2025.10.16        张兆博          1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ********************************************************
  */
#include "INS_task.h"
#include "cmsis_os.h"
#include <math.h>
#include "bmi088_mm.h"
#include "common.h"
#include "string.h"
#include "main.h"
#include "pid.h"
#include "ahrs.h"
#include "spl06.h"
#define GRAVITY_EARTH  (9.80665f)
struct bmi08_sensor_data bmi08_accel;
struct bmi08_sensor_data bmi08_gyro;
struct bmi08_sensor_data bmi08_accel_offset;//校准使用数据实例化
struct bmi08_sensor_data bmi08_gyro_offset;
int8_t rslt;
struct bmi08_dev bmi08;
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
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
fp32 angle[3] = {0.0f, 0.0f, 0.0f}; 
#define SAMPLE_FREQ 1000.0f
#define M_PI 3.14159265358979323846f
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
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // 陀螺仪积分的时间步长
    float deltat = 1.0f / SAMPLE_FREQ;

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
void attitude_update_madgwick(float* INS_gyro, float* INS_accel)
{


   
    float gx_algo_rad = -INS_gyro[2] * (M_PI / 180.0f); // Algo X = User Y
    float gy_algo_rad = -INS_gyro[0] * (M_PI / 180.0f); // Algo Y = -User X
    float gz_algo_rad = INS_gyro[1] * (M_PI / 180.0f); // Algo Z = User Z

 
    float ax_algo = INS_accel[0]; // Algo X = User Y
    float ay_algo = INS_accel[1]; // Algo Y = -User X
    float az_algo = INS_accel[2]; // Algo Z = User Z

   
    MadgwickAHRSupdateIMU(gx_algo_rad, gy_algo_rad, gz_algo_rad, 
                          ax_algo, ay_algo, az_algo);

   

  
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
const IMU_data_t*get_imu_data_point(void)
{
	return &imu_data;
}
/*!
 * @brief This internal API is used to initialize the bmi08 sensor
 */
static void init_bmi08(struct bmi08_dev *bmi08dev);

/*!
 * @brief This internal API is used to configure accel and gyro data ready interrupts
 */
static void configure_accel_gyro_data_ready_interrupts(struct bmi08_dev *bmi08dev);

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Value in Meter Per second squared.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Value in Degree Per Second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
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
static void init_bmi08(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;

    /* Initialize bmi08 sensors (accel & gyro) *///读取加速度计和陀螺仪的芯片id
    if (bmi088_mma_init(bmi08dev) == BMI08_OK && bmi08g_init(bmi08dev) == BMI08_OK)
	
    {
//        printf("BMI08 initialization success!\n");
//        printf("Accel chip ID - 0x%x\n", bmi08dev->accel_chip_id);
//        printf("Gyro chip ID - 0x%x\n", bmi08dev->gyro_chip_id);

        /* Reset the accelerometer */
        rslt = bmi08a_soft_reset(bmi08dev);
			  
    }
    else
    {
			
//        printf("BMI08 initialization failure!\n");
        return;
    }
    
    if (rslt == BMI08_OK)
    {
        /* Set accel power mode */
        bmi08dev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        rslt = bmi08a_set_power_mode(bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
        rslt = bmi08g_set_power_mode(bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev->accel_cfg.odr = BMI08_ACCEL_ODR_800_HZ;//100
        bmi08dev->accel_cfg.range = BMI088_MM_ACCEL_RANGE_6G;

        bmi08dev->accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
        bmi08dev->accel_cfg.bw = BMI08_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(bmi08dev);

        if (rslt == BMI08_OK)
        {
            rslt = bmi088_mma_set_meas_conf(bmi08dev);
            bmi08_check_rslt("bmi088_mma_set_meas_conf", rslt);

            bmi08dev->gyro_cfg.odr = BMI08_GYRO_BW_116_ODR_1000_HZ;//32/100
            bmi08dev->gyro_cfg.range = BMI08_GYRO_RANGE_1000_DPS;//1000
            bmi08dev->gyro_cfg.bw = BMI08_GYRO_BW_32_ODR_100_HZ;//32/100
            bmi08dev->gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

            rslt = bmi08g_set_power_mode(bmi08dev);

            if (rslt == BMI08_OK)
            {
                rslt = bmi08g_set_meas_conf(bmi08dev);
                bmi08_check_rslt("bmi08g_set_meas_conf", rslt);
            }
        }
    }
}
static void configure_accel_gyro_data_ready_interrupts(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;
    struct bmi08_accel_int_channel_cfg accel_int_config;
    struct bmi08_gyro_int_channel_cfg gyro_int_config;

    /* Configure the Interrupt configurations for accel */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Set the interrupt configuration */
    rslt = bmi088_mma_set_int_config(&accel_int_config, BMI088_MM_ACCEL_DATA_RDY_INT, bmi08dev);

    if (rslt == BMI08_OK)
    {
        /* Configure the Interrupt configurations for gyro */
        gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
        gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

        rslt = bmi08g_set_int_config(&gyro_int_config, bmi08dev);
    }

    if (rslt != BMI08_OK)
    {
//        printf("Failure in interrupt configurations \n");
        return;
    }
}
/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}
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
void offset_input(IMU_data_t *imu_data_offset)
{  
	//这里写入的是校准之后的零飘
	imu_data_offset->accel_offset[0]=-154;//-301
	imu_data_offset->accel_offset[1]=-38.5;//-207
	imu_data_offset->accel_offset[2]=3.8;//-59
	
	imu_data_offset->gyro_offset[0]=11.5;//10.76
	imu_data_offset->gyro_offset[1]=-9.8;//3.83
	imu_data_offset->gyro_offset[2]=3.9;//1.47
}

void autooffset(IMU_data_t *imu_data_cail)
{
	//进行校准，陀螺仪用原始数据去零飘，加速度计减去5461.3，这个值根据带宽和量程计算得出，改变参数时需要改变这个值
	//校准时，确保IMU的z方向竖直向下，
	for(int i=0;i<1000;i++)
	{
  bmi088_mma_get_data(&bmi08_accel_offset, &bmi08);
	bmi08g_get_data(&bmi08_gyro_offset, &bmi08);
	imu_data_cail->accel_offset[0]+=bmi08_accel_offset.x;
	imu_data_cail->accel_offset[1]+=bmi08_accel_offset.y;
	imu_data_cail->accel_offset[2]+=bmi08_accel_offset.z;
	imu_data_cail->gyro_offset[0]+=bmi08_gyro_offset.x;
	imu_data_cail->gyro_offset[1]+=bmi08_gyro_offset.y;
	imu_data_cail->gyro_offset[2]+=bmi08_gyro_offset.z;
	HAL_Delay(10);
	}
	imu_data_cail->accel_offset[0]/=1000;
	imu_data_cail->accel_offset[1]/=1000;
	imu_data_cail->accel_offset[2]/=1000;
	imu_data_cail->gyro_offset[0]/=1000;
	imu_data_cail->gyro_offset[1]/=1000;
	imu_data_cail->gyro_offset[2]/=1000;
	imu_data_cail->accel_offset[2]-=5461.3;//这里需要减去一个重力加速度
}
void INS_task(void const *pvParameters)
{   
	  vTaskDelay(1000);
	  bmi08_interface_init(&bmi08, BMI08_SPI_INTF);//初始化bmi088为spi模式，同时提供stm32的spi和delay操作的api
	  init_bmi08(&bmi08);
      SPL06_Init(&spl06_inst, &hi2c2);
	  IMU_data_init(&imu_data);
	   //autooffset(&imu_data);//校准函数
	  offset_input(&imu_data);//如果新板子需要校准这里要用校准函数，不使用这个函数
      uint8_t baro_divider = 0;

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

            // [B] 发起新一轮 DMA 读取 (瞬间完成)
            SPL06_Start_Read_DMA(&spl06_inst);
        }
	  rslt = bmi088_mma_get_data(&bmi08_accel, &bmi08);
      bmi08_check_rslt("bmi088_mma_get_data", rslt);
	  imu_data.bmi08_accel_raw.x=bmi08_accel.x;//拷贝一份到自定义结构体里
	  imu_data.bmi08_accel_raw.y=bmi08_accel.y;
	  imu_data.bmi08_accel_raw.z=bmi08_accel.z;
      INS_accel[0] = lsb_to_mps2(bmi08_accel.x-imu_data.accel_offset[0], (float)6, 16);
      INS_accel[1] = lsb_to_mps2(bmi08_accel.y-imu_data.accel_offset[1], (float)6, 16);
      INS_accel[2] = lsb_to_mps2(bmi08_accel.z-imu_data.accel_offset[2], (float)6, 16);
	  imu_data.INS_accel[0]=lsb_to_mps2(bmi08_accel.x-imu_data.accel_offset[0], (float)6, 16);
	  imu_data.INS_accel[1]=lsb_to_mps2(bmi08_accel.y-imu_data.accel_offset[1], (float)6, 16);
	  imu_data.INS_accel[2]=lsb_to_mps2(bmi08_accel.z-imu_data.accel_offset[2], (float)6, 16);
      rslt = bmi08g_get_data(&bmi08_gyro, &bmi08);
      bmi08_check_rslt("bmi08g_get_data", rslt);
	  imu_data.bmi08_gyro_raw.x=bmi08_gyro.x;//拷贝一份到自定义结构体里
	  imu_data.bmi08_gyro_raw.y=bmi08_gyro.y;
	  imu_data.bmi08_gyro_raw.z=bmi08_gyro.z;
      INS_gyro[0] = lsb_to_dps((int16_t)(bmi08_gyro.x-imu_data.gyro_offset[0]), (float)1000, 16);
      INS_gyro[1] = lsb_to_dps((int16_t)(bmi08_gyro.y-imu_data.gyro_offset[1]), (float)1000, 16);
      INS_gyro[2] = lsb_to_dps((int16_t)(bmi08_gyro.z-imu_data.gyro_offset[2]), (float)1000, 16);
      imu_data.INS_gyro[0]=lsb_to_dps((int16_t)(bmi08_gyro.x-imu_data.gyro_offset[0]), (float)1000, 16);
	  imu_data.INS_gyro[1]=lsb_to_dps((int16_t)(bmi08_gyro.y-imu_data.gyro_offset[1]), (float)1000, 16);
	  imu_data.INS_gyro[2]=lsb_to_dps((int16_t)(bmi08_gyro.z-imu_data.gyro_offset[2]), (float)1000, 16);
      attitude_update_madgwick(INS_gyro, INS_accel);
	  imu_data.INS_angle[0]=g_yaw;
	  imu_data.INS_angle[1]=g_pitch;
	  imu_data.INS_angle[2]=g_roll;
     


		vTaskDelay(1);
	}
	
}