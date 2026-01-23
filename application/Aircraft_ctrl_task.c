/**
  ********************************************************
  * @file       Aircraft_ctrl_task.c/h
  * @brief      飞行控制任务，主要思路是使用状态机去切换飞行模式，通过遥控的指令判断进入不同的
  *             飞行模式
  *             目前roll使用双环PID，pitch和yaw使用单环PID
  *             
  *             
  *             
	*             
	*             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.1     2026.1.10        张兆博          1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ********************************************************
  */
/***      《 飞行任务控制代码 》

 *      ┌─┐       ┌─┐ + +
 *   ┌──┘ ┴───────┘ ┴──┐++
 *   │                 │
 *   │       ───       │++ + + +
 *   ███████───███████ │+
 *   │                 │+
 *   │       ─┴─       │
 *   │                 │
 *   └───┐         ┌───┘
 *       │         │
 *       │         │   + +
 *       │         │
 *       │         └──────────────┐
 *       │                        │
 *       │                        ├─┐
 *       │                        ┌─┘
 *       │                        │
 *       └─┐  ┐  ┌───────┬──┐  ┌──┘  + + + +
 *         │ ─┤ ─┤       │ ─┤ ─┤
 *         └──┴──┘       └──┴──┘  + + + +
 *              
 *               代码无BUG!
 */
#include "Aircraft_ctrl_task.h"  // 飞行器控制任务头文件
#include "cmsis_os.h"            // CMSIS-RTOS操作系统头文件
#include "common/mavlink.h"      // 遥控器头文件
#include "tim.h"                  // 定时器头文件
#include "pid.h"                    // PID控制头文件
#include "INS_task.h"               // 惯性导航系统任务头文件
#include "filter.h"                //滤波器头文件
aircraft_control_t aircraft_control;  // 创建飞行器控制结构体实例
// 角度环PID控制器结构体
pid_type_def PID_control_pitch;  // 俯仰（PITCH）角度控制PID
pid_type_def PID_control_roll;   // 滚转（ROLL）角度控制PID
pid_type_def PID_control_yaw;    // 偏航（YAW）角度控制PID（未使用）

// 角速度环PID控制器结构体
pid_type_def PID_rate_pitch;      // 俯仰（PITCH）角速度控制PID
pid_type_def PID_rate_roll;       // 滚转（ROLL）角速度控制PID
// pid_type_def PID_rate_yaw;     // 偏航（YAW）角速度控制PID

//滤波器实例化
LPF2ndFilter gyro_pitch_lpf;
LPF2ndFilter gyro_roll_lpf;
LPF2ndFilter angle_pitch_lpf;
LPF2ndFilter angle_roll_lpf;
LPF2ndFilter acc_pitch_lpf;
LPF2ndFilter acc_roll_lpf;
/**************控制变量声明***************/

int32_t left_motor_pwm = 1000;     // 左电机PWM值
int32_t right_motor_pwm = 1000;    // 右电机PWM值

float pitch_servo_angle = 900.0f; // 俯仰舵机角度

float pitch_bias = 0.0f;          // 俯仰舵机偏置（零位补偿）

//* 电机差速控制相关参数*//
float base_throttle = 1000.0f;    // 基础油门
float diff_scale = 0.3f;          // 差速比例系数
float max_motor_diff = 200.0f;    // 最大差速值
float add_speed=0;
// 角度环PID参数数组
fp32 PID_Roll[3] = {5.0f, 0.0f, 0.0f};    // Roll角度环PID参数
fp32 PID_Pitch[3] = {2.0f, 0.0f, 0.0f};   // Pitch角度环PID参数                                                                                                       
fp32 PID_Yaw[3] = {2.0f, 0.0f, 0.0f};     // Yaw角度环PID参数


// 角速度环PID参数数组
fp32 PID_Rate_Roll[3] = {0.5f, 0.0f, 0.0f};   // Roll角速度环PID
fp32 PID_Rate_Pitch[3] = {0.5f, 0.0f, 0.0f};  // Pitch角速度环PID



/*****************************函数声明区*******************************/

void aircraft_init(aircraft_control_t*init);//初始化函数
void aricraft_behaviour_mode_set(aircraft_control_t*aircraft_mode_set);//模式切换判断函数
void aricraft_fly_ctrl(aircraft_control_t*aircraft_fly_ctrl);//飞行控制函数

/*****************************代码区***********************************/
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
void Aircraft_ctrl_task(void const *pvParameters)
{ // 初始化角度环PID控制器
	//括号里参数为：结构体、模式、增益数组、最大限幅、最小限幅、积分限幅
   PID_init(&PID_control_pitch, PID_POSITION, PID_Pitch, 1000, 10000);
   PID_init(&PID_control_roll, PID_POSITION, PID_Roll, 1000, 10000);
   PID_init(&PID_control_yaw, PID_POSITION, PID_Yaw, 1000, 10000);
	// 初始化角速度环PID控制器
	PID_init(&PID_rate_roll, PID_POSITION, PID_Rate_Roll, 1000, 500.0f);
	PID_init(&PID_rate_pitch, PID_POSITION, PID_Rate_Pitch, 300, 100.0f);
	LPF2nd_Init_30Hz(&gyro_pitch_lpf);
	LPF2nd_Init_30Hz(&gyro_roll_lpf);
	LPF2nd_Init_30Hz(&acc_pitch_lpf);
	LPF2nd_Init_30Hz(&acc_roll_lpf);
	LPF2nd_Init_30Hz(&angle_pitch_lpf);
	LPF2nd_Init_30Hz(&angle_roll_lpf);
	aircraft_init(&aircraft_control);                  // 初始化飞行器控制结构体
	
	
	// 主控制循环
	 while (1)
    {
      aricraft_behaviour_mode_set(&aircraft_control);// 设置飞行模式
			aricraft_fly_ctrl(&aircraft_control);          // 执行飞行控制
			vTaskDelay(1);
		}
}
/*****************************初始化函数*******************************/
void aircraft_init(aircraft_control_t*init)
{
	 init->aircraft_behaviour=Aircraft_ZERO_FORCE;      // 初始化为零力模式
	 init->last_aircraft_behaviour=Aircraft_ZERO_FORCE;  // 记录上一个模式
	 init->remote_data=get_remote_control_point();       // 获取遥控器数据指针
	 init->IMU_data=get_imu_data_point();                // 获取IMU数据指针
	 init->pc_data=get_pc_data_point();                  // 获取上位机数据指针
	 init->left_motor_out=0;
	 init->right_motor_out=0;
	 init->roll_motor_out=0;
	 init->Pitch_Rate_Set=0;
	 init->Roll_Rate_Set=0;
	 init->yaw_set_angle=0;
	
	 // 初始化电机和舵机为安全状态
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 1000); //电机停转
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 1000); //电机停转	
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1500); // 舵机回中
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1500); // 舵机回中
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 1500); // 舵机回中

}
	/*************************模式切换函数*************************/

void aricraft_behaviour_mode_set(aircraft_control_t*aircraft_mode_set)
{
	// 根据遥控器开关组合设置飞行模式
	/*
	使用方法举例，通过遥控器的不同开关状态让飞行器处于不同模式下
	这个判断条件也可以加上IMU，例如失控时转入Aircraft_ZERO_FORCE状态保护飞行器
	或者遥控器断联时让飞行器停止
	*/
	if(aircraft_mode_set->remote_data->B==0 && aircraft_mode_set->remote_data->C==0) 
 {
    aircraft_mode_set->aircraft_behaviour = Aircraft_ZERO_FORCE;
 }
	else if(aircraft_mode_set->remote_data->B==2 && aircraft_mode_set->remote_data->C==0)
 {
    aircraft_mode_set->aircraft_behaviour = Aircraft_RUN;//正常模式
 } 
	else if(aircraft_mode_set->remote_data->E==2 && aircraft_mode_set->remote_data->B==0&&aircraft_mode_set->remote_data->C==0) 
 {
    aircraft_mode_set->aircraft_behaviour = Aircraft_InRing_P;//yaw外环p
 } 
  else if(aircraft_mode_set->remote_data->E==2 && aircraft_mode_set->remote_data->B==2&&aircraft_mode_set->remote_data->C==2)
 {
    aircraft_mode_set->aircraft_behaviour = Aircraft_InRing_R;//pitch外环p
 } 
  else if(aircraft_mode_set->remote_data->E==2 && aircraft_mode_set->remote_data->B==0 && aircraft_mode_set->remote_data->C==2) 
 {
    aircraft_mode_set->aircraft_behaviour = Aircraft_OutRing_P;//roll 外环p
 } 
  else if(aircraft_mode_set->remote_data->B==2 && aircraft_mode_set->remote_data->C==2) 
 {
    aircraft_mode_set->aircraft_behaviour = Aircraft_Auto;//自动模式
 }
 else 
 {
    aircraft_mode_set->aircraft_behaviour = Aircraft_ZERO_FORCE;
 }
 //检测到模式切换，将yaw的期望角进行更新
 if(aircraft_mode_set->aircraft_behaviour==Aircraft_RUN&&aircraft_mode_set->last_aircraft_behaviour==Aircraft_ZERO_FORCE)
 {
	 aircraft_mode_set->yaw_set_angle=aircraft_mode_set->IMU_data->INS_angle[0];
 }
 aircraft_mode_set->last_aircraft_behaviour=aircraft_mode_set->aircraft_behaviour;
}

	/*************************电机舵机控制值限制函数*************************/
double num_limit(double num,double min, double max)
{
	if(num>max)
	{
		num=max;
	}
	if(num<min)
	{
		num=min;
	}
	return num;
}


	/*************************舵机偏置调整函数*************************/
void Bias(aircraft_control_t*aircraft_fly_ctrl)
{
    
    // 使用遥控器右摇杆调整Pitch1舵机偏置
    if (aircraft_fly_ctrl->remote_data->Right_Y  <-50) {
        pitch_bias -=0.2;  // 下推减小偏置
    } else if (aircraft_fly_ctrl->remote_data->Right_Y>50) {
        pitch_bias +=0.2;  // 上拉增加偏置
    }
    
    // 使用遥控器左摇杆调整基础油门偏置
    if (aircraft_fly_ctrl->remote_data->Left_X  <-50) {
        base_throttle -=0.2;  // 左摇减小偏置
    } else if (aircraft_fly_ctrl->remote_data->Left_X>50) {
        base_throttle +=0.2;  // 右摇增加偏置
    }
}

	/*************************串级PID控制函数*************************/




	/*************************主飞行控制函数*************************/
void aricraft_fly_ctrl(aircraft_control_t*aircraft_fly_ctrl)
{
	                   //****安全模式****//
	if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_ZERO_FORCE)
	{
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 1000); //电机停转
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 1000); //电机停转	
	    __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1500);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, 1500);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, 1500); 

	}
    
	
	                   //****Pitch外环****//
	else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_InRing_P)
	{
		// 如果遥控器F通道在位置2，用Left_Y调整Pitch角速度环Kp
		if(aircraft_fly_ctrl->remote_data->F==2)
		{
			PID_control_pitch.Kp=0.1*aircraft_fly_ctrl->remote_data->Left_Y;
		
		}
		//Bias(&aircraft_control);       // 更新偏置

	}
	          




	                    //****pitch内环调试模式****//
  else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_InRing_R)
	{
		if(aircraft_fly_ctrl->remote_data->F==2)
	{
		PID_rate_pitch.Kp=0.05*aircraft_fly_ctrl->remote_data->Left_Y;
	}

		//Bias(&aircraft_control);       // 更新偏置
	}




								//****ROLL 内环P调试模式****//
	else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_OutRing_P)
	{
		if(aircraft_fly_ctrl->remote_data->F==2)
	{
		PID_rate_roll.Kp = 0.05*aircraft_fly_ctrl->remote_data->Left_Y;
    
    
	}
 
		//Bias(&aircraft_control);       // 更新偏置
	}
	
	
	
	
	
	                   //****外环ROLL调试模式****//
	else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_OutRing_R)
	{

		if(aircraft_fly_ctrl->remote_data->F==2)
	{
		PID_control_roll.Kp =0.05*aircraft_fly_ctrl->remote_data->Left_Y;
    
    
	}	
		//Bias(&aircraft_control);       // 更新偏置
	}
	else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_Auto)
	{
		//自动模式下的处理逻辑
		//自动模式下，使用上位机数据进行控制
		yaw_PID_calc(&PID_control_yaw, aircraft_fly_ctrl->IMU_data->INS_angle[0], aircraft_fly_ctrl->pc_data->yaw);
		PID_calc(&PID_control_roll, aircraft_fly_ctrl->IMU_data->INS_angle[2], aircraft_fly_ctrl->pc_data->roll);
		PID_calc(&PID_control_pitch, aircraft_fly_ctrl->IMU_data->INS_angle[1], aircraft_fly_ctrl->pc_data->pitch);
		//roll输出限幅
	    aircraft_fly_ctrl->roll_motor_out=1500-PID_control_roll.out;
		aircraft_fly_ctrl->roll_motor_out=num_limit(aircraft_fly_ctrl->roll_motor_out,1100,1900);		
	    //roll舵机写值
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4,aircraft_fly_ctrl->roll_motor_out);	
		//pitch yaw 输出限幅与写值
		aircraft_fly_ctrl->left_motor_out=1500+PID_control_pitch.out+PID_control_yaw.out;
		 aircraft_fly_ctrl->right_motor_out=1500-PID_control_pitch.out+PID_control_yaw.out;
		aircraft_fly_ctrl->left_motor_out=num_limit(aircraft_fly_ctrl->left_motor_out,1100,1900);
		aircraft_fly_ctrl->right_motor_out=num_limit(aircraft_fly_ctrl->right_motor_out,1100,1900);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, aircraft_fly_ctrl->left_motor_out);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, aircraft_fly_ctrl->right_motor_out);
		//油门控制
		aircraft_fly_ctrl->power_motor_out = aircraft_fly_ctrl->pc_data->thrust*90;
		aircraft_fly_ctrl->power_motor_out = num_limit(aircraft_fly_ctrl->power_motor_out,0,900);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 1000+aircraft_fly_ctrl->power_motor_out);
	    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 1000+aircraft_fly_ctrl->power_motor_out);

	}
                   //****正常运行模式****//
	else if(aircraft_fly_ctrl->aircraft_behaviour==Aircraft_RUN)
	{
		//遥控死区限制
		int Right_X =0,Right_Y=0,Left_X=0;
		rc_deadband_limit(aircraft_fly_ctrl->remote_data->Right_X,Right_X,RC_DEADBAND);
		rc_deadband_limit(aircraft_fly_ctrl->remote_data->Right_Y,Right_Y,RC_DEADBAND);
		rc_deadband_limit(aircraft_fly_ctrl->remote_data->Left_X,Left_X,RC_DEADBAND);
		//yaw角度赋值
		aircraft_fly_ctrl->yaw_set_angle=aircraft_fly_ctrl->yaw_set_angle+Left_X*0.0003;
		//处理yaw角度在临界点跳变
		if(aircraft_fly_ctrl->yaw_set_angle>180)
		{
			aircraft_fly_ctrl->yaw_set_angle=aircraft_fly_ctrl->yaw_set_angle-360;
		}
		if(aircraft_fly_ctrl->yaw_set_angle<-180)
		{
			aircraft_fly_ctrl->yaw_set_angle=aircraft_fly_ctrl->yaw_set_angle+360;
		}
		//yaw角度环PID计算
		yaw_PID_calc(&PID_control_yaw, aircraft_fly_ctrl->IMU_data->INS_angle[0], aircraft_fly_ctrl->yaw_set_angle);
		// pitch roll串级PID控制
		// ROLL外环：遥控器Right_X/Y作为目标角度
	   aircraft_fly_ctrl->Roll_Rate_Set = PID_calc(&PID_control_roll, 
								aircraft_fly_ctrl->IMU_data->INS_angle[2], 
								Right_X*0.2+0);
		//PITCH外环
		aircraft_fly_ctrl->Pitch_Rate_Set = PID_calc(&PID_control_pitch, 
								 aircraft_fly_ctrl->IMU_data->INS_angle[1], 
								 Right_Y*0.2+0);

      //roll输出限幅
	    aircraft_fly_ctrl->roll_motor_out=1500-PID_control_roll.out;
		aircraft_fly_ctrl->roll_motor_out=num_limit(aircraft_fly_ctrl->roll_motor_out,1100,1900);		
	  //roll舵机写值
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4,aircraft_fly_ctrl->roll_motor_out);	
		// pitch和yaw相加限幅与输出
		// aircraft_fly_ctrl->left_motor_out=1500+PID_control_pitch.out+PID_control_yaw.out;
		// aircraft_fly_ctrl->right_motor_out=1500-PID_control_pitch.out+PID_control_yaw.out;
		 aircraft_fly_ctrl->left_motor_out=1500+PID_control_pitch.out;
		 aircraft_fly_ctrl->right_motor_out=1500-PID_control_pitch.out;
		aircraft_fly_ctrl->left_motor_out=num_limit(aircraft_fly_ctrl->left_motor_out,1100,1900);
		aircraft_fly_ctrl->right_motor_out=num_limit(aircraft_fly_ctrl->right_motor_out,1100,1900);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, aircraft_fly_ctrl->left_motor_out);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, aircraft_fly_ctrl->right_motor_out);

		//油门控制
		aircraft_fly_ctrl->power_motor_out = aircraft_fly_ctrl->remote_data->Left_Y*9;
		aircraft_fly_ctrl->power_motor_out = num_limit(aircraft_fly_ctrl->power_motor_out,0,900);
		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 1000+aircraft_fly_ctrl->power_motor_out);
	    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 1000+aircraft_fly_ctrl->power_motor_out);

	}
}
