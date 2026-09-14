#ifndef __AIRCRAFT_CTRL_TASK_H__
#define __AIRCRAFT_CTRL_TASK_H__
#include "INS_task.h"
#include "mavlink_rc.h"   /* ELRS_Data / PC_Data_t 及原子快照接口 */
#define RC_DEADBAND  2               //遥控器死区值
#define RC_LINK_TIMEOUT_MS 100U       //超过该时间未收到有效遥控帧则进入零力模式
typedef enum
{
	
  Aircraft_ZERO_FORCE = 0, 
  Aircraft_InRing_P,    
  Aircraft_InRing_R,	
	Aircraft_OutRing_P,    
  Aircraft_OutRing_R,
  Aircraft_Auto,  //自动运行
  Aircraft_RUN,   //正常运行        
} aircraft_behaviour_e;

typedef struct
{
    ELRS_Data remote_data;       //本控制周期使用的完整遥控快照
	PC_Data_t pc_data;            //本控制周期使用的上位机快照
	uint32_t remote_update_ms;
	uint8_t remote_valid;
	const IMU_data_t *IMU_data;//IMU数据实例化
	aircraft_behaviour_e aircraft_behaviour; //飞行器状态机初始化
  aircraft_behaviour_e last_aircraft_behaviour; //上一次飞行器状态机
  float Roll_Rate_Set;//期望roll角速度
	float Pitch_Rate_Set;//期望pitch角速度
    float roll_motor_out;//roll舵机输出
	float left_motor_out;//左舵机输出
	float right_motor_out;//右舵机输出
	float power_motor_out;//上升电机输出
	float yaw_set_angle;//yaw期望角度
} aircraft_control_t;
void Aircraft_ctrl_task(void const *pvParameters);


#endif
