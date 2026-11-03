#ifndef __AIRCRAFT_CTRL_TASK_H__
#define __AIRCRAFT_CTRL_TASK_H__
#include "INS_task.h"
#include "aircraft_mixer.h"
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
    uint32_t remote_frame_count;
    uint32_t pc_update_ms;
    uint8_t pc_valid;
    IMU_data_t imu; /* 本控制周期的完整姿态与角速度快照 */
	aircraft_behaviour_e aircraft_behaviour; //飞行器状态机初始化
  aircraft_behaviour_e last_aircraft_behaviour; //上一次飞行器状态机
    AircraftMixOutput actuator_output; /* 所选样机的电机/舵机 PWM，调试器可观察 */
	float yaw_set_angle;//yaw期望角度
} aircraft_control_t;
void Aircraft_ctrl_task(void const *pvParameters);


#endif
