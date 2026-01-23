/**
  ********************************************************
  * @file       mavlink.c/h
  * @brief      遥控器数据处理，解析elrs的遥控器数据，通过串口的空闲中断来处理数据，使用DMA
  *             来缓解CPU压力。
  *             后续优化的方向：
  *             1.增加DMA的断线重启处理，串口中断掉线处理来保证热插拔的稳定性
  *             2.使用DMA双缓存区来保证数据处理不会拥堵出错，目前单缓存测试正常（出问题可以关注这里）
  *             3.第一版使用的遥控器是RADIOMASTER POCKET,下面注释中通道与遥感不一定对应，后续使用者自己
	*             确认标定
	*             
	*             1.0.1版本更新：解决了之前遥控器需要软复位才能使用的问题，解决方案是在GPIO以及UART和DMA初始化之前
	*             手动加了一个5秒延时，不合理但有效，猜测是因为在初始化串口时不正常数据进入导致整个串口挂掉了
	*
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
#include "common/mavlink.h"
#include "mavlink_types.h"
#include "usart.h"
#include "ringbuffer.h"//环形缓冲区头文件
#include "stdio.h"
#define  MAVLINK_READ_BUFFER_SIZE 128
#include "cmsis_os.h"
#include "string.h"
//#include "mavlink_msg_rc_channels.h"
/*缓冲区管理器*/
//ringbuffer管理变量
RingBuffer  m_Mavlink_RX_RingBuffMgr;
/*MAVLINK接收数据缓冲区数组*/
uint8_t   m_Mavlink_RX_Buff[MAVLINK_READ_BUFFER_SIZE*10];
uint8_t mavlink_byte ;
mavlink_rc_channels_override_t packet;//遥控器数据结构体
mavlink_set_attitude_target_t packet2;//姿态数据结构体
mavlink_message_t lastmsg;
ELRS_Data elrs_data;
PC_Data_t pc_data;

/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const ELRS_Data *get_remote_control_point(void)
{
    return &elrs_data;
}
/**
  * @brief          获取上位机数据指针
  * @param[in]      none
  * @retval         上位机数据指针
  */
const PC_Data_t *get_pc_data_point(void)
{
    return &pc_data;
}

float float_Map(float input_value, float input_min, float input_max, float output_min, float output_max)
{
    float output_value;
    if (input_value < input_min)
    {
        output_value = output_min;
    }
    else if (input_value > input_max)
    {
        output_value = output_max;
    }
    else
    {
        output_value = output_min + (input_value - input_min) * (output_max - output_min) / (input_max - input_min);
    }
    return output_value;
}
float float_Map_with_median(float input_value, float input_min, float input_max, float median, float output_min, float output_max)
{
    float output_median = (output_max - output_min) / 2 + output_min;
    if (input_min >= input_max || output_min >= output_max || median <= input_min || median >= input_max)
    {
        return output_min;
    }

    if (input_value < median)
    {
        return float_Map(input_value, input_min, median, output_min, output_median);
    }
    else
    {
        return float_Map(input_value, median, input_max, output_median, output_max);
    }
}
/*******************************函数声明****************************************
* 函数名称: void Quaternion_to_Euler_Transform(void)
* 输入参数:
* 返回参数:
* 功    能: 将上位机发来的四元数转换为欧拉角，同时将信息传入结构体
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
void Quaternion_to_Euler_Transform(mavlink_set_attitude_target_t* packet,PC_Data_t* pc_data)
{ 
    float siny_cosp=0;
    float cosy_cosp=0;
    float sinp=0;
    float cosp=0;
    siny_cosp = 2.0f * (packet->q[0] * packet->q[3] + packet->q[1] * packet->q[2]);
    cosy_cosp = 1.0f - 2.0f * (packet->q[2] * packet->q[2] + packet->q[3] * packet->q[3]);
    pc_data->yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);
    sinp = 2.0f * (packet->q[0] * packet->q[1] + packet->q[2] * packet->q[3]); // 这是标准Roll(x)的公式
    cosp = 1.0f - 2.0f * (packet->q[1] * packet->q[1] + packet->q[2] * packet->q[2]);
    pc_data->pitch = atan2f(sinp, cosp) * (180.0f / M_PI);  
    sinp = 2.0f * (packet->q[0] * packet->q[2] - packet->q[3] * packet->q[1]); // 这是标准Pitch(y)的公式
    if (fabsf(sinp) >= 1.0f) 
{
     pc_data->roll = copysignf(M_PI / 2.0f, sinp) * (180.0f / M_PI); 
} 
    else 
{
     pc_data->roll = asinf(sinp) * (180.0f / M_PI);
}
   pc_data->thrust = packet->thrust;//传入油门
}

/*******************************函数声明****************************************
* 函数名称: void Mavlink_RB_Init(void)
* 输入参数:
* 返回参数:
* 功    能: 初始化一个循环队列，用来管理接收到的串口数据。其实就是一个数据缓冲区
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlink_RB_Init(void)
{
	//将m_Mavlink_RX_Buffm_Mavlink_RX_RingBuffMgr环队列进行关联管理。
	rbInit(&m_Mavlink_RX_RingBuffMgr, m_Mavlink_RX_Buff, sizeof(m_Mavlink_RX_Buff));
}

/*******************************函数声明****************************************
* 函数名称: void Mavlink_RB_Clear(void)
* 输入参数:
* 返回参数:
* 功    能: 归零ringbuffer里面的设置。
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlink_RB_Clear(void)
{
	rbClear(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: uint8_t  Mavlink_RB_IsOverFlow(void)
* 输入参数:
* 返回参数:  溢出为1，反之为0
* 功    能: 判断缓冲器是否溢出
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
uint8_t  Mavlink_RB_IsOverFlow(void)
{
	return m_Mavlink_RX_RingBuffMgr.flagOverflow;
}

/*******************************函数声明****************************************
* 函数名称: void Mavlink_RB_Push(uint8_t data)
* 输入参数: data：待压入的数据
* 返回参数:
* 功    能: 将接收的数据压入缓冲区
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlink_RB_Push(uint8_t data)
{
	rbPush(&m_Mavlink_RX_RingBuffMgr, (uint8_t)(data & (uint8_t)0xFFU));
}

/*******************************函数声明****************************************
* 函数名称: uint8_t Mavlink_RB_Pop(void)
* 输入参数:
* 返回参数: uint8_t 读出的数据
* 功    能: 从缓冲器读出数据
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
uint8_t Mavlink_RB_Pop(void)
{
	return rbPop(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: uint8_t Mavlink_RB_HasNew(void)
* 输入参数:
* 返回参数:
* 功    能: 判断是否有新的数据
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
uint8_t Mavlink_RB_HasNew(void)
{
	return !rbIsEmpty(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: uint16_t Mavlink_RB_Count(void)
* 输入参数:
* 返回参数:
* 功    能: 判断有多少个新数据
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
uint16_t Mavlink_RB_Count(void)
{
	return rbGetCount(&m_Mavlink_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: void Mavlink_Rece_Enable(void)
* 输入参数:
* 返回参数:
* 功    能: 使能DMA接收
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlink_Rece_Enable(void)
{
	HAL_UART_Receive_IT(&huart3, &mavlink_byte, 1);
}
/*******************************函数声明****************************************
* 函数名称: void Mavlink_Init(void)
* 输入参数:
* 返回参数:
* 功    能: 初始化MAVLINK：使能接收，ringbuffer关联
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlink_Init(void)
{
	Mavlink_Rece_Enable();
	Mavlink_RB_Init();
}
/*******************************函数声明****************************************
* 函数名称: void Mavlin_RX_InterruptHandler(void)
* 输入参数:
* 返回参数:
* 功    能: 串口中断的处理函数，主要是讲数据压入ringbuffer管理器
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlin_RX_InterruptHandler(void)
{
	//数据压入
	rbPush(&m_Mavlink_RX_RingBuffMgr,mavlink_byte);
}
/*在“mavlink_helpers.h中需要使用”*/
mavlink_system_t mavlink_system =
{
	1,
	1
}; // System ID, 1-255, Component/Subsystem ID, 1-255
/*******************************函数声明****************************************
* 函数名称: void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
* 输入参数:
* 返回参数:
* 功    能: 重新定义mavlink的发送函数，与底层硬件接口关联起来
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length)
{
	HAL_UART_Transmit(&huart3, (uint8_t *)ch, length, 2000);
}
/*******************************函数声明****************************************
* 函数名称: void Mavlink_Msg_Handle(void)
* 输入参数:
* 返回参数:
* 功    能: 处理从QGC上位机传过来的数据信息
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
void Mavlink_Msg_Handle(mavlink_message_t msg)
{
	switch(msg.msgid)
	{
	case MAVLINK_MSG_ID_HEARTBEAT:
		//printf("this is heartbeat from QGC/r/n");
		break;
	case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        mavlink_msg_set_attitude_target_decode(&msg, &packet2);//解析数据
        Quaternion_to_Euler_Transform(&packet2,&pc_data);//处理传入数据
		break;
	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
	{
		mavlink_msg_rc_channels_override_decode(&msg, &packet);
	elrs_data.channels[0] = packet.chan1_raw; // Roll (通常)
    elrs_data.channels[1] = packet.chan2_raw; // Pitch (通常)
    elrs_data.channels[2] = packet.chan3_raw; // Throttle (通常)
    elrs_data.channels[3] = packet.chan4_raw; // Yaw (通常)
    elrs_data.channels[4] = packet.chan5_raw; // Aux1
    elrs_data.channels[5] = packet.chan6_raw; // Aux2
    elrs_data.channels[6] = packet.chan7_raw; // Aux3
    elrs_data.channels[7] = packet.chan8_raw; // Aux4
    elrs_data.channels[8]  = packet.chan9_raw;
    elrs_data.channels[9]  = packet.chan10_raw;
    elrs_data.channels[10] = packet.chan11_raw;
    elrs_data.channels[11] = packet.chan12_raw;
    elrs_data.channels[12] = packet.chan13_raw;
    elrs_data.channels[13] = packet.chan14_raw;
    elrs_data.channels[14] = packet.chan15_raw;
    elrs_data.channels[15] = packet.chan16_raw;
	elrs_data.Left_X = float_Map_with_median(elrs_data.channels[3], 1000, 2000, 1500, -100, 100);
    elrs_data.Left_Y = float_Map_with_median(elrs_data.channels[2], 1000, 2000, 1500, 0, 100);
    elrs_data.Right_X = float_Map_with_median(elrs_data.channels[0], 1000, 2000, 1500, -100, 100);
    elrs_data.Right_Y = float_Map_with_median(elrs_data.channels[1], 1000, 2000, 1500, -100, 100);
    elrs_data.S1 = float_Map_with_median(elrs_data.channels[8], 1000, 2000, 1500, 0, 100);
    elrs_data.S2 = float_Map_with_median(elrs_data.channels[9], 1000, 2000, 1500, 0, 100);
    elrs_data.A = elrs_data.channels[10] > 1500 ? 1 : 0;
    elrs_data.B = (elrs_data.channels[5] < 1200) ? 0 : ((elrs_data.channels[5] > 1800) ? 2 : 1);
    elrs_data.C = (elrs_data.channels[6] < 1200) ? 0 : ((elrs_data.channels[6] > 1800) ? 2 : 1);
    elrs_data.D = elrs_data.channels[11] > 1500 ? 1 : 0;
    elrs_data.E = (elrs_data.channels[4] < 1200) ? 1 : ((elrs_data.channels[4] > 1800) ? 2 : 0);
    elrs_data.F = (elrs_data.channels[7] < 1200) ? 1 : ((elrs_data.channels[7] > 1800) ? 2 : 0);
	}
	break;
	default:
		break;
	}
}
/*******************************函数声明****************************************
* 函数名称: Loop_Mavlink_Parse(void)
* 输入参数:
* 返回参数:
* 功    能: 在main函数中不间断调用此函数
* 作    者: by Across
* 日    期: 2018/06/02
*******************************************************************************/
mavlink_message_t msg;
mavlink_status_t status;

void Loop_Mavlink_Parse(void)
{
	if(Mavlink_RB_IsOverFlow())
	{
		Mavlink_RB_Clear();
	}
    static uint16_t last_drop_count = 0;
	while(Mavlink_RB_HasNew())
	{
		uint8_t read = Mavlink_RB_Pop();
		uint8_t result = mavlink_parse_char(MAVLINK_COMM_0, read, &msg, &status);
		if (status.packet_rx_drop_count > last_drop_count)
        {
			last_drop_count = status.packet_rx_drop_count; // 更新计数器
        }
		if(result)
        {
            // 信号处理函数
            Mavlink_Msg_Handle(msg);
        }
		
	}
}
static void mavlink_test_heartbeat2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
	if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_HEARTBEAT >= 256)
	{
		return;
	}
#endif
	mavlink_message_t msg;
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	uint16_t i;
	mavlink_heartbeat_t packet_in =
	{
		963497464,17,84,151,218,3
	};
	mavlink_heartbeat_t packet1, packet2;
	memset(&packet1, 0, sizeof(packet1));
	packet1.custom_mode = packet_in.custom_mode;
	packet1.type = packet_in.type;
	packet1.autopilot = packet_in.autopilot;
	packet1.base_mode = packet_in.base_mode;
	packet1.system_status = packet_in.system_status;
	packet1.mavlink_version = packet_in.mavlink_version;


#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)
	{
		// cope with extensions
		memset(MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN);
	}
#endif
	memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0 , packet1.type , packet1.autopilot , packet1.base_mode , packet1.custom_mode , packet1.system_status );
	MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart == &huart3)
	{
		Mavlin_RX_InterruptHandler();
		HAL_UART_Receive_IT(&huart3, &mavlink_byte, 1);//重启中断
	}

}
void mavlink_task(void const *pvParameters)
{
	
	 while (1)
    {
      Loop_Mavlink_Parse();
	  mavlink_test_heartbeat2(1,1,&lastmsg);
	  vTaskDelay(10);
	}
}