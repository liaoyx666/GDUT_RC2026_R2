#include "RC_dm4310.h"

namespace motor
{
	DM4310::DM4310(uint8_t id_, can::Can &can_, tim::Tim &tim_) : can::CanHandler(can_), tim::TimHandler(tim_), Motor(1.f)
	{
		id = id_;
		
		CanHandler_Register();
		
		pid_spd.Pid_Mode_Init(true, false, 0.4);
		pid_spd.Pid_Param_Init(40, 0.07, 0, 0, 0.001, 0, 16384, 10000, 5000, 5000, 5000);
		
		pid_pos.Pid_Mode_Init(false, false, 0.4);
		pid_pos.Pid_Param_Init(600, 2, 2, 0, 0.001, 0, 50, 25, 25 ,25 ,25);
	}
	
	void DM4310::CanHandler_Register()
	{
		if(id > 3)
		{
			tx_id = 0x4FE;
		}
		else
		{
			tx_id = 0x3FE;
		}
		
		rx_id = 0x300 + id;
		can_frame_type = can::FRAME_STD;
		rx_mask = 0xfff;
		
		if (can->tx_frame_num == 0)// can上还没有挂载帧
		{
			tx_frame_dx = 0;	//帧编号
			can->tx_frame_num = 1;// 第一个帧
			
			can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
			can->tx_frame_list[tx_frame_dx].id = tx_id;
			can->tx_frame_list[tx_frame_dx].dlc = 8;
			
			can->tx_frame_list[tx_frame_dx].hd_num = 1;	//登记挂载第一个
			can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;	//将设备总编号登记到此CAN帧的设备列表
		}
		else// can上已经有帧
		{
			bool have_same_tx_id = false;// 是否有同样帧id的帧
			
			// 查询是否有帧id相同的帧
			for (uint16_t i = 0; i < can->tx_frame_num; i++)
			{
				if (can->tx_frame_list[i].frame_type == can_frame_type && can->tx_frame_list[i].id == tx_id)// 帧种类和帧id相同
				{
					have_same_tx_id = true;
					tx_frame_dx = i;// 合并相同帧id的帧（索引相同）
					
					can->tx_frame_list[tx_frame_dx].hd_num++;
					
					if (can->tx_frame_list[tx_frame_dx].hd_num > 4)// 一个帧最多挂载4个设备
					{
						Error_Handler();
					}
					
					can->tx_frame_list[tx_frame_dx].hd_dx[can->tx_frame_list[tx_frame_dx].hd_num - 1] = hd_list_dx;
					break;
				}
			}
			
			// 无相同帧id的帧
			if (have_same_tx_id == false)
			{
				can->tx_frame_num++;// 帧总数加一
				tx_frame_dx = can->tx_frame_num - 1;	//登记帧的编号
				
				can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
				can->tx_frame_list[tx_frame_dx].id = tx_id;
				can->tx_frame_list[tx_frame_dx].dlc = 8;
		
				can->tx_frame_list[tx_frame_dx].hd_num = 1;
				can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;	//登记设备编号到此帧的设备列表
			}
		}
	}
	
	
	void DM4310::Can_Tx_Process()
	{
		uint16_t dx = ((id - 1) % 4) * 2;
	
		int16_t current_int;
		
		if (target_current >= 0)// 四舍五入
		{
			current_int = (int16_t)(target_current + 0.5f);
		}
		else
		{
			current_int = (int16_t)(target_current - 0.5f);
		};
		
		if (current_int > 16384)	//数据溢出
		{
			current_int = 16384;
		}
		else if (current_int < -16384)
		{
			current_int = -16384;
		}
				
		can->tx_frame_list[tx_frame_dx].data[dx] = (uint8_t)(current_int);         // 低8位
		can->tx_frame_list[tx_frame_dx].data[dx + 1] = (uint8_t)(current_int >> 8);// 高8位
		
	}
	
	
	void DM4310::Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data)
	{
		angle 		= ((float)(int16_t)(((uint16_t)rx_data[0] << 8) | (uint16_t)rx_data[1])) / 8192.f * TWO_PI;	//计算实际角度
		rpm 		= ((float)(int16_t)(((uint16_t)rx_data[2] << 8) | (uint16_t)rx_data[3])) / 100.f;	//计算实际转速
		current 	= (float)(int16_t)(((uint16_t)rx_data[4] << 8) | (uint16_t)rx_data[5]);	//计算扭矩电流
		temperature = (float)(int8_t)rx_data[6];	//线圈温度
		error_code 	= (uint8_t)rx_data[7];	//错误状态
		
		
		if (can_rx_is_first != true)	//若不为第一次上发
		{
			if (last_angle < HALF_PI && angle > TWO_THIRD_PI)
			{
				cycle--;
			}
			else if (last_angle > TWO_THIRD_PI && angle < HALF_PI)
			{
				cycle++;
			}
		}
		else can_rx_is_first = false;
		
		pos = cycle * TWO_PI + angle + pos_offset;
		out_pos = pos / gear_ratio;
		
		if (pos > 6434) pos = 6434;
		else if (pos < -6434) pos = -6434;
		
		last_angle = angle;
	}
	
	
	void DM4310::Tim_It_Process()
	{
		if (motor_mode != CURRENT_MODE)			//> 电流模式
		{
			float temp_target_rpm = 0;// 目标速度
			
			if (motor_mode == RPM_MODE)			//> 速度模式
			{
				temp_target_rpm = target_rpm;
			}
			else if (motor_mode == POS_MODE)	//> 位置模式
			{
				pid_pos.Update_Real(pos);
				pid_pos.Update_Target(target_pos);
				temp_target_rpm = pid_pos.Pid_Calculate();
			}
			else if (motor_mode == ANGLE_MODE)	//> 角度模式
			{
				pid_pos.Update_Real(angle);
				pid_pos.Update_Target(target_angle);
				temp_target_rpm = pid_pos.Pid_Calculate(true, PI);
			}
			
			pid_spd.Update_Target(temp_target_rpm);
			pid_spd.Update_Real(rpm);
			target_current = pid_spd.Pid_Calculate();
		}
	}
	
}


