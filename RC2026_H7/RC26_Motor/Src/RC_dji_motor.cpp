#include "RC_dji_motor.h"

namespace motor
{	
	DjiMotor::DjiMotor(can::Can &can_, tim::Tim *tim_, float gear_ratio_, bool is_reset_pos_angle)
	: can::CanHandler(can_), tim::TimHandler(tim_), Motor(gear_ratio_, is_reset_pos_angle)
	{
		is_gear_ratio_int = (fabsf(gear_ratio_ - roundf(gear_ratio_)) < 1e-6f);
		
		out_angle_max = 8192 * (int32_t)gear_ratio_; /* 输出轴一圈对应的转子编码器总数（周期长度），gear_ratio_需为正整数 */
	}
	
	void DjiMotor::CanHandler_Register()
	{
		if (can->hd_num > 8) Error_Handler();// 设备数量超过8
	
		can_frame_type = FDCAN_STANDARD_ID;// 标准帧
	
		if (can->tx_frame_num == 0)// can上还没有挂载帧
		{
			tx_frame_dx = 0;
			can->tx_frame_num = 1;// 第一个帧
			
			can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
			can->tx_frame_list[tx_frame_dx].id = tx_id;
			can->tx_frame_list[tx_frame_dx].dlc = 8;
			
			can->tx_frame_list[tx_frame_dx].hd_num = 1;
			can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;
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
				can->tx_frame_num++;// 帧加一
				tx_frame_dx = can->tx_frame_num - 1;// 索引后移一位
				
				can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
				can->tx_frame_list[tx_frame_dx].id = tx_id;
				can->tx_frame_list[tx_frame_dx].dlc = 8;
		
				can->tx_frame_list[tx_frame_dx].hd_num = 1;
				can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;
			}
		}
	}
	
	void DjiMotor::Tim_It_Process()
	{
		if (motor_mode == LOCAL_MIT_MODE)			//>本地计算mit模式
		{
			target_current = pid_pos.Mit_Calculate(pos, rpm, target_pos, target_rpm, ff_torque) * tor_to_cur;
		}
		else if (motor_mode != CURRENT_MODE)		//> 电流模式
		{
			float temp_target_rpm = 0;// 目标速度
			
			if (motor_mode == RPM_MODE)				//> 速度模式
			{
				temp_target_rpm = target_rpm;
			}
			else if (motor_mode == POS_MODE)		//> 位置模式
			{
				temp_target_rpm = pid_pos.Pid_Calculate(pos, target_pos);
			}
			else if (motor_mode == ANGLE_MODE)		//> 角度模式
			{
				temp_target_rpm = pid_pos.Pid_Calculate(angle, target_angle, true, PI);
			}
			else if (motor_mode == OUT_ANGLE_MODE && is_gear_ratio_int)	//> 输出轴角度模式（减速比为整数才能用）
			{
				temp_target_rpm = pid_pos.Pid_Calculate(((float)out_angle_int / 8192.f) * TWO_PI, target_pos, true, PI * gear_ratio);
			}
			
			target_current = pid_spd.Pid_Calculate(rpm, temp_target_rpm);
		}
		
		can->tx_frame_list[tx_frame_dx].new_message = true;
	}
	
	void DjiMotor::Can_Tx_Process()
	{
		uint16_t dx = ((id - 1) % 4) * 2;

		int16_t current_int = (int16_t)(target_current);
		
		if (current_int > 16384) current_int = 16384;
		else if (current_int < -16384) current_int = -16384;
		
		can->tx_frame_list[tx_frame_dx].data[dx] = (uint8_t)(current_int >> 8);// 高8位
		can->tx_frame_list[tx_frame_dx].data[dx + 1] = (uint8_t)(current_int);// 低8位
	}
	
	
	void DjiMotor::Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data)
	{
		encoder     = (int16_t)(((uint16_t)rx_data[0] << 8) | (uint16_t)rx_data[1]);
		rpm 		= (float)(int16_t)(((uint16_t)rx_data[2] << 8) | (uint16_t)rx_data[3]);
		current 	= (float)(int16_t)(((uint16_t)rx_data[4] << 8) | (uint16_t)rx_data[5]);
		temperature = (float)(int8_t)rx_data[6];
		
		angle 		= (float)encoder / 8192.f * TWO_PI;
		
		// 计算转子旋转圈数
		if (can_rx_is_first != true)
		{
			int16_t delta_encoder = encoder - last_encoder;
			
			if (delta_encoder > 4096)
			{
				cycle--;
				rotor_cycle--;
			}
			else if (delta_encoder < -4096)
			{
				cycle++;
				rotor_cycle++;
			}
		}
		else
		{
			can_rx_is_first = false;
		}
		
		// 更新
		last_encoder = encoder;

		if (is_gear_ratio_int)
		{
			// 防止pos变nan
			if (cycle > 1024) cycle = 1024;
			else if (cycle < -1024) cycle = -1024;
			
			// 计算转子位置
			pos = cycle * TWO_PI + angle + pos_offset;
			
			out_angle_int = rotor_cycle * 8192 + encoder + out_angle_offset;

			// 归一化[0, 8192 * gear_ratio)
			if (out_angle_int >= out_angle_max)
			{
				out_angle_int -= out_angle_max;
				rotor_cycle -= (int32_t)gear_ratio;
			}
			else if (out_angle_int < 0)
			{
				out_angle_int += out_angle_max;
				rotor_cycle += (int32_t)gear_ratio;
			}
		}

		/*位置，角度重置*/
		if (is_reset_pos)
		{
			Reset_Pos(0);
			Reset_Out_Angle(0);
			is_reset_pos = false;
		}
	}
	
	float DjiMotor::Get_Out_Angle()
	{
		return ((float)out_angle_int / (float)out_angle_max) * TWO_PI;
	}
	
	// 重置输出轴角度
	void DjiMotor::Reset_Out_Angle(float out_angle_)
	{
		if (out_angle_ < 0.f || out_angle_ >= TWO_PI) out_angle_ = 0.f;
		
		rotor_cycle = 0;
		out_angle_offset = roundf((out_angle_ / TWO_PI * gear_ratio) * 8192) - encoder;
	}
	
	float DjiMotor::Get_Angle()
	{
		return angle;
	}
}
	