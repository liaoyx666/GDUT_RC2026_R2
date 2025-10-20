#include "RC_j60.h"


#define P_MIN -40.f// rad
#define P_MAX 40.f

#define V_MIN -40.f// rad/s
#define V_MAX 40.f

#define KP_MIN 0.0f
#define KP_MAX 1023.f

#define KD_MIN 0.0f
#define KD_MAX 51.f

#define T_MIN -40.f// N * m
#define T_MAX 40.f

#define TEMP_MIN -20.f// 摄氏度
#define TEMP_MAX 200.f

// 命令种类
#define CMD_CONTROL 4
#define CMD_DISABLE 1
#define CMD_ENABLE 2


namespace motor
{
	J60::J60(uint8_t id_, can::Can& can_, tim::Tim& tim_, bool use_mit_, float k_spd_, float k_pos_)
		: motor::Motor(1.f), can::CanHandler(can_), tim::TimHandler(tim_), use_mit(use_mit_)
	{  
		if (id_ > 15) Error_Handler();
		id = id_;

		tx_id = id;

		rx_id = 0x10 + id;
		rx_mask = 0x01f;

		can_frame_type = can::FRAME_STD;

		CanHandler_Register();
		
		if (use_mit_ != true)
		{
			k_spd = 0;// 阻尼系数
			k_pos = 0;// 刚度系数
		}
		else
		{
			Set_K_Spd(k_spd_);// 阻尼系数
			Set_K_Pos(k_pos_);// 刚度系数
		}
		
		// j60默认参数
		pid_spd.Pid_Mode_Init(true, true, 0.1);
		pid_spd.Pid_Param_Init(0.1, 0.0006, 0, 0, 0.001, 0, 40, 20, 20, 20, 20);
			
		pid_pos.Pid_Mode_Init(false, false, 0.1, true); 
		pid_pos.Pid_Param_Init(250, 0, 15, 0, 0.001, 0, 10, 5, 5, 5, 5, 2, 0.5f);
	}


	void J60::CanHandler_Register()
	{
		can->tx_frame_num++;// 帧加一
		tx_frame_dx = can->tx_frame_num - 1;// 帧索引后移一位
		
		can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
		can->tx_frame_list[tx_frame_dx].id = tx_id;
		can->tx_frame_list[tx_frame_dx].dlc = 8;

		can->tx_frame_list[tx_frame_dx].hd_num = 1;
		can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;
	}


	void J60::Tim_It_Process()
	{
		if (use_mit != true)// 不使用mit
		{
			if (motor_mode != TORQUE_MODE)			//> 力矩模式
			{
				// 目标速度
				float temp_target_rpm = 0;
				
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
				
				pid_spd.Update_Target(temp_target_rpm);
				pid_spd.Update_Real(rpm);
				target_torque = pid_spd.Pid_Calculate();
			}
		}
	}


	void J60::Can_Tx_Process()
	{
		if (is_enable)
		{
			uint16_t pos_int, vel_int, kp_int, kd_int, tor_int;

			if (use_mit != true)
			{
				// 不使用mit
				pos_int = 0;
				vel_int = 0;
				kp_int  = 0;
				kd_int  = 0;
			}
			else
			{
				// 使用mit
				pid::Limit(&target_pos, P_MAX);// 限幅
				pid::Limit(&target_rpm, V_MAX * 9.54929658551f);// 限幅
				
				
				pos_int = float_to_uint(target_pos, P_MIN, P_MAX, 16);// rad
				vel_int = float_to_uint(target_rpm / 9.54929658551f, V_MIN, V_MAX, 14);// rpm to rad
				kp_int  = float_to_uint(target_k_pos, KP_MIN, KP_MAX, 10);
				kd_int  = float_to_uint(target_k_spd, KD_MIN, KD_MAX, 8);
			}
			
			float temp_target_torque = target_torque + feedforward;// 加上前馈力矩
			
			pid::Limit(&temp_target_torque, T_MAX);

			tor_int = float_to_uint(temp_target_torque, T_MIN, T_MAX, 16);
			
			can->tx_frame_list[tx_frame_dx].data[0] = pos_int;
			can->tx_frame_list[tx_frame_dx].data[1] = (pos_int >> 8);
			can->tx_frame_list[tx_frame_dx].data[2] = vel_int;
			can->tx_frame_list[tx_frame_dx].data[3] = ((vel_int >> 8) & 0x3f) | ((kp_int & 0x03) << 6);
			can->tx_frame_list[tx_frame_dx].data[4] = (kp_int >> 2);
			can->tx_frame_list[tx_frame_dx].data[5] = kd_int;
			can->tx_frame_list[tx_frame_dx].data[6] = tor_int;
			can->tx_frame_list[tx_frame_dx].data[7] = (tor_int >> 8);

			can->tx_frame_list[tx_frame_dx].id = id | (CMD_CONTROL << 5);
			can->tx_frame_list[tx_frame_dx].dlc = 8;
		}
		else
		{
			// 使能帧
			memset(can->tx_frame_list[tx_frame_dx].data, 0, 8);
			
			can->tx_frame_list[tx_frame_dx].id = id | (CMD_ENABLE << 5);
			can->tx_frame_list[tx_frame_dx].dlc = 0;
		}
	}


	void J60::Can_Rx_It_Process(uint32_t rx_id_, uint8_t* rx_data)
	{
		uint8_t return_id = rx_id_ & 0x0F;// 接收的id
		uint8_t return_flag = (rx_id_ >> 4) & 0x01;// 返回帧标志
		uint8_t return_cmd = (rx_id_ >> 5) & 0x3F;// 返回命令

		if (return_id != id && return_flag != 0x01)
		{
			return;
		}
		
		switch (return_cmd)
		{
		case 1:
			if (rx_data[0] == 0) is_enable = false;// 失能成功
			break;

		case 2:
			if (rx_data[0] == 0) is_enable = true;// 使能成功
			break;

		case 4:
		{
			pos 				= uint_to_float(rx_data[0] | (rx_data[1] << 8) | ((rx_data[2] & 0x0f) << 16), P_MIN, P_MAX, 20) + pos_offset;//计算实际位置
			rpm 				= uint_to_float(((rx_data[2] & 0xf0) >> 4) | (rx_data[3] << 4) | (rx_data[4] << 12), V_MIN, V_MAX, 20) * 9.54929658551f;//计算实际转速rad to rpm
			torque 				= uint_to_float(rx_data[5] | (rx_data[6] << 8), T_MIN, T_MAX, 16);//计算扭矩电流N*m
			uint8_t temp_flag 	= rx_data[7] & 0x01;
			float temp 			= uint_to_float((rx_data[7] & 0xfe) >> 1, TEMP_MIN, TEMP_MAX, 7);

			out_pos = pos / gear_ratio;
			
			if (temp_flag == 1) temperature = temp; 
			else mos_temperature = temp; 
			break;
		}
		default:
			break;
		}
	}
	
	
	
	void J60::Set_K_Pos(float target_k_pos_)
	{
		if (use_mit == true)
		{
			if (target_k_pos_ < KP_MIN) target_k_pos = KP_MIN;
			else if (target_k_pos_ > KP_MAX) target_k_pos = KP_MAX;
			else target_k_pos = target_k_pos_;
		}
	}
	
	
	void J60::Set_K_Spd(float target_k_spd_)
	{
		if (use_mit == true)
		{
			if (target_k_spd_ < KD_MIN) target_k_spd = KD_MIN;
			else if (target_k_spd_ > KD_MAX) target_k_spd = KD_MAX;
			else target_k_spd = target_k_spd_;
		}
	}

}