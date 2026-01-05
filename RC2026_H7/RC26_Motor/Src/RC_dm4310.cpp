#include "RC_dm4310.h"



#define DM_ENABLE_FRAME_TAIL 	0xfc
#define DM_DISABLE_FRAME_TAIL 	0xfd



#define P_MIN -PI// rad
#define P_MAX PI

#define V_MIN -30.0f// rad/s
#define V_MAX 30.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f

#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define T_MIN -10.0f// N * m
#define T_MAX 10.0f


namespace motor
{
	DM4310::DM4310(uint8_t id_, can::Can &can_, tim::Tim &tim_, bool use_mit_, float k_spd_, float k_pos_)
		: can::CanHandler(can_), tim::TimHandler(tim_), Motor(1.5f), use_mit(use_mit_)
	{
		id = id_;// 电机id

		tx_id = id;

		rx_id = 0x300 + id;
		rx_mask = 0xfff;

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
		
		// dm4310默认参数
		pid_spd.Pid_Mode_Init(true, false, 0.4);
		pid_spd.Pid_Param_Init(0.025, 0.0007, 0, 0, 0.001, 0, 10, 5, 5, 5, 5);
		
		pid_pos.Pid_Mode_Init(false, false, 0.4, true);
		pid_pos.Pid_Param_Init(150, 0, 10, 0, 0.001, 0, 10, 5, 5 ,5 ,5, 2, 1.f);
	}
	
	void DM4310::CanHandler_Register()
	{
		can->tx_frame_num++;// 帧加一
		tx_frame_dx = can->tx_frame_num - 1;// 帧索引后移一位
		
		can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
		can->tx_frame_list[tx_frame_dx].id = tx_id;
		can->tx_frame_list[tx_frame_dx].dlc = 8;

		can->tx_frame_list[tx_frame_dx].hd_num = 1;
		can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;
	}
	
	
	void DM4310::Can_Tx_Process()
	{
		if (is_enable == true)
		{
			uint16_t pos_int, vel_int, kp_int, kd_int, tor_int;

			if (use_mit != true)// 不使用mit
			{
				pos_int = 0.f;
				vel_int = 0.f;
				kp_int  = 0.f;
				kd_int  = 0.f;
			}
			else
			{
				pid::Limit(&target_pos, P_MAX);
				pid::Limit(&target_rpm, V_MAX * 9.54929658551f);
				
				
				pos_int = float_to_uint(target_pos, P_MIN, P_MAX, 16);// rad
				vel_int = float_to_uint(target_rpm / 9.54929658551f, V_MIN, V_MAX, 12);// rpm to rad
				kp_int  = float_to_uint(target_k_pos, KP_MIN, KP_MAX, 12);
				kd_int  = float_to_uint(target_k_spd, KD_MIN, KD_MAX, 12);
			}
			
			float temp_target_torque = target_torque + feedforward;// 加上前馈力矩
			
			pid::Limit(&temp_target_torque, T_MAX);

			tor_int = float_to_uint(temp_target_torque, T_MIN, T_MAX, 12);

			can->tx_frame_list[tx_frame_dx].data[0] = (pos_int >> 8);
			can->tx_frame_list[tx_frame_dx].data[1] = pos_int;
			can->tx_frame_list[tx_frame_dx].data[2] = (vel_int >> 4);
			can->tx_frame_list[tx_frame_dx].data[3] = ((vel_int & 0xF) << 4) | (kp_int >> 8 );
			can->tx_frame_list[tx_frame_dx].data[4] = kp_int;
			can->tx_frame_list[tx_frame_dx].data[5] = (kd_int >> 4);
			can->tx_frame_list[tx_frame_dx].data[6] = ((kd_int & 0xF) << 4) | (tor_int >> 8);
			can->tx_frame_list[tx_frame_dx].data[7] = tor_int;
		}
		else
		{
			// 使能帧
			memset(can->tx_frame_list[tx_frame_dx].data, 0xff, 7);
			can->tx_frame_list[tx_frame_dx].data[7] = DM_ENABLE_FRAME_TAIL;
		}
	}
	
	
	void DM4310::Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data)
	{
		error_code		= (rx_data[0] >> 4);//错误状态
		angle 			= uint_to_float((rx_data[1] << 8) | rx_data[2], P_MIN, P_MAX, 16) + PI;//计算实际角度
		rpm 			= uint_to_float((rx_data[3] << 4) | (rx_data[4] >> 4), V_MIN, V_MAX, 12) * 9.54929658551f;//计算实际转速rad to rpm
		torque 			= uint_to_float(((rx_data[4] & 0x0f) << 8) | rx_data[5], T_MIN, T_MAX, 12);//计算扭矩电流N*m
		temperature 	= (float)(int8_t)rx_data[6];//线圈温度
		mos_temperature = (float)(int8_t)rx_data[7];//mos温度
		

		if (error_code == 0x01)
		{
			is_enable = true;
		}
		else
		{
			is_enable = false;
		}

		
		if (can_rx_is_first != true)//若不为第一次上发
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
		if (use_mit != true)// 不使用mit
		{
			if (motor_mode != TORQUE_MODE)			//> 力矩模式
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
				target_torque = pid_spd.Pid_Calculate();
			}
		}
	}



	void DM4310::Set_K_Pos(float target_k_pos_)
	{
		if (use_mit == true)
		{
			if (target_k_pos_ < KP_MIN) target_k_pos = KP_MIN;
			else if (target_k_pos_ > KP_MAX) target_k_pos = KP_MAX;
			else target_k_pos = target_k_pos_;
		}
	}
	
	
	void DM4310::Set_K_Spd(float target_k_spd_)
	{
		if (use_mit == true)
		{
			if (target_k_spd_ < KD_MIN) target_k_spd = KD_MIN;
			else if (target_k_spd_ > KD_MAX) target_k_spd = KD_MAX;
			else target_k_spd = target_k_spd_;
		}
	}
}