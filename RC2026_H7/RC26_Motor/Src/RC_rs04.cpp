#include "RC_rs04.h"

#define P_MIN -12.57f 
#define P_MAX 12.57f  

#define V_MIN -15.0f
#define V_MAX 15.0f 

#define KP_MIN 0.0f 
#define KP_MAX 5000.0f 

#define KD_MIN 0.0f 
#define KD_MAX 100.0f 

#define T_MIN -120.0f 
#define T_MAX 120.0f

namespace motor
{
	RS04::RS04(uint8_t id_, can::Can& can_, tim::Tim& tim_, bool use_mit_, float k_spd_, float k_pos_)
		: can::CanHandler(can_), tim::TimHandler(tim_), Motor(1.f), use_mit(use_mit_)
	{
		id = id_;
		
		can_frame_type = can::FRAME_EXT;
		
		rx_mask = (0xff << 8) | 0x00;
		rx_id = (id << 8) | 0x00;

		tx_id = id | 0x00;
		
		master_id = 0xFD;
		
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
	}
	
	
	void RS04::CanHandler_Register()
	{
		can->tx_frame_num++;// 帧加一
		tx_frame_dx = can->tx_frame_num - 1;// 帧索引后移一位
		
		can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
		can->tx_frame_list[tx_frame_dx].id = tx_id;
		can->tx_frame_list[tx_frame_dx].dlc = 8;

		can->tx_frame_list[tx_frame_dx].hd_num = 1;
		can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;
	}


	void RS04::Tim_It_Process()
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


	void RS04::Can_Tx_Process()
	{
		switch(tx_com_type)
		{
			case RS04_COM_TYPE_1_CONTROL:
			{
				int16_t tor_int, pos_int, spd_int, kp_int, kd_int;
				
				if (use_mit == true)
				{
					pos_int = float_to_uint(target_pos, P_MIN, P_MAX, 16);
					spd_int = float_to_uint(rpm_to_radps(target_rpm), V_MIN, V_MAX, 16);
					kp_int = float_to_uint(target_k_pos, KP_MIN, KP_MAX, 16);
					kd_int = float_to_uint(target_k_spd, KD_MIN, KD_MAX, 16);
				}
				else
				{
					pos_int = 0;
					spd_int = 0;
					kp_int = 0;
					kd_int = 0;
				}
				
				can->tx_frame_list[tx_frame_dx].data[0] = (pos_int >> 8);
				can->tx_frame_list[tx_frame_dx].data[1] = pos_int;
				can->tx_frame_list[tx_frame_dx].data[2] = (spd_int >> 8);
				can->tx_frame_list[tx_frame_dx].data[3] = spd_int;
				can->tx_frame_list[tx_frame_dx].data[4] = (kp_int >> 8);
				can->tx_frame_list[tx_frame_dx].data[5] = kp_int;
				can->tx_frame_list[tx_frame_dx].data[6] = (kd_int >> 8);
				can->tx_frame_list[tx_frame_dx].data[7] = kd_int;
				
				tor_int = float_to_uint(target_torque, T_MIN, T_MAX, 16);
				
				can->tx_frame_list[tx_frame_dx].id = (tx_com_type << 24) | ((uint16_t)tor_int << 8) | id;
				break;
			}
			
			case RS04_COM_TYPE_3_ENABLE:
			{
				memset(can->tx_frame_list[tx_frame_dx].data, 0, 8);
				can->tx_frame_list[tx_frame_dx].id = (tx_com_type << 24) | (master_id << 8) | id;
				break;
			}
			
			case RS04_COM_TYPE_7_SET_ID:
			{
				memset(can->tx_frame_list[tx_frame_dx].data, 0, 8);
				can->tx_frame_list[tx_frame_dx].id = (tx_com_type << 24) | (target_id << 16) | (master_id << 8) | id;
				id = target_id;
				rx_id = (id << 8) | 0x00;
				break;
			}
			
			case RS04_COM_TYPE_6_RESET_POS:
			{
				can->tx_frame_list[tx_frame_dx].data[0] = 1;
				memset(&can->tx_frame_list[tx_frame_dx].data[1], 0, 7);
				can->tx_frame_list[tx_frame_dx].id = (tx_com_type << 24) | (master_id << 8) | id;
				//tx_com_type = RS04_COM_TYPE_1_CONTROL;
				break;
			}
			
			case RS04_COM_TYPE_4_DISABLE:
			{
				memset(can->tx_frame_list[tx_frame_dx].data, 0, 8);
				can->tx_frame_list[tx_frame_dx].id = (tx_com_type << 24) | (master_id << 8) | id;
				is_enable = false;
				break;
			}
			
			default:
				tx_com_type = RS04_COM_TYPE_1_CONTROL;
				break;
		}
	}


	void RS04::Can_Rx_It_Process(uint32_t rx_id_, uint8_t* rx_data)
	{
		uint8_t return_com_type = (rx_id_ >> 24) & 0x1f;

		switch(return_com_type)
		{
			case RS04_COM_TYPE_0_ID:
			{
				tx_com_type = RS04_COM_TYPE_1_CONTROL;
				break;
			}
			
			case RS04_COM_TYPE_2_FEEDBACK:
			{
				if (is_enable == false)
				{
					is_enable = true;
					tx_com_type = RS04_COM_TYPE_1_CONTROL;
				}
				
				pos			 = uint_to_float((rx_data[0] << 8) | rx_data[1], P_MIN, P_MAX, 16) + pos_offset;
				rpm			 = radps_to_rpm(uint_to_float((rx_data[2] << 8) | rx_data[3], V_MIN, V_MAX, 16));
				torque		 = uint_to_float((rx_data[4] << 8) | rx_data[5], T_MIN, T_MAX, 16);
				temperature	 = (float)(int16_t)((rx_data[6] << 8) | rx_data[7]);
				
				if (tx_com_type == RS04_COM_TYPE_6_RESET_POS && fabs(pos) < 0.01f)
				{
					tx_com_type = RS04_COM_TYPE_1_CONTROL;
				}
				break;
			}

			default:
				break;
		}
	}
	
	
	void RS04::Set_K_Pos(float target_k_pos_)
	{
		if (use_mit == true)
		{
			if (target_k_pos_ < KP_MIN) target_k_pos = KP_MIN;
			else if (target_k_pos_ > KP_MAX) target_k_pos = KP_MAX;
			else target_k_pos = target_k_pos_;
		}
	}
	
	
	void RS04::Set_K_Spd(float target_k_spd_)
	{
		if (use_mit == true)
		{
			if (target_k_spd_ < KD_MIN) target_k_spd = KD_MIN;
			else if (target_k_spd_ > KD_MAX) target_k_spd = KD_MAX;
			else target_k_spd = target_k_spd_;
		}
	}
	
	
	void RS04::Set_Can_Id(uint8_t target_id_)
	{
		if (is_enable != true)
		{
			target_id = target_id_;
			tx_com_type = RS04_COM_TYPE_7_SET_ID;
		}
	}
	
	
	void RS04::Set_ZeroPos()
	{
		if (use_mit == true)
		{
			tx_com_type = RS04_COM_TYPE_6_RESET_POS;
		}
	}
}