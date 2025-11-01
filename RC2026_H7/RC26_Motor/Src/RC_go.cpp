#include "RC_go.h"

namespace motor
{
	/**
	* @brief 
	* @note !!!go电机的canh和canl是反的，和大疆电机不一样!!!
	* @param id_:电机id
	* @param module_id_:can转485模块id
	* @param can_:挂载的can外设类
	* @param tim_:挂载的tim定时器中断外设类
	* @param use_mit_:是否使用mit控制（如果不用速度环，位置环在stm32实现，只发送力矩）
	* @param k_spd_:阻尼系数（只使用力矩时给0）
	* @param k_pos_:刚度系数（只使用力矩时给0）
	*/
	Go::Go(uint8_t id_, uint8_t module_id_, can::Can &can_, tim::Tim &tim_, bool use_mit_, float k_spd_, float k_pos_)
		: can::CanHandler(can_), tim::TimHandler(tim_), use_mit(use_mit_), Motor(6.33f)
	{
		if (module_id_ > 3) Error_Handler();
		if (id_ > 14) Error_Handler();
		
		id = id_;// 电机id
		module_id = module_id_;// 模块id
		
		can_frame_type = can::FRAME_EXT;
		
		//!!!unitree文档错误，读取k模式时id的25，26位返回2而非1!!!
		rx_mask = (3 << 27) | (1 << 26) | (15 << 8);
		rx_id = (module_id << 27) | (1 << 26) | (id << 8);

		// 第一次通讯先设置k值
		tx_id = (module_id << 27) | (0 << 26) | (0 << 24) | ((uint8_t)GO_CONTROL_WRITE_K << 16) | (id << 8) | ((uint8_t)GO_STATUS_FOC << 12);
		
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
		
		// go默认pid参数
		pid_spd.Pid_Mode_Init(true, false, 0);
		pid_spd.Pid_Param_Init(0.004, 0.0001, 0, 0, 0.001, 0, 127, 60, 60, 60, 60);
		
		pid_pos.Pid_Mode_Init(false, false, 0, true);
		pid_pos.Pid_Param_Init(300, 0, 5, 0, 0.001, 0, 10, 5, 5, 5, 5, 2, 1.f);
	}

	
	
	// 注册can设备
	void Go::CanHandler_Register()
	{
		can->tx_frame_num++;// 帧加一
		tx_frame_dx = can->tx_frame_num - 1;// 帧索引后移一位
		
		can->tx_frame_list[tx_frame_dx].frame_type = can_frame_type;
		can->tx_frame_list[tx_frame_dx].id = tx_id;
		can->tx_frame_list[tx_frame_dx].dlc = 8;

		can->tx_frame_list[tx_frame_dx].hd_num = 1;
		can->tx_frame_list[tx_frame_dx].hd_dx[0] = hd_list_dx;
	}

	
	
	// can发送处理
	void Go::Can_Tx_Process()
	{
		// 设置id
		can->tx_frame_list[tx_frame_dx].id = (module_id << 27) | (0 << 26) | (0 << 24) | ((uint8_t)control_mode << 16) | ((uint8_t)motor_status << 12) | (id << 8);
		
		if (control_mode == GO_CONTROL_MODE_1 || control_mode == GO_CONTROL_MODE_2)
		{
			pid::Limit(&target_torque, 127.99f);
			
			int16_t tor_int = (int16_t)(target_torque * 256.f);
			
			if (use_mit != true)// 只使用前馈力矩控制
			{
				memset(&can->tx_frame_list[tx_frame_dx].data[0], 0, 6);
				memcpy(&can->tx_frame_list[tx_frame_dx].data[6], &tor_int, 2);
			}
			else// 使用mit
			{
				float temp_target_pos = target_pos - pos_offset;
				
				pid::Limit(&temp_target_pos, 6000);
				pid::Limit(&target_rpm, 7677);
				
				int32_t pos_int = (int32_t)(temp_target_pos / TWO_PI * 32768.f);
				int16_t spd_int = (int16_t)(target_rpm * 256.f / 60.f);
				
				memcpy(&can->tx_frame_list[tx_frame_dx].data[0], &pos_int, 4);
				memcpy(&can->tx_frame_list[tx_frame_dx].data[4], &spd_int, 2);
				memcpy(&can->tx_frame_list[tx_frame_dx].data[6], &tor_int, 2);
			}
		}
		else if (control_mode == GO_CONTROL_WRITE_K)// 设置k
		{
			uint16_t k_spd_int = (uint16_t)(target_k_spd * 1280.f);
			uint16_t k_pos_int = (uint16_t)(target_k_pos * 1280.f);

			memcpy(&can->tx_frame_list[tx_frame_dx].data[0], &k_spd_int, 2);
			memcpy(&can->tx_frame_list[tx_frame_dx].data[2], &k_pos_int, 2);
			
			control_mode = GO_CONTROL_MODE_1;// 切换模式1
		}
		else if (control_mode == GO_CONTROL_READ_K)// 读取k
		{
			control_mode = GO_CONTROL_MODE_1;// 切换模式1
		}
		else
		{
			control_mode = GO_CONTROL_MODE_1;// 切换模式1
		}
	}

	
	
	//can接收处理
	void Go::Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data)
	{
		if ((rx_id_ & 0xff) == 0x80)
		{
			uint8_t error = ((rx_id_ >> 8) & 0xff);
			if (error < 4)
			{
				error_code = (GoError)error;// 发生错误
			}
		}
		else
		{
			if (((rx_id_ >> 16) & 0xff) == 2)
			{
				k_spd = (float)(uint16_t)(rx_data[1] << 8 | rx_data[0]) / 1280.f;// 阻尼系数
				k_pos = (float)(rx_data[3] << 8 | rx_data[2]) / 1280.f;// 刚度系数
			}
			else
			{
				temperature = (int8_t)(rx_id_ & 0xff);
				air = (uint8_t)(rx_id_ >> 8);
				pos = ((float)(int32_t)(((uint32_t)rx_data[3] << 24) | ((uint32_t)rx_data[2] << 16) | ((uint32_t)rx_data[1] << 8) | (uint32_t)rx_data[0])) / 32768.f * TWO_PI + pos_offset;
				rpm = ((float)(int16_t)(((uint16_t)rx_data[5] << 8) | (uint16_t)rx_data[4])) / 256.f * 60.f;
				torque = ((float)(int16_t)(((uint16_t)rx_data[7] << 8) | (uint16_t)rx_data[6])) / 256.f;
				
				out_pos = pos / gear_ratio;
			}
		}
	}


	
	
	// 定时器中断计算pid
	void Go::Tim_It_Process()
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
	
	
	
	void Go::Set_K_Pos(float target_k_pos_)
	{
		if (use_mit == true)
		{
			if (target_k_pos_ < 0) target_k_pos = 0;
			else if (target_k_pos_ > 25.599f) target_k_pos = 25.599f;
			else target_k_pos = target_k_pos_;
			
			control_mode = GO_CONTROL_WRITE_K;
		}
	}
	
	void Go::Set_K_Spd(float target_k_spd_)
	{
		if (use_mit == true)
		{
			if (target_k_spd_ < 0) target_k_spd = 0;
			else if (target_k_spd_ > 25.599f) target_k_spd = 25.599f;
			else target_k_spd = target_k_spd_;
			
			control_mode = GO_CONTROL_WRITE_K;
		}
	}
}