#include "RC_m3508.h"

namespace motor
{
	M3508::M3508(uint8_t id_, can::Can &can_, tim::Tim *tim_, float gear_ratio_, bool is_reset_pos_angle) : DjiMotor(can_, tim_, gear_ratio_, is_reset_pos_angle)
	{
		tor_to_cur = (16384.f / (6.f / (3591.f / 187.f)));
		
		// 设置tx，rx和m3508的id
		Dji_Id_Init(id_);
		
		// 登记can设备
		CanHandler_Register();
		
		// m3508默认pid参数
		pid_spd.Pid_Mode_Init(true, false, 0.01);
		pid_spd.Pid_Param_Init(10, 0.54, 0, 0, 0.001, 0, 16384, 10000, 5000, 5000, 5000);
		
		pid_pos.Pid_Mode_Init(false, false, 0.01, true);
		pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 300, 1000, 500, 500, 500, 150, 200);
	}
	
	
	void M3508::Dji_Id_Init(uint8_t id_)
	{
		if (id_ <= 8 && id_ != 0) id = id_;
		else Error_Handler();
		
		// 设置发送帧id
		if (id <= 4) tx_id = 0x200;
		else tx_id = 0x1ff;
		
		// 设置接收帧id和mask
		rx_mask = 0xfff;
		rx_id = 0x200 + id;
	}
	
	M3508D::M3508D(
		uint8_t id_m, can::Can &can_m, tim::Tim *tim_m, 
		uint8_t id_s, can::Can &can_s, tim::Tim *tim_s, 
		float gear_ratio_, MotorPol pol_s, bool is_reset_pos_angle
	) : M3508(id_m, can_m, tim_m, gear_ratio_, is_reset_pos_angle), slave(id_s, can_s, tim_s, gear_ratio_, is_reset_pos_angle), pol(pol_s)
	{
		
	}
	
	void M3508D::Tim_It_Process()
	{
		if (motor_mode != CURRENT_MODE)		//> 电流模式
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
		
		slave.Set_Current(target_current * (float)(int8_t)pol); /*从电机电流大小与主电机相同*/
		can->tx_frame_list[tx_frame_dx].new_message = true;
	}
}