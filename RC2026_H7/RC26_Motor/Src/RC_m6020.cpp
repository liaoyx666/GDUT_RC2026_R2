#include "RC_m6020.h"

namespace motor
{
	M6020::M6020(uint8_t id_, can::Can &can_, tim::Tim &tim_, bool is_reset_pos_angle, M6020CtrlType control_type_) : DjiMotor(can_, tim_, 1.f, is_reset_pos_angle), control_type(control_type_)
	{
		// 设置tx，rx和m6020的id
		Dji_Id_Init(id_);
		
		// 登记can设备
		CanHandler_Register();
		
		if (control_type == M6020CtrlType::CURRENT)// 电流控制
		{
			// m6020默认pid参数
			pid_spd.Pid_Mode_Init(true, false, 0);
			pid_spd.Pid_Param_Init(8, 1, 0, 0, 0.001, 0, 16384, 10000, 5000, 5000, 5000);

			pid_pos.Pid_Mode_Init(false, false, 0.01);
			pid_pos.Pid_Param_Init(500, 0, 5, 0, 0.001, 0, 300, 150, 150, 150, 150);
		}
		else if (control_type == M6020CtrlType::VOLTAGE)// 电压控制
		{
			pid_spd.Pid_Mode_Init(true, false, 0);
			pid_spd.Pid_Param_Init(10, 7, 0, 0, 0.001, 0, 25000, 10000, 5000, 5000, 5000);

			pid_pos.Pid_Mode_Init(false, false, 0.01);
			pid_pos.Pid_Param_Init(0, 0, 0, 0, 0.001, 0, 300, 150, 150, 150, 150);
		}
	}
	
	void M6020::Can_Tx_Process()
	{
		uint16_t dx = ((id - 1) % 4) * 2;

		int16_t current_int = (int16_t)(target_current);
		
		if (control_type == M6020CtrlType::CURRENT)// 电流控制
		{
			if (current_int > 16384) current_int = 16384;
			else if (current_int < -16384) current_int = -16384;
		}
		else if (control_type == M6020CtrlType::VOLTAGE)// 电压控制
		{
			//// 用target_current表示target_voltage
			if (current_int > 25000) current_int = 25000;
			else if (current_int < -25000) current_int = -25000;
		}
		
		can->tx_frame_list[tx_frame_dx].data[dx] = (uint8_t)(current_int >> 8);// 高8位
		can->tx_frame_list[tx_frame_dx].data[dx + 1] = (uint8_t)(current_int);// 低8位
	}
	
	
	void M6020::Dji_Id_Init(uint8_t id_)
	{
		if (id_ <= 7 && id_ != 0) id = id_;
		else Error_Handler();
		
		if (control_type == M6020CtrlType::CURRENT)// 电流控制
		{
			// 设置发送帧id
			if (id <= 4) tx_id = 0x1fe;
			else tx_id = 0x2fe;
		}
		else if (control_type == M6020CtrlType::VOLTAGE)// 电压控制
		{
			// 设置发送帧id
			if (id <= 4) tx_id = 0x1ff;
			else tx_id = 0x2ff;
		}
		
		// 设置接收帧id和mask
		rx_mask = 0xfff;
		rx_id = 0x204 + id;
	}
}