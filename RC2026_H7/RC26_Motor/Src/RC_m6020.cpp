#include "RC_m6020.h"

namespace motor
{
	M6020::M6020(uint8_t id_, can::Can &can_, tim::Tim &tim_, bool is_reset_pos_angle) : DjiMotor(can_, tim_, 1.f, is_reset_pos_angle)
	{
		// 设置tx，rx和m6020的id
		Dji_Id_Init(id_);
		
		// 登记can设备
		CanHandler_Register();
		
		// m6020默认pid参数
		pid_spd.Pid_Mode_Init(true, false, 0);
		pid_spd.Pid_Param_Init(8, 1, 0, 0, 0.001, 0, 16384, 10000, 5000, 5000, 5000);


		pid_pos.Pid_Mode_Init(false, false, 0.2);
		pid_pos.Pid_Param_Init(500, 0, 5, 0, 0.001, 0, 300, 150, 150, 150, 150);
		
		
//		spd_smc.SMC_Init(
//			16000,
//			1,
//			2,
//			0.1,
//			0.001
//		);
		
	}
	
	
	void M6020::Dji_Id_Init(uint8_t id_)
	{
		if (id_ <= 7 && id_ != 0) id = id_;
		else Error_Handler();
		
		// 设置发送帧id
		if (id <= 4) tx_id = 0x1fe;
		else tx_id = 0x2fe;
		
		// 设置接收帧id和mask
		rx_mask = 0xfff;
		rx_id = 0x204 + id;
	}
}