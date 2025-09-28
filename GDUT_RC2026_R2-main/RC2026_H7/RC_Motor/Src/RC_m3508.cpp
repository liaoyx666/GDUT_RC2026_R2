#include "RC_m3508.h"

namespace motor
{
	M3508::M3508(uint8_t id_, can::Can &can_, tim::Tim &tim_) : DjiMotor(can_, tim_)
	{
		// 设置tx，rx和m3508的id
		Dji_Id_Init(id_);
		
		// 登记can设备
		CanHandler_Register();
		
		// m3508默认pid参数
		pid_spd.Pid_Mode_Init(true, false, 0);
		pid_spd.Pid_Param_Init(10, 0.54, 0, 0, 0.001, 0, 15000, 10000, 5000, 5000, 5000);// 1ms
		
		pid_pos.Pid_Mode_Init(false, false, 0);
		pid_pos.Pid_Param_Init(100, 0, 0.005, 0, 0.001, 0, 1000, 1000, 500, 500, 500);// 1ms
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

}