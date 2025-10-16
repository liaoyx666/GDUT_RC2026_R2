#include "RC_m6020.h"

namespace motor
{
	M6020::M6020(uint8_t id_, can::Can &can_, tim::Tim &tim_) : DjiMotor(can_, tim_)
	{
		// 设置tx，rx和m6020的id
		Dji_Id_Init(id_);
		
		// 登记can设备
		CanHandler_Register();
		
		// m6020默认pid参数
		pid_spd.Pid_Mode_Init(true, false, 0);
		pid_spd.Pid_Param_Init(8, 1, 0, 0, 0.001, 0, 16384, 10000, 5000, 5000, 5000);
		
		pid_pos.Pid_Mode_Init(false, false, 0.4);
		pid_pos.Pid_Param_Init(500, 0, 5, 0, 0.001, 0, 300, 150, 150, 150, 150);
		
		
		
		
		pos_adrc.ADRC_Param_Init(
			5000,				// 输出限幅
			30,         		// 快速跟踪因子
			0.001,     		 	// 滤波因子，系统调用步长
			5,         			// 系统系数
			0.01,          		// fal函数的线性区间宽度
			100,          		// 扩张状态观测器反馈增益1
			300,          		// 扩张状态观测器反馈增益2
			1500,          		// 扩张状态观测器反馈增益3
			0.5,          		// 非线性因子1
			0.25,          		// 非线性因子2
			0,          		// 跟踪输入信号增益kp
			0           		// 跟踪微分信号增益kd
		);
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