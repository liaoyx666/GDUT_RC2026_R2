#include "RC_go.h"

namespace motor
{
	Go::Go(uint8_t id_, uint8_t module_id_, can::Can &can_, tim::Tim &tim_) : can::CanHandler(can_), tim::TimHandler(tim_)
	{
		if (module_id_ > 3) Error_Handler();
		if (id_ > 14) Error_Handler();
		
		id = id_;
		module_id = module_id_;
		
		can_frame_type = can::FRAME_EXT;
		
		rx_mask = (3 << 28) | (1 << 27) | (3 << 25) | (15 << 12);// 1 1111 0000 0000 1111 0000 0000 0000
		rx_id = (module_id << 28) | (1 << 27) | (1 << 25) | (id << 12);
		
		tx_id = (module_id << 28) | (0 << 27) | (0 << 25) | (CONTROL_MODE_1 << 16) | (id << 4) | (STATUS_FOC << 1);
		
		CanHandler_Register();
	}
	
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
	
	void Go::Can_Tx_Process()
	{
		
	}
	
	void Go::Can_Rx_It_Process(uint32_t rx_id_, uint8_t *rx_data)
	{
		if ((rx_id_ & 0xff) == 0x80)
		{
			error_code = (rx_id_ >> 8) & 0xff;// 发生错误
		}
		else
		{
			if ((rx_id_ & (0xf << 8)) == 0)
			{
				k_spd = (uint16_t)(rx_data[0] << 8 | rx_data[1]);// 阻尼系数
				k_pos = (uint16_t)(rx_data[2] << 8 | rx_data[3]);// 刚度系数
			}
			else
			{
				temperature = (int8_t)(rx_id_ & 0xff);
				air = (uint8_t)(rx_id_ >> 8);
				pos = ((float)(int32_t)(((uint32_t)rx_data[0] << 24) | ((uint32_t)rx_data[1] << 16) | ((uint32_t)rx_data[2] << 8) | (uint32_t)rx_data[3])) / 32768.f * TWO_PI;
				rpm = ((float)(int16_t)(((uint16_t)rx_data[4] << 8) | (uint16_t)rx_data[5])) / 256.f * 60.f;
				torque = ((float)(int16_t)(((uint16_t)rx_data[6] << 8) | (uint16_t)rx_data[7])) / 256.f;
			}
		}
	}
}