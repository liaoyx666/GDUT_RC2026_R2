#include "RC_ros_interface.h"



namespace ros
{
	/*-----------------------------------Radar---------------------------------------*/
	
	Radar::Radar(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_)
	{
		
	}
	
	void Radar::CDC_Receive_Process(uint8_t *buf, uint16_t len)
	{
		if (len == 16)
		{
			x   = *(float*)(&buf[0]);
			y   = *(float*)(&buf[4]);
			z   = *(float*)(&buf[8]);
			yaw = *(float*)(&buf[12]);
		}
	}
	
	
	/*-----------------------------------Map---------------------------------------*/
	
	Map::Map(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_)
	{
		memset(map, -1, 12);// 初始化为未知
	}
	
	void Map::CDC_Receive_Process(uint8_t *buf, uint16_t len)
	{
		
	}
	
	
	
	
	
	
}