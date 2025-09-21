#include "RC_ros_interface.h"


namespace ros
{
	Radar::Radar(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_)
	{
		
	}
	
	void Radar::CDC_Receive_Process(uint8_t *buf, uint16_t len)
	{
		x = *(float*)buf;
		
	}
	
	
}