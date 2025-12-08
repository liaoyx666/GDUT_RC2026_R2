#include "RC_ros_arm.h"

namespace ros
{
	RosArm::RosArm(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_)
	{
		
	}
	
	void RosArm::CDC_Receive_Process(uint8_t *buf, uint16_t len)
	{
		if (len == 16)
		{
			
		}
	}
}