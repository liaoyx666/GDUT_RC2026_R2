#include "RC_radar.h"


namespace ros
{
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
			
			latest_receive_time = timer::Timer::Get_TimeStamp();
		}
	}
	
	bool Radar::Is_Valid()
	{
		uint32_t delta_time = timer::Timer::Get_DeltaTime(latest_receive_time);
		
		if (delta_time > 200000)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
	
	bool Radar::Get_Radar_Data(float* x_, float* y_, float* yaw_)
	{
		*x_ = x;
		*y_ = y;
		*yaw_ = yaw;
	
		return Is_Valid();
	}
}