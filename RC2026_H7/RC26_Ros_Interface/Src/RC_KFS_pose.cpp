#include "RC_KFS_pose.h"

namespace ros
{
	KfsPose::KfsPose(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_), id(rx_id_)
	{
		x = 0;
		is_enable = false;
	}
	
	void KfsPose::CDC_Receive_Process(uint8_t *buf, uint16_t len)
	{
		if (len == 4)
		{
			x = *(float*)(&buf[0]);
			is_enable = true;
		}
		else if (len == 0 && *buf == 0)
		{
			/* 失能应答 */
			is_enable = false;
		}
	}
	
	bool KfsPose::Enable()
	{
		uint8_t send = 1;
		cdc->CDC_Send_Pkg(id, &send, 1, 0); /*不等待*/
		return is_enable;
	}
	
	bool KfsPose::Disable()
	{
		uint8_t send = 0;
		cdc->CDC_Send_Pkg(id, &send, 1, 0); /*不等待*/
		return !is_enable;
	}
	
}