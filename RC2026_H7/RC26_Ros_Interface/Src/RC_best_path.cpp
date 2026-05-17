#include "RC_best_path.h"

namespace ros
{
	BestPath::BestPath(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_)
	{
		path_len = 0;
		is_init = false;
	}
	
	void BestPath::CDC_Receive_Process(uint8_t *buf, uint16_t len)
	{
		if (len <= BESTPATH_MAX_PATH_LEN)
		{
			if (!is_init)
			{
				path_len = len;
				
				memcpy(path, buf, len);
				
				is_init = true;
			}
			
			// 应答
			uint8_t ack = 1;
			cdc->CDC_Send_Pkg(3, &ack, 1, 1000);
		}
	}
	

}