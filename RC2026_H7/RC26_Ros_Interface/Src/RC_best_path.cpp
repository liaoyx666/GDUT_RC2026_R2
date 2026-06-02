#include "RC_best_path.h"

namespace ros
{
	BestPath::BestPath(cdc::CDC &cdc_, uint8_t rx_id_, path::Navigation& navi_) : cdc::CDCHandler(cdc_, rx_id_), navi(navi_)
	{
		path_len = 0;
		is_init = false;
		kfs_pick_num = 0;
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
	
	void BestPath::Generate_Path()
	{
		for (uint8_t i = 1; i <= 12; i++)
		{
			path::MapGraph::Set_MF_Valid(i, false);
		}
		
		uint8_t i = 0;
		while (i < path_len)
		{
			if (path[i] == '(')
			{
				kfs_pick_num++;
				i += 2;
			}
			else if (path[i] == 0)
			{
				break;
			}
			else
			{
				path::MapGraph::Set_MF_Valid(i, true);
			}
			
			i += 1;
		}
		
		
		
		
		
		
		
		
		
		
	}
	

}