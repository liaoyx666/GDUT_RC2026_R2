#include "RC_best_path.h"

namespace ros
{
	BestPath::BestPath(cdc::CDC &cdc_, uint8_t rx_id_, path::Navigation& navi_) : cdc::CDCHandler(cdc_, rx_id_), navi(navi_)
	{
		path_len = 0;
		is_init = false;
		kfs_pick_num = 0;
		
		state = PICK_KFS_WAIT_PATH;
		current_path = 0;
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
			cdc->CDC_Send_Pkg(7, &ack, 1, 1000);
		}
	}
	
	void BestPath::Generate_Path()
	{
		
		while (!is_init)
		{
			osDelay(10);
		}
		
		for (uint8_t i = 1; i <= 12; i++)
		{
			path::MapGraph::Set_MF_Valid(i, false);
		}
		
		uint8_t i = 0;
		
		while (i < path_len)
		{
//			if (path[i] == '(')
//			{
//				if (i == 0 || path[i - 1] == ')')
//				{
//					pick_kfs[kfs_pick_num].kfs_pos = path[i + 1];
//					pick_kfs[kfs_pick_num].dir = path::DIR_B;
//				}
//				else
//				{
//					pick_kfs[kfs_pick_num].kfs_pos = path[i + 1];
//					pick_kfs[kfs_pick_num].dir = path::MapGraph::Dir_From_To(path[i + 1], path[i - 1]);
//				}
//				
//				kfs_pick_num++;
//				i += 2;
//			}
//			else if (path[i] == 0)
//			{
//				break;
//			}
//			else
//			{
//				path::MapGraph::Set_MF_Valid(path[i], true);
//			}

			if (path[i] == 0) break;




			switch (state)
			{
				case PICK_KFS_WAIT_PATH:
				{
					if (path[i] == '(')
					{
						state = PICK_KFS_WAIT_KFS_POS;
					}
					else
					{
						path::MapGraph::Set_MF_Valid(path[i], true);
						current_path = path[i];
					}
					
					break;
				}
				
				case PICK_KFS_WAIT_KFS_POS:
				{
					if (path[i] == ')')
					{
						state = PICK_KFS_WAIT_PATH;
					}
					else
					{
						pick_kfs[kfs_pick_num].kfs_pos = path[i];
						pick_kfs[kfs_pick_num].dir = path::MapGraph::Dir_From_To(path[i], current_path);
						
						kfs_pick_num++;
					}
					
					break;
				}
				
				default:
				{
					state = PICK_KFS_WAIT_PATH;
					break;
				}
			}
			
			
			
			
			i += 1;
		}
		
		
		
		
		
		for (uint8_t i = 0; i < kfs_pick_num; i++)
		{
			navi.Go_To_Get_KFS(pick_kfs[i].kfs_pos, pick_kfs[i].dir);
		}
		
		
	}
	
	
	
}