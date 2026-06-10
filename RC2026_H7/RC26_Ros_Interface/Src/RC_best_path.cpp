#include "RC_best_path.h"

namespace ros
{
	uint8_t BestPath::wait_R1_num = 0;
	uint8_t BestPath::wait_R1_pos[] = { 0 };
	
	BestPath::BestPath(cdc::CDC &cdc_, uint8_t rx_id_, path::Navigation& navi_) : cdc::CDCHandler(cdc_, rx_id_), navi(navi_)
	{
		path_len = 0;
		is_init = false;
		pick_kfs_num = 0;
		
		state = PICK_KFS_WAIT_PATH;
		current_path = 0;
		R1_kfs_num  = 0;
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
			cdc->CDC_Send_Pkg(7, &ack, 1, 100);
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
		
		
		
		for (uint8_t i = 0; i < path_len; i += 1)
		{
			if (path[i] == 0)
			{
				state = PICK_KFS_WAIT_R1_KFS;
				continue;
			}

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
						pick_kfs[pick_kfs_num].kfs_pos = path[i];
						pick_kfs[pick_kfs_num].dir = path::MapGraph::Dir_From_To(path[i], current_path);
						
						pick_kfs_num++;
					}
					
					break;
				}
				
				case PICK_KFS_WAIT_R1_KFS:
				{
					R1_kfs_pos[R1_kfs_num] = path[i];
					R1_kfs_num++;
					break;
				}
				
				default:
				{
					state = PICK_KFS_WAIT_PATH;
					break;
				}
			}
		}
		
		/* 设置需要躲避R1KFS的位置 */
		for (uint8_t i = 0; i < R1_kfs_num; i++)
		{
			if (path::MapGraph::Is_Valid(R1_kfs_pos[i]))
			{
				wait_R1_pos[i] = R1_kfs_pos[i];
				wait_R1_num++;
			}
		}
		
		/* 生成路径 */
		for (uint8_t i = 0; i < pick_kfs_num; i++)
		{
			navi.Go_To_Get_KFS(pick_kfs[i].kfs_pos, pick_kfs[i].dir);
		}
	}
}