#include "RC_map.h"

namespace ros
{
	Map::Map(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_)
	{
		//memset(map, -1, 12);// 初始化为未知
	}
	
	void Map::CDC_Receive_Process(uint8_t *buf, uint16_t len)
	{
		if (len == 12 && is_init == false)
		{
			memcpy(map, buf, 12);

			is_init = true;
		}
		
		// 应答
		uint8_t ack = 1;
		cdc->CDC_Send_Pkg(2, &ack, 1, 1000);
	}
	
	int8_t Map::Get_MF(uint8_t n)
	{
		if (n <= 12 && n >= 1)
		{
			return map[n - 1];
		}
		else
		{
			return 0;
		}
	}
	
	void Map::Set_MF(uint8_t n, int8_t kfs)
	{
		if (n <= 12 && n >= 1)
		{
			map[n - 1] = kfs;
		}
	}
	
	
	int8_t Map::Kfs_On_Dir(uint8_t n, Dir d)
	{
		int8_t s;
		
		// map:4x3
		if (n <= 12 && n >= 1)
		{
			switch (d)
			{
			case Dir::F:
				s = 3;
				break;
			
			case Dir::B:
				s = -3;
				break;
			
			case Dir::L:
				if (n % 3 == 1) return 0;
				s = -1;
				break;
			
			case Dir::R:
				if (n % 3 == 0) return 0;
				s = 1;
				break;
			
			default:
				return 0;
				break;
			}
		}
		else
		{
			return 0;
		}
		
		
		if (n + s <= 12 && n + s >= 1)
		{
			return map[n + s - 1];
		}
		else
		{
			return 0;
		}
	}
	
}