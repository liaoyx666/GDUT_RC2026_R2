#include "RC_map.h"



namespace ros
{
	Map::Map(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_)
	{
		memset(map, -1, 12);// 初始化为未知
	}
	
	void Map::CDC_Receive_Process(uint8_t *buf, uint16_t len)
	{
		
	}
	

	
}