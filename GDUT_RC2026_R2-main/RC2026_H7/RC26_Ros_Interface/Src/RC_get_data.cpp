#include "RC_get_data.h"

namespace ros
{
	GetData::GetData(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_)
	{
		is_blue_side		= 0;
		kfs_num             = 0;
		weapon_num          = 0;
		is_dock             = 0;
		pick_weapon_num     = 0;
		boot_area           = 0;
	}
	
	
	
}