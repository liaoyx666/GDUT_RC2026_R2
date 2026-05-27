#include "RC_KFS_pose.h"

namespace ros
{
	KfsPose::KfsPose(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_)
	{
		
	}
}