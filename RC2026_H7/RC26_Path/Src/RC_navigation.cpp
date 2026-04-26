#include "RC_navigation.h"

namespace path
{
	Navigation::Navigation(data::RobotPose& pose_) : pose(pose_)
	{
		head = 0;
		tail = 0;
	}
	
	bool Navigation::Add_Dst(NavPoint nav_, DstType type_, Event3_t event_)
	{
		if (Dst_FreeSpace() == 0) return false;
		
		dst[tail].nav = nav_;
		dst[tail].type = type_;
		dst[tail].event = event_;
		
		tail = (tail + 1) % NAVIGATION_MAX_DESTINATION;
		return true;
	}
	
	void Navigation::Delete_Dst()
	{
		if (Dst_Num() != 0)
		{
			head = (head + 1) % NAVIGATION_MAX_DESTINATION;
		}
	}
	
	
	
}