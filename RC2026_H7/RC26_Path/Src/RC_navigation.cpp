#include "RC_navigation.h"

namespace path
{
	Navigation::Navigation(GraphPlan& plan_) : plan(plan_), task::ManagedTask("NavigationTask", 19, 128, task::TASK_DELAY, 2)
	{
		head = 0;
		tail = 0;
		is_start = false;
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
	
	void Navigation::Task_Process()
	{
		if (!is_start)
		{
			//	全局起点
			vector2d::Vector2D start_point(0.42, -4.53);
			plan.plan.Add_Start_Point(start_point);
			
			last_navp.p = start_point;
			last_navp.yaw = 0;
			
			is_start = true;
		}
		else
		{
			if (plan.Plan(last_navp, dst[head]))
			{
				last_navp = dst[head].nav;
			}
			
			Delete_Dst();
		}
	}
}