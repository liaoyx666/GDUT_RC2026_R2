#include "RC_path2.h"

namespace path
{
	
	
	
	
	
	/*-------------------------------------------------------------------------------------------*/
	
	
	bool PathPlan2::Add_One_Point()
	{
		uint16_t free_space = (point_head - point_tail - 1 + MAX_PATHPOINT_NUM) % MAX_PATHPOINT_NUM;
		
		if (free_space <= 0)
		{
			return false;
		}
		else
		{
			point_tail = (point_tail + 1) % MAX_PATHPOINT_NUM;
			return true;
		}
	}
	
	
	void PathPlan2::Add_End_Point(
		vector2d::Vector2D end_point_, 
		float target_yaw_, 
		float leave_target_yaw_,
		float linear_vel,
		float linear_accel,
		float linear_decel,
		float angular_vel,
		float angular_accel,
		float angular_decel
	)
	{
		
	}
		
	
	
	void PathPlan2::Add_Point(
		vector2d::Vector2D point, 
		float target_yaw_, 
		float smoothness_,
		float linear_vel,
		float linear_accel,
		float linear_decel,
		float angular_vel,
		float angular_accel,
		float angular_decel
	)
	{

	}
	
	
	
	
	
	
	
}