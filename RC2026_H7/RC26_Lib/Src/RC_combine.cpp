#include "RC_combine.h"

namespace combine
{
	Combine::Combine(
		chassis::Chassis& chassis_, 
		chassis::LiftChassis& lift_,
		path::PathPlan3& plan_
	) : combine_event(22, 0.02f, false, false), chassis(chassis_), lift(lift_), plan(plan_)
	{
		is_combine = false;
		uncombine_flag = false;
	}
	
	void Combine::Auto_Combine()
	{
		if (combine_event.Is_Trig())
		{
			if (!is_combine)
			{
				chassis.Chassis_Disable();
				is_combine = true;
			}
			else
			{
				uncombine_flag = true;
			}
		}
		
		
		
		
		if (uncombine_flag)
		{
			if (!plan.Is_End())
			{
				chassis.Chassis_Enable();
				uncombine_flag = false;
				is_combine = false;
			}
		}
	}

}
