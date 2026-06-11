#include "RC_combine.h"

namespace combine
{
	Combine::Combine(chassis::Chassis& chassis_, chassis::LiftChassis& lift_)
	: combine_event(22, 0.02f, true, true), chassis(chassis_), lift(lift_)
	{
		is_combine = false;
	}
	
	void Combine::Auto_Combine()
	{
		if (combine_event.Is_Trig())
		{
			chassis.Chassis_Disable();
		}
		
	}

}
