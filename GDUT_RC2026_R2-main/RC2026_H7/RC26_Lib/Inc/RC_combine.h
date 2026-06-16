#pragma once
#ifdef __cplusplus

#include "RC_event3.h"
#include "RC_chassis.h"
#include "RC_lift_chassis.h"
#include "RC_navigation.h"
#include "RC_path_plan3.h"

namespace combine
{
	
	class Combine
    {
    public:
		Combine(
			chassis::Chassis& chassis_, 
			chassis::LiftChassis& lift_,
			path::PathPlan3& plan_
		);
		~Combine() = default;
	
		bool Is_Combine() const { return is_combine; }
	
		void Auto_Combine();
    private:
		path::Event3 combine_event;
		chassis::Chassis& chassis;
		chassis::LiftChassis& lift;
		
		bool uncombine_flag;
	
	
		path::PathPlan3& plan;
	
		bool is_combine;
    };

}
#endif
