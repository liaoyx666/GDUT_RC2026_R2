#pragma once
#ifdef __cplusplus

#include "RC_event3.h"
#include "RC_chassis.h"
#include "RC_lift_chassis.h"
#include "RC_navigation.h"
#include "RC_path_plan3.h"
#include "RC_QEO.h"
#include "RC_data_pool.h"
#include "RC_timer.h"

namespace combine
{
	enum UncombineState : uint8_t
	{
		UNCOMBINE_DIS = 0,
		UNCOMBINE_RESET,
		UNCOMBINE_MOVE,
		UNCOMBINE_END,
	};
	
	
	
	
	
	
	class Combine
    {
    public:
		Combine(
			chassis::Chassis& chassis_, 
			chassis::LiftChassis& lift_,
			path::PathPlan3& plan_,
			fusion::QEO& qeo_,
			path::Navigation& nav_,
			data::RobotPose& pose_
		);
		~Combine() = default;
	
		bool Is_Combine() const { return is_combine; }
	
		void Auto_Combine();
    private:
		path::Event3 combine_event;
		chassis::Chassis& chassis;
		chassis::LiftChassis& lift;
		
		bool uncombine_flag;
	
		UncombineState unc_state;
	
		path::PathPlan3& plan;
	
		fusion::QEO& qeo;
	
		data::RobotPose& pose;
	
		path::Navigation& nav;
	
		uint32_t last_time;
	
		const float target_y = -0.95f;
	
		bool is_combine;
    };

}
#endif
