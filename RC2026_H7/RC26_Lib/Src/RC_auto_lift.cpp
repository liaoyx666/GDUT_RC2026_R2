#include "RC_auto_lift.h"


namespace chassis
{
	AutoLift::AutoLift(LiftChassis& lift_, path::TrajTrack3& track_, data::RobotPose& pose_)
	 : check_event{
		path::Event3(1, 0.07f, false, false),		// EVENT_HEAD_CHECK_F
		path::Event3(2, 0.07f, false, false),      // EVENT_HEAD_CHECK_B
		path::Event3(3, 0.07f, false, false),      // EVENT_HEAD_CHECK_L
		path::Event3(4, 0.07f, false, false),      // EVENT_HEAD_CHECK_R
	}, lift_event{      
		path::Event3(5 , 0.1f, false, false),		// EVENT_UP_2_READY_L
		path::Event3(6 , 0.3f, false, false),     // EVENT_UP_4_READY_L
		path::Event3(7 , 0.1f, false, false),     // EVENT_UP_2_READY_R
		path::Event3(8 , 0.3f, false, false),     // EVENT_UP_4_READY_R
		path::Event3(9 , 0.1f, false, false),     // EVENT_DOWN_2_READY_L
		path::Event3(10, 0.1f, false, false),     // EVENT_DOWN_4_READY_L
		path::Event3(11, 0.1f, false, false),     // EVENT_DOWN_2_READY_R
		path::Event3(12, 0.1f, false, false)      // EVENT_DOWN_4_READY_R
	}, /*task::ManagedTask("AutoLiftTask", 21, 150, task::TASK_DELAY, 1), */track(track_), pose(pose_), lift(lift_)
	{
		check_flag = false;
		lift_trig = false;
		yaw = 0;
		check_dx = 0;
	}
	
	
}
