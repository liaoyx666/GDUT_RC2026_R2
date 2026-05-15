#include "RC_auto_lift.h"


namespace chassis
{
	AutoLift::AutoLift(LiftChassis& lift_, path::TrajTrack3& track_, data::RobotPose& pose_)
	 : check_event{
		path::Event3(1, false, 0.07f),		// EVENT_HEAD_CHECK_F
		path::Event3(2, false, 0.07f),      // EVENT_HEAD_CHECK_B
		path::Event3(3, false, 0.07f),      // EVENT_HEAD_CHECK_L
		path::Event3(4, false, 0.07f),      // EVENT_HEAD_CHECK_R
	}, lift_event{
		path::Event3(5 , false, 0.1f),		// EVENT_UP_2_READY_L
		path::Event3(6 , false, 0.3f),     // EVENT_UP_4_READY_L
		path::Event3(7 , false, 0.1f),     // EVENT_UP_2_READY_R
		path::Event3(8 , false, 0.3f),     // EVENT_UP_4_READY_R
		path::Event3(9 , false, 0.1f),     // EVENT_DOWN_2_READY_L
		path::Event3(10, false, 0.1f),     // EVENT_DOWN_4_READY_L
		path::Event3(11, false, 0.1f),     // EVENT_DOWN_2_READY_R
		path::Event3(12, false, 0.1f)      // EVENT_DOWN_4_READY_R
	}, /*task::ManagedTask("AutoLiftTask", 21, 150, task::TASK_DELAY, 1), */track(track_), pose(pose_), lift(lift_)
	{
		check_flag = false;
		lift_trig = false;
		yaw = 0;
		check_dx = 0;
	}
	
	
}
