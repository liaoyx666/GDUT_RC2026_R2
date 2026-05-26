#include "RC_head_check.h"

namespace check
{
	HeadCheck::HeadCheck(path::TrajTrack3& track_, data::RobotPose& pose_)
	: check_event{
		path::Event3(1, 0.09f, false, false),				// EVENT_HEAD_CHECK_F
		path::Event3(2, 0.09f, false, false),   		  	// EVENT_HEAD_CHECK_B
		path::Event3(3, 0.09f, false, false),   		  	// EVENT_HEAD_CHECK_L
		path::Event3(4, 0.09f, false, false),   		  	// EVENT_HEAD_CHECK_R
	}, track(track_), pose(pose_)
	{
		check_flag = false;
		yaw = 0;
		check_dx = 0;
	}
	
}
