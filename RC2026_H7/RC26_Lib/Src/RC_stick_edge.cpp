#include "RC_stick_edge.h"

StickEdge::StickEdge(
		chassis::Omni4Chassis& c_, 
		path::PathPlan3& p_, 
		gantry::Gantry& gan_,
		gantry::Gripper& gripper_,
		data::RobotPose& pose_
	)
: 	stick_l_event(26, 0.01f, true, true),
	c(c_),
	p(p_),
	user(gan_),
	gripper(gripper_),
	ir_cmd(1),
	pose(pose_)
{
	state = STICK_DOCK_RESET;
	last_time = 0;
}