#include "RC_stick_edge.h"

StickEdge::StickEdge(
		chassis::Omni4Chassis& c_, 
		path::PathPlan3& p_, 
		gantry::Gantry& gan_,
		gantry::Gripper& gripper_,
		data::RobotPose& pose_,
		ws2812::Ws2812B& led_
	)
: 	stick_l_event(26, 0.01f, true, true),
	stick_l_event_2(27, 0.01f, true, true),
	c(c_),
	p(p_),
	user(gan_),
	gripper(gripper_),
	open_cmd(1, led_),
	go_cmd(9, led_),
	pose(pose_)
{
	state = STICK_DOCK_RESET;
	last_time = 0;
}