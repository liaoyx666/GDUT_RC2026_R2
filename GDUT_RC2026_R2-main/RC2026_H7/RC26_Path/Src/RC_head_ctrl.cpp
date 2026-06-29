#include "RC_head_ctrl.h"

namespace path
{
	HeadCtrl::HeadCtrl(data::RobotPose& pose_, chassis::Chassis& c, float deadzone_) : /*task::ManagedTask("HeadCtrlTask", 30, 128, task::TASK_PERIOD, 1), */pose(pose_), chassis(c)
	{
		pid.Init(4, 0, 6, 0.4, 4.4, deadzone_);
		
		is_enable = false;
	}
	
	
	
}