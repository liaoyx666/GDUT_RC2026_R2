#include "RC_head_ctrl.h"

namespace path
{
	HeadCtrl::HeadCtrl(data::RobotPose& pose_, chassis::Chassis& c, float deadzone_) : task::ManagedTask("HeadCtrlTask", 30, 128, task::TASK_PERIOD, 1), pose(pose_), chassis(c)
	{
		pid.Init(2.3, 0, 5, 0.1, 3, deadzone_);
		
		is_enable = false;
	}
	
	
	void HeadCtrl::Task_Process()
	{
		if (is_enable)
		{
			float vw = pid.NPid_Calculate(target_yaw, pose.Yaw(), true, PI);
			
			chassis.Set_Ang_Vel(vw);
		}
	}
}