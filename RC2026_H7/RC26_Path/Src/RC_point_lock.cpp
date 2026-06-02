#include "RC_point_lock.h"

namespace path
{
	PointLock::PointLock(data::RobotPose& pose_, chassis::Chassis& chassis_, PathPlan3& path_)
	: pose(pose_), chassis(chassis_), path(path_)
	{
		Reset();
	}
}