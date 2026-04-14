#pragma once
#include "RC_nonlinear_pid.h"
#include "RC_task.h"
#include "RC_data_pool.h"
#include "RC_chassis.h"

#ifdef __cplusplus
namespace path
{
	class HeadCtrl : public task::ManagedTask
    {
    public:
		HeadCtrl(data::RobotPose& pose, chassis::Chassis& c, float deadzone_);
	
		pid::NonlinearPid pid;
	
		const bool& Is_Enable() const {return is_enable;}
		
		void Set_Yaw(float y_)
		{
			if (y_ <= -PI || y_ > PI) y_ = PI;
			target_yaw = y_;
		}
		
		void Enable() {is_enable = true;}
		void Disable() {is_enable = false;}
		
    protected:
		void Task_Process() override;
    private:
		float target_yaw;
		data::RobotPose& pose;
		chassis::Chassis& chassis;
		bool is_enable;
    };
}
#endif
