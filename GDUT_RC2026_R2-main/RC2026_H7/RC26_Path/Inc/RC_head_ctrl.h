#pragma once
#include "RC_nonlinear_pid.h"
#include "RC_task.h"
#include "RC_data_pool.h"
#include "RC_chassis.h"

#ifdef __cplusplus
namespace path
{
	class HeadCtrl// : public task::ManagedTask
    {
    public:
		HeadCtrl(data::RobotPose& pose, chassis::Chassis& c, float deadzone_);
	
		pid::NonlinearPid pid;
	
		bool Is_Enable() const {return is_enable;}
		
		void Set_Yaw(float y_)
		{
			if (y_ <= -PI || y_ > PI) y_ = PI;
			target_yaw = y_;
		}
		
		void Enable() {is_enable = true;}
		void Disable() {is_enable = false;}
		
		inline void Head_Ctrl()
		{
			if (is_enable)
			{
				float vw = pid.NPid_Calculate(target_yaw, pose.Yaw(), true, PI);
				
				chassis.Set_Ang_Vel(vw);
			}
		}
		
		/* 获取与目标yaw偏差 */
		float Get_Delta_Yaw()
		{
			float delta = target_yaw - pose.Yaw();
			
			if (delta > PI)
				delta -= TWO_PI;
			else if (delta < -PI)
				delta += TWO_PI;
			
			return delta;
		}
		
    protected:
		
    private:
		float target_yaw;
		data::RobotPose& pose;
		chassis::Chassis& chassis;
		bool is_enable;
    };
}
#endif
