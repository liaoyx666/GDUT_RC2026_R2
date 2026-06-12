#pragma once
#ifdef __cplusplus
#include "RC_nonlinear_pid.h"
#include "RC_data_pool.h"
#include "RC_chassis.h"
#include "RC_path_plan3.h"

namespace path
{
	/*底盘锁点*/
	class PointLock
    {
    public:
		PointLock(data::RobotPose& pose_, chassis::Chassis& chassis_, PathPlan3& path_);
		~PointLock() = default;
		
		void Point_Lock()
		{
			if (is_enable)
			{
				float vel_x = pid_x.NPid_Calculate(target_x, pose.X());
				float vel_y = pid_y.NPid_Calculate(target_y, pose.Y());
				
				chassis.Set_World_Lin_Vel(vector2d::Vector2D(vel_x, vel_y));
			}
		}
		
		void Enable_And_Disable_PathPlan()
		{
			path.Disable();
			is_enable = true;
		}
		
		void Disable_And_Enable_PathPlan()
		{
			is_enable = false;
			path.Enable();
		}
		
		constexpr void Disable_Only() { is_enable = false; }
		
		constexpr void Set_X(float x) { target_x = x; }
		constexpr void Set_Y(float y) { target_y = y; }
	
		constexpr void Set_X_Max_Vel(float vel) { pid_x.Set_Max_Out(vel); }
	    constexpr void Set_Y_Max_Vel(float vel) { pid_y.Set_Max_Out(vel); }
		
		constexpr void Set_X_Deadzone(float deadzone) { pid_x.Set_Deadzone(deadzone); }
	    constexpr void Set_Y_Deadzone(float deadzone) { pid_y.Set_Deadzone(deadzone); }
		
		void Reset()
		{
			pid_x.Init(1.62f, 0.0f, 2.0f, 0.05f, 1.5f, 0.01f);
			pid_y.Init(1.62f, 0.0f, 2.0f, 0.05f, 1.5f, 0.01f);
		}
		
    private:
		float target_x;
		float target_y;
		
		pid::NonlinearPid pid_x;
		pid::NonlinearPid pid_y;
	
		data::RobotPose& pose;
		chassis::Chassis& chassis;
		PathPlan3& path;
		
		bool is_enable;
    };
}
#endif
