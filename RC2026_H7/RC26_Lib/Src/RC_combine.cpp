#include "RC_combine.h"

namespace combine
{
	Combine::Combine(
		chassis::Chassis& chassis_, 
		chassis::LiftChassis& lift_,
		path::PathPlan3& plan_,
		fusion::QEO& qeo_,
		path::Navigation& nav_,
		data::RobotPose& pose_
	) : combine_event(22, 0.02f, false, false), chassis(chassis_), lift(lift_), plan(plan_), qeo(qeo_), nav(nav_), pose(pose_)
	{
		is_combine = false;
		uncombine_flag = false;
		
		last_time = 0;
		
		unc_state = UNCOMBINE_DIS;
	}
	
	void Combine::Auto_Combine()
	{
		if (combine_event.Is_Trig())
		{
			if (!is_combine)
			{
				chassis.Chassis_Disable();
				is_combine = true;
				uncombine_flag = false;
			}
			else
			{
				uncombine_flag = true;
			}
		}
		
		
		
		
		if (uncombine_flag)
		{
			switch (unc_state)
			{
				case UNCOMBINE_DIS:
				{
					chassis.Chassis_Enable();
					plan.Disable();
					qeo.Set_X(0);
					qeo.Set_Y(0);
					path::Event3::Trig_Event(EVENT_DOWN_4_READY_R);
					
					last_time = timer::Timer::Get_TimeStamp();
					
					unc_state = UNCOMBINE_RESET;
					break;
				}
				
				case UNCOMBINE_RESET:
				{
					if (timer::Timer::Get_DeltaTime(last_time) > 1000000)
					{
						unc_state = UNCOMBINE_MOVE;
					}
					
					break;
				}
				
				
				
				case UNCOMBINE_MOVE:
				{
					if (qeo.Y() > target_y)
					{
						chassis.Set_Robot_Lin_Vel(vector2d::Vector2D(0, -0.3));
					}
					else
					{
						unc_state = UNCOMBINE_END;
					}
					break;
				}
				
				case UNCOMBINE_END:
				{
					plan.Force_End(); // 强制上一路径结束
					
					vector2d::Vector2D p = vector2d::Vector2D(pose.X(), pose.Y());
					
					
					nav.Update_Last_Navp(p, pose.Yaw()); // 更新起点
					nav.Open(); // 允许添加目的地
					
					//Update_Last_Navp(p, yaw); // 更新起点
				//	Open(); // 允许添加目的地
					vector2d::Vector2D dir = vector2d::Vector2D(pose.Y() - HALF_PI) * 0.2f; // 移动方向
					
				//	Event3::Trig_Event(EVENT_COMBINE | EVENT_DOWN_4_READY_R);// 直接触发合体事件和下台阶事件，实现解体
					
					nav.Go_To_Do(p + dir, pose.Yaw(), EVENT3_NULL);
					

					plan.Enable();
					uncombine_flag = false;
					is_combine = false;
					unc_state = UNCOMBINE_DIS;
					break;
				}
				
				default:
				{
					break;
				}
			}
			
			
			
			
			
			
			
			
			
			
//			if (!plan.Is_End())
//			{
//				chassis.Chassis_Enable();
//				uncombine_flag = false;
//				is_combine = false;
//			}
		}
	}

}
