#include "RC_aim.h"

namespace gantry
{
	Aim_Ctrl::Aim_Ctrl(ros::Camera& camera_,
		gantry::Gantry& gantry_,
		chassis::Chassis& chassis_,
		gantry::Gripper& gripper_)
		: camera(camera_), gantry(gantry_),
		  chassis(chassis_),
		  gripper(gripper_),
		  user(gantry_),
		  aim_event(21, 0.1f, true, true), // EVENT_AIM = EVENT3_ID_21
		  z_lpf(0.60f, 1000.0f), y_lpf(0.60f, 1000.0f)
	{
	}

	float Aim_Ctrl::Get_Data(Axis axis)
	{
		switch (axis)
		{
		case Axis_X:   return camera.X();
		case Axis_Y:   return camera.Y();
		case Axis_Z:   return camera.Z();
		default:       return 0;
		}
	}

	void Aim_Ctrl::Tracker_Clear()
	{
		for (uint8_t i = 0; i < 4; i++) axis_tracker[i] = Stable_Tracker();
	}

	void Aim_Ctrl::Reset()
	{
		user.Give_Control();
		Tracker_Clear();
		phase    = Phase_Idle;
		y_result = 0;
	}

	void Aim_Ctrl::Auto_Aim()
	{
		/*---- 空闲：等待事件触发 ----*/
		if (phase == Phase_Idle)
		{
			if (aim_event.Is_Trig())
			{
				Tracker_Clear();
				phase = Phase_PrePosition;
			}
			return;
		}

		/* 等待上位机确认 QR 通道就绪 */
		if (!camera.Is_QR_Enabled())
		{
			camera.QR_Enable();
			return;
		}

		if (!user.Take_Control()) return;

		switch (phase)
		{
		/*---- 阶段0：预定位，将龙门架移至预设起始位置 ----*/
		case Phase_PrePosition:
		{
			user.Set_X(PRE_POS_X);
			user.Set_Y(PRE_POS_Y);
			user.Set_Z(PRE_POS_Z);

			float dx = fabsf(gantry.Get_X() - PRE_POS_X);
			float dy = fabsf(gantry.Get_Y() - PRE_POS_Y);
			float dz = fabsf(gantry.Get_Z() - PRE_POS_Z);

			if (dx < PRE_POS_THRESHOLD && dy < PRE_POS_THRESHOLD && dz < PRE_POS_THRESHOLD)
			{
				Tracker_Clear();
				phase = Phase_Check;
			}
			break;
		}

		/*---- 阶段1：等待相机检测到目标 ----*/
		case Phase_Check:

			if (camera.Event() == 0) return;                    // 相机未检测到目标，等待

				phase = Phase_Yaw;

			break;

		/*---- 阶段1：yaw角补正，PID闭环控制底盘角速度 ----*/
		case Phase_Yaw:

				chassis.Force_Lin_Vel_Zero(4);

				phase = Phase_YZ_Coarse;

			break;

		/*---- Y-Z粗调：双轴同时大步逼近 ----*/
		 case Phase_YZ_Coarse:
		 {
		// 	float error_z = Get_Data(Axis_Z);
		// 	float error_y = Get_Data(Axis_Y);
		//
		// 	if (!check_error())
		// 	{
		// 		final_error_z = z_lpf.filter(error_z + gantry.Get_Z());
		// 		final_error_y = y_lpf.filter(error_y + gantry.Get_Y());
		//
		// 		user.Set_Z(final_error_z);
		// 		user.Set_Y(final_error_y);
		// 	}
		// 	else break;
		//
		// 	bool z_stable = Frame_Stable(Axis_Z, COARSE_FRAME_COUNT, COARSE_STABLE_THRESHOLD);
		// 	bool y_stable = Frame_Stable(Axis_Y, COARSE_FRAME_COUNT, COARSE_STABLE_THRESHOLD);
		//
		// 	if (z_stable && y_stable &&
		// 	    fabsf(error_z) < COARSE_STABLE_THRESHOLD &&
		// 	    fabsf(error_y) < COARSE_STABLE_THRESHOLD)
		// 	{
		// 		Tracker_Clear();
		 		phase = Phase_Z;
		// 	}
		// 	break;
		 }

		/*---- 阶段2：y补正，PID闭环控制Gantry Y轴 ----*/
		case Phase_Y:
			if (!check_error())
			{
				error  = Get_Data(Axis_Y);
				final_error_y = y_lpf.filter(gantry.Get_Y() + error);
				user.Set_Y(final_error_y);
			}
			else 
				break;

			if (Frame_Stable(Axis_Y))
			{
				Tracker_Clear();
				if (fabsf(error) < 0.001f)
					phase = Phase_Z;
			}
			break;
		 
		/*---- 阶段3：z补正，PID闭环控制Gantry Z轴 ----*/
		case Phase_Z:
			if (!check_error())
			{
				error  = Get_Data(Axis_Z);
				final_error_z = z_lpf.filter(error + gantry.Get_Z());
				user.Set_Z(final_error_z);
			}
			else 
				break;
			
			if (Frame_Stable(Axis_Z))
			{
				Tracker_Clear();
				if (fabsf(error) < 0.001f)
					phase = Phase_Y2;
			}
			break;
			
		case Phase_Y2:
			if (!check_error())
			{
				error  = Get_Data(Axis_Y);
				final_error_y = y_lpf.filter(gantry.Get_Y() + error);
				user.Set_Y(final_error_y);
			}
			else 
				break;

			if (Frame_Stable(Axis_Y))
			{
				Tracker_Clear();
				if (fabsf(error) < 0.001f)
					phase = Phase_Done;
			}
			break;

		/*---- 阶段5：对准完成 ----*/
		case Phase_Done:
			if(finish_flag)
			{
				gripper.Open();

				if(!timer_flag)
				{
					last_time = timer::Timer::Get_TimeStamp();
					timer_flag = 1;
				}
				
				if (camera.Is_QR_Enabled())
				{
					camera.QR_Disable();
				}
				
				if(timer::Timer::Get_DeltaTime(last_time) > 3000000)
				{
					chassis.Unforce_Lin_Vel_Zero(4);
					user.Set_Reset_Pos();
					Tracker_Clear();
					user.Give_Control();
					aim_event.Finish();
					timer_done = 1;
					phase = Phase_Idle;
				}
			}

			break;
		default:
			phase = Phase_Idle;
			break;
		}
	}
}
