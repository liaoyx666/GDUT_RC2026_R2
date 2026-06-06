#include "RC_aim.h"

namespace gantry
{
	Aim_Ctrl::Aim_Ctrl(ros::Camera& camera_,
	         gantry::Gantry& gantry_)
		: camera(camera_), gantry(gantry_),
		  user(gantry_),
		  aim_event(21, 0.1f, true, true), // EVENT_AIM = EVENT3_ID_23
		  z_lpf(0.60f, 1000.0f), y_lpf(0.60f, 1000.0f)
	{
		// AIM PID 参数 — 待调参
		z_pid.Pid_Param_Init(0.2, 0, 0., 0, 0.001, 0.001, 0.002, 0.5, 0, 0, 0, 50, 0.01);
		y_pid.Pid_Param_Init(0.2, 0, 0., 0, 0.001, 0.001, 0.002, 0.5, 0, 0, 0, 50, 0.01);
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
		float error = 0;
		float output = 0;
		float final_error_z = 0;
		float final_error_y = 0;

		/*---- 空闲：等待事件触发 ----*/
		if (phase == Phase_Idle)
		{
			if (aim_event.Is_Trig())
			{
				Tracker_Clear();
				phase = Phase_Check;
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
		/*---- 阶段0：等待相机检测到目标，5帧判稳确认数据无异常 ----*/
		case Phase_Check:
			if (camera.Event() == 0) return;                    // 相机未检测到目标，等待

	//		if (Frame_Stable(Axis_X, 5) && Frame_Stable(Axis_Y, 5) && Frame_Stable(Axis_Z, 5))
	//		{
	//			Tracker_Clear();
				phase = Phase_Yaw;
	//		}
			break;

		/*---- 阶段1：yaw角补正，PID闭环控制底盘角速度 ----*/
		case Phase_Yaw:

				phase = Phase_YZ_Coarse;

			break;

		/*---- Y-Z粗调：双轴同时大步逼近 ----*/
		case Phase_YZ_Coarse:
		{

				float error_z = Get_Data(Axis_Z);
				float error_y = Get_Data(Axis_Y);
			if (!check_error())
			{
				final_error_z = z_lpf.filter(error_z + gantry.Get_Z());
				final_error_y = y_lpf.filter(error_y + gantry.Get_Y());

				user.Set_Z(final_error_z);
				user.Set_Y(final_error_y);
			}
			
			if (Frame_Stable(Axis_Z, COARSE_FRAME_COUNT, 0.02) &&
			    Frame_Stable(Axis_Y, COARSE_FRAME_COUNT, 0.02) &&
			    fabsf(error_z) < 0.05 &&
			    fabsf(error_y) < 0.05 )
			{
				Tracker_Clear();
				phase = Phase_Z;
			}
			break;
		}

		/*---- 阶段2：z补正，PID闭环控制Gantry Z轴 ----*/
		case Phase_Z:
			if (!check_error())
			{			
				error  = Get_Data(Axis_Z);
				final_error_z = z_lpf.filter(error + gantry.Get_Z());
				user.Set_Z(final_error_z);
			}
			
			if (Frame_Stable(Axis_Z))
			{
				Tracker_Clear();
				if (fabsf(error) < 0.02f)
					phase = Phase_Y;
			}
			break;

		/*---- 阶段3：y补正，PID闭环控制Gantry Y轴 ----*/
		case Phase_Y:
			if (!check_error())
			{
				error  = Get_Data(Axis_Y);
				final_error_y = y_lpf.filter(gantry.Get_Y() + error);
				user.Set_Y(final_error_y);
			}
			
			if (Frame_Stable(Axis_Y))
			{
				y_result = Get_Data(Axis_Y);
				if (fabsf(error) < 0.02f)
					phase = Phase_Done;
			}
			break;

		/*---- 阶段5：对准完成 ----*/
		case Phase_Done:
			if(finish_flag)
			{
				if(!timer_flag)
				{
					timer_flag = 1;
					last_time = timer::Timer::Get_TimeStamp();
				}
				
				if(timer::Timer::Get_DeltaTime(last_time) > 1000000)
				{
					aim_event.Finish();        // 正式: 通知导航aim完成
					user.Give_Control();
					Tracker_Clear();
					phase = Phase_Idle;          // 持续瞄准: 直接回到粗调循环; 正式改为 Phase_Idle
					
				}
				
				if (camera.Is_QR_Enabled())
				{
					camera.QR_Disable();  // 正式: 通知上位机关闭QR
				}
     	  

			
			}

			break;
		default:
			phase = Phase_Idle;
			break;
		}
	}
}
