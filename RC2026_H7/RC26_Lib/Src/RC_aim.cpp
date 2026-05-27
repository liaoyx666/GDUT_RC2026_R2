#include "RC_aim.h"

namespace aim
{
	Aim_Ctrl::Aim_Ctrl(ros::Camera& camera_,
	         gantry::Gantry& gantry_,
	         pid::Pid& z_pid_, pid::Pid& y_pid_)
		: camera(camera_), gantry(gantry_),
		  user(gantry_),
		  z_pid(z_pid_), y_pid(y_pid_),
		  aim_event(22, 0.1f, true, true),
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

	void Aim_Ctrl::Run()
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

		camera.Send_QR_Req();

		// 四轴全零 = 相机未识别到目标，数据不予采纳
		if (fabsf(camera.X()) < 1e-6f && fabsf(camera.Y()) < 1e-6f
		 && fabsf(camera.Z()) < 1e-6f && fabsf(camera.Yaw()) < 1e-6f)
			return;

		if (!user.Take_Control()) return;

		switch (phase)
		{
		/*---- 阶段0：等待相机检测到目标，5帧判稳确认数据无异常 ----*/
		case Phase_Check:
			if (camera.Event() == 0) return;                    // 相机未检测到目标，等待

			if (Frame_Stable(Axis_X, 5) && Frame_Stable(Axis_Y, 5) && Frame_Stable(Axis_Z, 5))
			{
				Tracker_Clear();
				phase = Phase_Yaw;
			}
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

			final_error_z = z_lpf.filter(error_z + gantry.Get_Z());
			final_error_y = y_lpf.filter(error_y + gantry.Get_Y());

			user.Set_Z(final_error_z);
			user.Set_Y(final_error_y);

			if (Frame_Stable(Axis_Z, COARSE_FRAME_COUNT, 0.02) &&
			    Frame_Stable(Axis_Y, COARSE_FRAME_COUNT, 0.02) &&
			    fabsf(error_z) < 0.05 &&
			    fabsf(error_y) < 0.05)
			{
				Tracker_Clear();
				phase = Phase_Z;
			}
			break;
		}

		/*---- 阶段2：z补正，PID闭环控制Gantry Z轴 ----*/
		case Phase_Z:
			error  = Get_Data(Axis_Z);
			final_error_z = z_lpf.filter(error + gantry.Get_Z());
			user.Set_Z(final_error_z);

			if (Frame_Stable(Axis_Z))
			{
				Tracker_Clear();
				if (fabsf(error) < 0.002f)
					phase = Phase_Y;
			}
			break;

		/*---- 阶段3：y补正，PID闭环控制Gantry Y轴 ----*/
		case Phase_Y:
			error  = Get_Data(Axis_Y);
			final_error_y = y_lpf.filter(gantry.Get_Y() + error);
			user.Set_Y(final_error_y);

			if (Frame_Stable(Axis_Y))
			{
				y_result = Get_Data(Axis_Y);
				if (fabsf(error) < 0.002f)
					phase = Phase_Done;
			}
			break;

		/*---- 阶段5：对准完成 ----*/
		case Phase_Done:
			aim_event.Finish();
			user.Give_Control();
			phase = Phase_Idle;
			break;
		default:
			phase = Phase_Idle;
			break;
		}
	}
}
