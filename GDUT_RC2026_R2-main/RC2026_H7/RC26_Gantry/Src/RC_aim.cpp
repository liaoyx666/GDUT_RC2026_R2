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
		  y_lpf(0.50f, 1000.0f),
			ir(1)
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
		phase       = Phase_Idle;
		y_result    = 0;
		timer_flag  = 0;
		timer_done  = 0;
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
		/*---- 阶段0：预定位（四轴 + Pitch 前倾 + 力矩限幅，复用 Stick 策略）----*/
		case Phase_PrePosition:
		{
			user.Set_X(DOCK_POS_X);
			user.Set_Y(DOCK_POS_Y);
			user.Set_Z(DOCK_POS_Z);
			user.Set_P_Max_T(DOCK_P_MAX_T);
			user.Set_P(DOCK_POS_P);

			float dx = fabsf(gantry.Get_X() - DOCK_POS_X);
			float dy = fabsf(gantry.Get_Y() - DOCK_POS_Y);
			float dz = fabsf(gantry.Get_Z() - DOCK_POS_Z);
			float dp = fabsf(gantry.Get_P() - DOCK_POS_P);

			if (dx < DOCK_POS_THRESHOLD && dy < DOCK_POS_THRESHOLD &&
			    dz < DOCK_POS_THRESHOLD && dp < DOCK_POS_THRESHOLD)
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

		/*---- 阶段2：yaw 补正（占位，锁底盘平动）----*/
		case Phase_Yaw:

				chassis.Force_Lin_Vel_Zero(4);

				// TODO: 基于 camera.Yaw() 闭环控制底盘角速度
				// target_w = pid.Calculate(0, camera.Yaw());
				// chassis.Set_Ang_Vel(target_w);
				phase = Phase_Y;

			break;

		/*---- 阶段3：Y 轴视觉闭环 ----*/
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
					phase = Phase_Wait;
			}
			break;

	/*---- 阶段4：等待红外确认 ----*/
	case Phase_Wait:
		if(ir.Get_Cmd())
		{
			phase = Phase_Release;
		}
		break;

	/*---- 阶段5：松爪并计时 ----*/
	case Phase_Release:
		
		gripper.Open();

		last_time = timer::Timer::Get_TimeStamp();
		phase = Phase_Done;
	
		break;

	/*---- 阶段6：计时到，恢复 ----*/
	case Phase_Done:
		
		if (camera.Is_QR_Enabled())
		{
			camera.QR_Disable();
		}

	
		if(timer::Timer::Get_DeltaTime(last_time) > 6000000)
		{
			user.Set_P_Max_T(RESTORE_P_MAX_T);  // 恢复满力矩
			chassis.Unforce_Lin_Vel_Zero(4);
			user.Set_Reset_Pos();
			Tracker_Clear();
			user.Give_Control();
			aim_event.Finish();
			timer_done = 1;
			phase = Phase_Idle;
		}
		break;
		default:
			phase = Phase_Idle;
			break;
		}
	}
}
