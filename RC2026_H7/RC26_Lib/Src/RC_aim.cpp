#include "RC_aim.h"

namespace aim
{
	Aim_Ctrl::Aim_Ctrl(ros::Camera& camera_, chassis::Chassis& chassis_, pid::Pid& yaw_pid_, pid::Pid& x_pid_)
		: camera(camera_), chassis(chassis_), yaw_pid(yaw_pid_), x_pid(x_pid_)
	{

	}

	float Aim_Ctrl::Get_Data(Axis axis)
	{
		switch (axis)
		{
		case Axis_X:   return camera.X();
		case Axis_Y:   return camera.Y();
		case Axis_Z:   return camera.Z();
		case Axis_Yaw: return camera.Yaw();
		default:       return 0;
		}
	}

	void Aim_Ctrl::Tracker_Clear()
	{
		for (uint8_t i = 0; i < 4; i++) axis_tracker[i] = Stable_Tracker();
	}

	void Aim_Ctrl::Reset()
	{
		Tracker_Clear();
		phase    = Phase_Check;
		y_result = 0;
	}

	void Aim_Ctrl::Run()
	{
		float error = 0;
		float output = 0;

		switch (phase)
		{
		/*---- 阶段0：等待相机检测到目标，5帧判稳确认数据无异常 ----*/
		case Phase_Check:
			if (camera.Event() == 0) return;                    // 相机未检测到目标，等待

			if (Frame_Stable(Axis_X, 5) && Frame_Stable(Axis_Y, 5)       // x/y/z/yaw 四轴均5帧稳定
				&& Frame_Stable(Axis_Z, 5) && Frame_Stable(Axis_Yaw, 5))
			{
				Tracker_Clear();
				phase = Phase_Yaw;
			}
			break;

		/*---- 阶段1：yaw角补正，PID闭环控制底盘角速度 ----*/
		case Phase_Yaw:
			error  = camera.Yaw();                              // yaw误差，目标为0
			output = yaw_pid.Pid_Calculate(error, 0);           // PID解算角速度
			chassis.Set_Ang_Vel(output);                        // 驱动底盘旋转

			if (Frame_Stable(Axis_Yaw))                                // yaw判稳
			{
				chassis.Set_Ang_Vel(0);                         // 停止旋转
				Tracker_Clear();
				phase = Phase_X;
			}
			break;

		/*---- 阶段2：x补正，PID闭环控制底盘世界x方向速度 ----*/
		case Phase_X:
			error  = camera.X();                                // x误差，目标为0
			output = x_pid.Pid_Calculate(error, 0);             // PID解算线速度
			chassis.Set_World_Lin_Vel(vector2d::Vector2D(output, 0));// 驱动底盘平移

			if (Frame_Stable(Axis_X))                                // x判稳
			{
				chassis.Set_World_Lin_Vel(vector2d::Vector2D(0, 0));// 停止平移
				Tracker_Clear();
				phase = Phase_Z;
			}
			break;

		/*---- 阶段3：z补正（上层机构执行，暂不实现） ----*/
		case Phase_Z:
			Tracker_Clear();
			phase = Phase_Y;
			break;

		/*---- 阶段4：y补正，无PID，判稳后存储结果 ----*/
		case Phase_Y:
			if (Frame_Stable(Axis_Y))                                // y判稳
			{
				y_result = camera.Y();                          // 存储稳定后的y值
				phase = Phase_Done;
			}
			break;

		/*---- 阶段5：对准完成 ----*/
		case Phase_Done:
		default:
			break;
		}
	}
}
