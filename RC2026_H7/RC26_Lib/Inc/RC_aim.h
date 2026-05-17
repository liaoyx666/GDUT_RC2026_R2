#pragma once
#include "RC_camera.h"
#include "RC_pid.h"
#include "RC_chassis.h"
#include "RC_vector2d.h"
#include "RC_gantry.h"

#ifdef __cplusplus
namespace aim
{
	class Aim_Ctrl
	{
	public:
		static constexpr uint16_t DEFAULT_FRAME = 60;
		static constexpr float   DEFAULT_ERROR = 0.005f;

		Aim_Ctrl(ros::Camera& camera_, chassis::Chassis& chassis_,
	         gantry::Gantry& gantry_,
	         pid::Pid& yaw_pid_, pid::Pid& z_pid_, pid::Pid& y_pid_);

		enum Axis : uint8_t
		{
			Axis_X,
			Axis_Y,
			Axis_Z
		};

		// 帧判稳 — 相机中断到场才推进窗口
		bool Frame_Stable(Axis axis, uint16_t frame_count = DEFAULT_FRAME, float error = DEFAULT_ERROR)
		{
			if (frame_count > 64) frame_count = 64;

			Stable_Tracker& tracker = axis_tracker[axis];

			// 无新帧 → 返回缓存结果
			if (!camera.Check_New_Data())
				return tracker.stable;

			float current = Get_Data(axis);

			tracker.sum += current;
			if (tracker.frame_count < frame_count)
			{
				tracker.buffer[tracker.frame_count++] = current;
				return tracker.stable = false;
			}

			tracker.sum -= tracker.buffer[tracker.index];
			tracker.buffer[tracker.index] = current;
			tracker.index = (tracker.index + 1) % frame_count;

			float average = tracker.sum / (float)frame_count;
			return tracker.stable = (fabsf(current - average) <= error);
		}

		void Reset();
		void Run();
		float Get_Y() const { return y_result; }

	private:
		float Get_Data(Axis axis);
		void Tracker_Clear();

		struct Stable_Tracker
		{
			float    buffer[64];
			float    sum         = 0;
			uint16_t frame_count = 0;
			uint16_t index       = 0;
			bool     stable      = false;
		} axis_tracker[4];

		ros::Camera&     camera;
		chassis::Chassis& chassis;
		gantry::Gantry&  gantry;
		pid::Pid&        yaw_pid;
		pid::Pid&        z_pid;
		pid::Pid&        y_pid;

		enum Phase : uint8_t
		{
			Phase_Check,
			Phase_Yaw,
			Phase_Z,
			Phase_Y,
			Phase_Done
		};
		Phase  phase    = Phase_Check;
		float  y_result = 0;
	};
}
#endif
