#pragma once
#include "RC_camera.h"
#include "RC_pid.h"
#include "RC_chassis.h"
#include "RC_vector2d.h"
#include "RC_gantry.h"
#include "RC_filter.h"
#include "RC_event3.h"

#ifdef __cplusplus
namespace aim
{
	class Aim_Ctrl
	{
	public:
		static constexpr uint16_t DEFAULT_FRAME = 60;
		static constexpr float   DEFAULT_ERROR = 0.002f;

		static constexpr float    COARSE_STABLE_THRESHOLD = 0.002f;
		static constexpr uint16_t COARSE_FRAME_COUNT      = 20;

		float final_error_z =0;
	  float final_error_y =0;

		Aim_Ctrl(ros::Camera& camera_,
		         gantry::Gantry& gantry_,
		         pid::Pid& z_pid_, pid::Pid& y_pid_ );

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
		bool Is_Done() const { return phase == Phase_Done; }
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
		gantry::Gantry&  gantry;
		gantry::GantryUser user;
		pid::Pid&        z_pid;
		pid::Pid&        y_pid;

		path::Event3     aim_event;

		filter::SecondOrderLPF z_lpf;
		filter::SecondOrderLPF y_lpf;

		enum Phase : uint8_t
		{
			Phase_Idle,
			Phase_Check,
			Phase_Yaw,
			Phase_YZ_Coarse,
			Phase_Z,
			Phase_Y,
			Phase_Done
		};
		Phase  phase    = Phase_Idle;
		float  y_result = 0;
	};
}
#endif
