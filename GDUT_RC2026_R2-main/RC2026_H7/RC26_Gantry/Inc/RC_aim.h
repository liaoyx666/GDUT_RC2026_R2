#pragma once
#include "RC_camera_dock.h"

#include "RC_chassis.h"
#include "RC_gantry.h"
#include "RC_filter.h"
#include "RC_event3.h"
#include "RC_gripper.h"
#include "RC_IR_communication.h"


#ifdef __cplusplus
namespace gantry
{
	class Aim_Ctrl
	{
	public:
		static constexpr uint16_t DEFAULT_FRAME = 60;
		static constexpr float   DEFAULT_ERROR = 0.001f;

		static constexpr float    COARSE_STABLE_THRESHOLD = 0.002f;
		static constexpr uint16_t COARSE_FRAME_COUNT      = 60;

		Aim_Ctrl(ros::Camera& camera_,
		         gantry::Gantry& gantry_,
				 chassis::Chassis& chassis_,
				 gantry::Gripper& gripper_);
		
	  IR::IRCmd ir;
	
	
		enum Axis : uint8_t
		{
			Axis_X,
			Axis_Y,
			Axis_Z
		};

		bool check_error()
		{
			// 四轴全零 = 相机未识别到目标，数据不予采纳
			return (fabsf(camera.X()) < 1e-6f && fabsf(camera.Y()) < 1e-6f
			&& fabsf(camera.Z()) < 1e-6f && fabsf(camera.Yaw()) < 1e-6f);
		}


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
		void Auto_Aim();
		bool Is_Done() const { return phase == Phase_Done; }
		float Get_Y() const { return y_result; }
		
		float bais = -0.016;
		
	private:
		float Get_Data(Axis axis);
		void Tracker_Clear();
				float error = 0;
		float final_error_z = 0;
		float final_error_y = 0;
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
		chassis::Chassis& chassis;
		gantry::Gripper&  gripper;
		gantry::GantryUser user;

		path::Event3 aim_event;

		filter::SecondOrderLPF z_lpf;
		filter::SecondOrderLPF y_lpf;

		static constexpr float PRE_POS_X = 0.05f;
		static constexpr float PRE_POS_Y = 0.0f;
		static constexpr float PRE_POS_Z = 0.0f;
		static constexpr float PRE_POS_THRESHOLD = 0.005f;

		bool aim_finish_flag = false;
		bool timer_flag = false;
		uint8_t timer_done = 0;

		uint32_t last_time = 0 ;

		enum Phase : uint8_t
		{
			Phase_Idle,
			Phase_PrePosition,
			Phase_Check,
			Phase_Yaw,
			Phase_YZ_Coarse,
			Phase_Z,
			Phase_Y,
			Phase_Y2,
			Phase_Wait,
			Phase_Done
		};
		Phase  phase    = Phase_Idle;
		float  y_result = 0;
	};
}
#endif
