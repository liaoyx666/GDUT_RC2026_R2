#pragma once
#include "RC_camera.h"
#include "RC_pid.h"
#include "RC_chassis.h"
#include "RC_vector2d.h"

#ifdef __cplusplus
namespace aim
{
	class Aim_Ctrl
	{
	public:
		// 默认判稳参数（可在头文件中调试修改）
		static constexpr uint16_t DEFAULT_FRAME = 20;
		static constexpr float   DEFAULT_ERROR = 0.002f;

		Aim_Ctrl(ros::Camera& camera_, chassis::Chassis& chassis_, pid::Pid& yaw_pid_, pid::Pid& x_pid_);

		// 判稳数据轴枚举
		enum Axis : uint8_t
		{
			Axis_X,
			Axis_Y,
			Axis_Z,
			Axis_Yaw
		};

		/* 帧判稳 — 内联于头文件以方便调试 ------------------------------------*/
		// axis:       判稳数据轴
		// frame_count: 判稳帧数
		// error:      允许偏离窗口均值的最大误差
		bool Frame_Stable(Axis axis, uint16_t frame_count = DEFAULT_FRAME, float error = DEFAULT_ERROR)
		{
			if (frame_count > 64) frame_count = 64;             // 钳位到缓冲区上限
			Stable_Tracker& tracker = axis_tracker[axis];
			float current = Get_Data(axis);

			tracker.sum += current;
			if (tracker.frame_count < frame_count)              // 先填满窗口
			{
				tracker.buffer[tracker.frame_count++] = current;
				return false;
			}

			tracker.sum -= tracker.buffer[tracker.index];        // 滑动窗口
			tracker.buffer[tracker.index] = current;
			tracker.index = (tracker.index + 1) % frame_count;

			float average = tracker.sum / (float)frame_count;   // 窗口均值
			tracker.stable = (fabsf(current - average) <= error);// 稳定标志
			return tracker.stable;
		}
		/*------------------------------------------------------------------------*/

		void Reset();               // 复位状态机
		void Run();                 // 状态机主循环，周期性调用
		float Get_Y() const { return y_result; }  // 获取判稳后的y值

	private:
		float Get_Data(Axis axis);
		void Tracker_Clear();       // 清空四轴判稳追踪器

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
		pid::Pid&        yaw_pid;
		pid::Pid&        x_pid;

		enum Phase : uint8_t
		{
			Phase_Check,        // 异常判断 + 5帧确认
			Phase_Yaw,          // yaw PID对准
			Phase_X,            // x   PID对准
			Phase_Z,            // z   对准（占位）
			Phase_Y,            // y   判稳 + 存储
			Phase_Done          // 完成
		};
		Phase  phase    = Phase_Check;
		float  y_result = 0;
	};
}
#endif
