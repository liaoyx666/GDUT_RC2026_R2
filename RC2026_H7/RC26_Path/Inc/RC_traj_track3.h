#pragma once
#include "RC_nonlinear_pid.h"
#include "RC_vector2d.h"
#include "RC_data_pool.h"
#include "RC_pid.h"
#include "RC_task.h"
#include "RC_chassis.h"
#include "RC_head_ctrl.h"

#ifdef __cplusplus

namespace path
{
	constexpr float TRAJTRACK3_PRE_ALIGN_THRESHOLD = 3.f * PI / 180.f; /*3度*/
	constexpr float TRAJTRACK3_END_THRESHOLD = 0.04f; /*路径结束阈值 m*/
	
	
	class Path3;
	
	class TrajTrack3 : public task::ManagedTask
    {
    public:
		TrajTrack3(data::RobotPose& pose_, chassis::Chassis& chassis_, HeadCtrl& head_ctrl_, float deadzone_);
		~TrajTrack3() = default;
		
		bool Load_Path(Path3* path_);
		
		void Enable()
		{
			is_enable = true;
			head_ctrl.Enable();
		}
		
		void Disable()
		{
			is_enable = false;
			head_ctrl.Disable();
		}
		
		const bool& Is_End() const {return is_end;}
		
		bool Is_Load() const {return (bool)path;}
		
		void Force_Tan_Vel_Zero() {tan_vel_zero = true;}
		void Unforce_Tan_Vel_Zero() {tan_vel_zero = false;}
		
    protected:
		void Task_Process() override;
		
    private:
		void Reset();
		bool Calc_Vel(vector2d::Vector2D& v_) const;
	
		Path3* path;
		mutable pid::NonlinearPid tan_pid;
		mutable pid::NonlinearPid nor_pid;
	
		mutable float last_wa;
		mutable float last_a;
		mutable float last_v;
		mutable float last_w;
		
		mutable float last_tan_v;
	
		mutable uint32_t last_time;
		
		float ld_kf; /*前视点前馈系数*/
		
		float ld_min; /*最小前视点距离 lookahead_dis*/
	
		mutable bool is_start;
		mutable bool is_end;

		bool is_enable;
		bool tan_vel_zero; /*强制切向速度为0*/
		
		data::RobotPose& pose;
		chassis::Chassis& chassis;
		HeadCtrl& head_ctrl;
    };
}

#endif
