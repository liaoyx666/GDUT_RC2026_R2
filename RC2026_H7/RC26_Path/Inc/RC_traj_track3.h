#pragma once
#include "RC_nonlinear_pid.h"
#include "RC_vector2d.h"
#include "RC_data_pool.h"
#include "RC_pid.h"

#ifdef __cplusplus

namespace path
{
	constexpr float TRAJTRACK3_PRE_ALIGN_THRESHOLD = 3.f / PI * 180.f; /*3度*/
	constexpr float TRAJTRACK3_END_THRESHOLD = 0.03; /*路径结束阈值 m*/
	
	class Path3;
	
	class TrajTrack3
    {
    public:
		TrajTrack3(data::RobotPose& pose_);
		virtual ~TrajTrack3() {}
		
		bool Load_Path(Path3* path_);
		
		bool Calc_Vel(vector2d::Vector2D* v_, float* vw_) const;
		
		bool Is_End() const {return is_end;}
		
		bool Is_Load() const {return (bool)path;}
    protected:
		
    private:
		void Reset();
	
		Path3* path;
		mutable pid::NonlinearPid tan_pid;
		mutable pid::NonlinearPid nor_pid;
		mutable pid::NonlinearPid head_pid;
		
		mutable float last_wa;
		mutable float last_a;
		mutable float last_v;
		mutable float last_w;
		
		mutable float last_tan_v;
		mutable float last_head_v;
		
		mutable uint32_t last_time;
		
		float ld_kf; /*前视点前馈系数*/
		
		float ld_min; /*最小前视点距离 lookahead_dis*/
		float ld_k; /*前视点距离增益*/
		
		mutable bool is_start;
		mutable bool is_end;
		
		data::RobotPose* pose;
    };
}

#endif
