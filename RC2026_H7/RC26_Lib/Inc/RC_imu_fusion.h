#pragma once
#ifdef __cplusplus
#include "RC_HWT101CT.h"
#include "RC_radar.h"
#include "RC_filter.h"

namespace fusion
{
	class ImuFusion
    {
    public:
		ImuFusion(ros::Radar& radar_, HWT101CT& imu_);
		~ImuFusion() = default;

		void Fusion();
    private:
		ros::Radar& radar;
		HWT101CT& imu;
	
		uint32_t last_time;
		filter::SecondOrderLPF filter;
	
		// 卡尔曼滤波状态变量 (类成员)
		float angle = 0;        // 融合角度 (rad)
		float bias = 0;         // 角速度偏置估计 (rad/s)
		float P[2][2];      // 协方差矩阵
		bool first_run = 1;     // 首次运行标志
	};
}
#endif
