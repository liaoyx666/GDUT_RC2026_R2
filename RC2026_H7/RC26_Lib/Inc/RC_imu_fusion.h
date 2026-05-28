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
		filter::SecondOrderLPF filter;
	
    float yaw_fused;  // 融合输出角度
    float integral;   // 弱积分（仅消漂移）
    };
}
#endif
