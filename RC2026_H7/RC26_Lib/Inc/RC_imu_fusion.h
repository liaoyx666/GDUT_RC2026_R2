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

		bool first_run = 1;     // 首次运行标志
	
		bool reset_flag;
	};
}
#endif
