#pragma once
#ifdef __cplusplus
#include "RC_HWT101CT.h"
#include "RC_radar.h"
#include "RC_filter.h"

namespace fusion
{
	enum ImuFusionMode : uint8_t
	{
		IMU_FUSION_MODE = 0,
		IMU_RADAR_MODE,
	};
	
	
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

		bool is_init;  
		
		const float kp = 0.005f;
	
		float last_radar_yaw;
	
		bool reset_flag;
	
		ImuFusionMode mode;
		friend class FusionCtrl;
	};
}
#endif
