#pragma once
#ifdef __cplusplus
#include "RC_imu_fusion.h"
#include "RC_QEO.h"

namespace fusion
{
	enum FusionCtrlMode : uint8_t
	{
		CTRL_RADAR_MODE = 0,
		CTRL_FUSION_MODE,
	};
	
	class FusionCtrl
    {
    public:
		FusionCtrl(QEO& chassis_qeo_, ImuFusion& imu_fusion_);
		~FusionCtrl() = default;
		
		void Radar_Mode()
		{
			chassis_qeo.mode = RADAR_MODE;
			imu_fusion.mode = IMU_RADAR_MODE;
		}
		
		void Fusion_Mode()
		{
			chassis_qeo.mode = FUSION_MODE;
			imu_fusion.mode = IMU_FUSION_MODE;
		}
		
		
    private:
		QEO& chassis_qeo;
		ImuFusion& imu_fusion;
    };
}
#endif
