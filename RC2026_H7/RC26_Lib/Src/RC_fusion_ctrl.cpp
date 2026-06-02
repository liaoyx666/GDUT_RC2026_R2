#include "RC_fusion_ctrl.h"

namespace fusion
{
	FusionCtrl::FusionCtrl(QEO& chassis_qeo_, ImuFusion& imu_fusion_) : chassis_qeo(chassis_qeo_), imu_fusion(imu_fusion_)
	{
		
	}
}