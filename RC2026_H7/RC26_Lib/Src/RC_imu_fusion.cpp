#include "RC_imu_fusion.h"

namespace fusion
{
	ImuFusion::ImuFusion(ros::Radar& radar_, HWT101CT& imu_) : radar(radar_), imu(imu_)
	{
		reset_flag = false;
		last_time = 0;
	}
	
	void ImuFusion::Fusion()
	{
		float imu_yaw   = imu.Yaw();
		float radar_yaw = radar.Yaw();
		float imu_delay_yaw = imu.Delay_Yaw();
		
		float imu_w = imu.W();

		float error = radar_yaw - imu_delay_yaw;
		
		if (error > PI)
			error -= TWO_PI;
		else if (error < -PI)
			error += TWO_PI;
		
		
		
		
		
		const float kp = 0.003f;
			
		imu_yaw += kp * error;
		
		if (imu_yaw > PI)
			imu_yaw -= TWO_PI;
		else if (imu_yaw < -PI)
			imu_yaw += TWO_PI;
		
		imu.Set_Yaw(imu_yaw);		
		
		
		
		if (!reset_flag && fabsf(error) > (float)(6.0 / 180.0 * PI) && fabsf(imu_w) < 0.05f)
		{
		
			reset_flag = true;
			last_time = timer::Timer::Get_TimeStamp();
			
		}
		
		if (reset_flag && timer::Timer::Get_DeltaTime(last_time) > 1000000) // 100ms
		{
			if (fabsf(error) > (float)(6.0 / 180.0 * PI) && fabsf(imu_w) < 0.05f)
			{
				imu.Set_Yaw(radar_yaw);
				
			}
			reset_flag = false;
		}
	}
}
