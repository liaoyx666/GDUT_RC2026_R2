#pragma once
#include "RC_event3.h"
//#include "RC_traj_track3.h"
#include "RC_chassis.h"

#ifdef __cplusplus
namespace check
{
	constexpr float HEAD_CHEAK_THRESHOLD = 4.0f * PI / 180.f; /*4度*/

	class HeadCheck
    {
    public:
		HeadCheck(chassis::Chassis& chassis_, data::RobotPose& pose_);
		~HeadCheck() = default;

		inline void Head_Check()
		{
			// EVENT_HEAD_CHECK_F
			// EVENT_HEAD_CHECK_B
			// EVENT_HEAD_CHECK_L
			// EVENT_HEAD_CHECK_R
			
			if (!check_flag)
			{
				for (uint8_t i = 0; i < 4; i++)
				{
					if (check_event[i].Is_Trig())
					{
						check_flag = true;
						switch (i)
						{
							case 0:
								yaw = 0.f; break;
							case 1:
								yaw = PI; break;
							case 2:
								yaw = HALF_PI; break;
							case 3:
								yaw = -HALF_PI; break;
							default:
								check_flag = false; break;
						}
						break;
					}
				}
			}
			
			
			if (check_flag)
			{
				float delta_yaw = pose.Yaw() - yaw;
				
				if (delta_yaw < -PI)
					delta_yaw += TWO_PI;
				else if (delta_yaw > PI)
					delta_yaw -= TWO_PI;
				
				if (fabsf(delta_yaw) <= HEAD_CHEAK_THRESHOLD) /*yaw对齐后才能出发*/
				{
					check_event[check_dx].Finish();
					check_flag = false;
					chassis.Unforce_Lin_Vel_Zero(2); /*解除*/
				}
				else
				{
					chassis.Force_Lin_Vel_Zero(2); /*强制停车*/
				}
			}
		}
	
    private:
		path::Event3 check_event[4];
	
		chassis::Chassis& chassis;
		data::RobotPose& pose;
		bool check_flag;
		uint8_t check_dx;
		float yaw;
    };

}
#endif
