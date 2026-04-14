#include "RC_head_check_event.h"

int llll = 0;


namespace path
{
	HeadCheck::HeadCheck(uint8_t id_, float yaw_, TrajTrack3& t_, data::RobotPose& pose_) : Event3(id_, false), track(t_), pose(pose_), yaw(yaw_)
	{
		flag = false;
	}
	
	void HeadCheck::Cheak_Head()
	{
		if (Is_Trig())
		{
			flag = true;
		}
		
		if (flag)
		{
			float delta_yaw = pose.Yaw() - yaw;
						
			if (delta_yaw < -PI)
			{
				delta_yaw += TWO_PI;
			}
			else if (delta_yaw > PI)
			{
				delta_yaw -= TWO_PI;
			}
			
			if (fabsf(delta_yaw) <= HEAD_CHEAK_THRESHOLD) /*yaw对齐后才能出发*/
			{
				Finish();
				flag = false;
				track.Unforce_Tan_Vel_Zero(); /*解除*/
			}
			else
			{
				llll = 1;
				track.Force_Tan_Vel_Zero(); /*强制停车*/
			}
		}
	}
}