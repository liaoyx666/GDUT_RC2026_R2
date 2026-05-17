#pragma once
#include "RC_event3.h"
#include "RC_task.h"
#include "RC_traj_track3.h"
#include "RC_data_pool.h"
#include "RC_lift_chassis.h"

#ifdef __cplusplus

constexpr float HEAD_CHEAK_THRESHOLD = 5.f * PI / 180.f; /*4度*/

namespace chassis
{
	class AutoLift// : public task::ManagedTask
    {
    public:
		AutoLift(LiftChassis& lift_, path::TrajTrack3& track_, data::RobotPose& pose_);
		~AutoLift() = default;

		inline void Auto_Lift()
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
					track.Unforce_Tan_Vel_Zero(); /*解除*/
				}
				else
				{
					track.Force_Tan_Vel_Zero(); /*强制停车*/
				}
			}
			
			/*-------------------------------------------------------*/
			// EVENT_UP_2_READY_L
			// EVENT_UP_4_READY_L
			// EVENT_UP_2_READY_R
			// EVENT_UP_4_READY_R
			// EVENT_DOWN_2_READY_L
			// EVENT_DOWN_4_READY_L
			// EVENT_DOWN_2_READY_R
			// EVENT_DOWN_4_READY_R
			
			if (lift.Is_End())
			{
				for (uint8_t i = 0; i < 8; i++)
				{
					if (lift_event[i].Is_Trig())
					{
						if (i < 4)
							la = LIFT_UP;
						else
							la = LIFT_DOWN;
						
						if ((i % 2) == 0)
							lh = LIFT_20;
						else
							lh = LIFT_40;
							
						 if ((i % 4) < 2)
							ld = LIFT_L;  // 左
						else
							ld = LIFT_R;  // 右
						
						lift_trig = true;
						break;
					}
				}
			}
			
			lift.Lift(la, lh, ld, lift_trig);
			
			if (lift_trig) lift_trig = false;
		}
    private:
		
		path::Event3 lift_event[8];
		path::Event3 check_event[4];
	
		path::TrajTrack3& track;
		data::RobotPose& pose;
		bool check_flag;
		uint8_t check_dx;
		float yaw;
	
		LiftChassis& lift;
		bool lift_trig;
		LiftAction la;
		LiftHeigth lh;
		LiftDir ld;
    };
}
#endif
