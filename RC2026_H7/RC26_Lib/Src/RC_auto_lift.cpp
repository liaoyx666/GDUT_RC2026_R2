#include "RC_auto_lift.h"

constexpr float HEAD_CHEAK_THRESHOLD = 5.f * PI / 180.f; /*4度*/

namespace chassis
{
	AutoLift::AutoLift(LiftChassis& lift_, path::TrajTrack3& track_, data::RobotPose& pose_)
	 : check_event{
		path::Event3(1, false, 0.07f),		// EVENT_HEAD_CHECK_F
		path::Event3(2, false, 0.07f),      // EVENT_HEAD_CHECK_B
		path::Event3(3, false, 0.07f),      // EVENT_HEAD_CHECK_L
		path::Event3(4, false, 0.07f),      // EVENT_HEAD_CHECK_R
	}, lift_event{
		path::Event3(5 , false, 0.1f),		// EVENT_UP_2_READY_L
		path::Event3(6 , false, 0.3f),     // EVENT_UP_4_READY_L
		path::Event3(7 , false, 0.1f),     // EVENT_UP_2_READY_R
		path::Event3(8 , false, 0.3f),     // EVENT_UP_4_READY_R
		path::Event3(9 , false, 0.1f),     // EVENT_DOWN_2_READY_L
		path::Event3(10, false, 0.1f),     // EVENT_DOWN_4_READY_L
		path::Event3(11, false, 0.1f),     // EVENT_DOWN_2_READY_R
		path::Event3(12, false, 0.1f)      // EVENT_DOWN_4_READY_R
	}, task::ManagedTask("AutoLiftTask", 21, 128, task::TASK_DELAY, 1), track(track_), pose(pose_), lift(lift_)
	{
		check_flag = false;
		lift_trig = false;
		yaw = 0;
		check_dx = 0;
	}
	
	void AutoLift::Task_Process()
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
}
