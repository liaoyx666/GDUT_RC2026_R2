#include "RC_auto_arm.h"

namespace arm
{
	AutoArm::AutoArm(Arm_task& arm_task_, path::PathPlan2& path_plan_, uint8_t event_arm_up_id_, uint8_t event_arm_down_id_)
	 : 	arm_task(&arm_task_), 
		event_arm_up(event_arm_up_id_, path_plan_),
		event_arm_down(event_arm_down_id_, path_plan_)
	{
		kfs_count = 0;
		
		arm_state = 0;
	}
	
	
	void AutoArm::Auto_Arm()
	{
		if(arm_state == 0 && kfs_count <= 3)
		{
			if (event_arm_up.Is_Start())
			{
				// 夹上方kfs
				arm_state = 1;
			}
			else if (event_arm_down.Is_Start())
			{
				// 夹下方kfs
				arm_state = 2;
			}
		}
		
		
		
		if (arm_state == 1)
		{
			// 夹上方kfs
			if (arm_task->Arm_Control(ARM_TASK::PICK_FRONT_UP_CUBE))
			{
				// 完成夹
				kfs_count++;
				
				arm_state = 3;
			}
		}
		else if (arm_state == 2)
		{
			// 夹下方kfs
			if (arm_task->Arm_Control(ARM_TASK::PICK_FRONT_DOWN_CUBE))
			{
				// 完成夹
				kfs_count++;
				
				arm_state = 3;
			}
		}
		else if (arm_state == 3)
		{
			if (kfs_count == 1)
			{
				// 放左边
				if (arm_task->Arm_Control(ARM_TASK::PLACE_LEFT_CUBE))
				{
					// 结束
					event_arm_down.Continue();
					event_arm_up.Continue();
					arm_state = 0;
				}
			}
			else if (kfs_count == 2)
			{
				// 放左边
				if (arm_task->Arm_Control(ARM_TASK::PLACE_RIGHT_CUBE))
				{
					// 结束
					event_arm_down.Continue();
					event_arm_up.Continue();
					arm_state = 0;
				}
			}
			else
			{
				// 放机械臂
				if (arm_task->Arm_Control(ARM_TASK::HOME))
				{
					// 结束
					event_arm_down.Continue();
					event_arm_up.Continue();
				}
			}
		}

		
		
	}
}