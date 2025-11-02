#include "RC_path_task.h"


vector2d::Vector2D location(0, 0);
float yaw = 0;
float sx, sy, sa;


void path_teat(void *argument)
{
	MF_path.MF_Best_Path_Plan(map, path_plan);

	for (;;)
	{
		if (remote_ctrl.swb == 0)
		{
			omni_chassis.Set_Chassis_World_Spd(0, 0, 0, -radar.Get_Yaw() / 180.f * PI);
		}
		else if (remote_ctrl.swb == 1)
		{
			omni_chassis.Set_Chassis_World_Spd(remote_ctrl.left_x / 300.f, remote_ctrl.left_y / 300.f, remote_ctrl.right_x / 300.f, -radar.Get_Yaw() / 180.f * PI);
		}
		else
		{
			if (radar.Is_Valid() == true)
			{
				path_plan.Get_Speed(
					vector2d::Vector2D(radar.Get_X(), radar.Get_Y()),
					-radar.Get_Yaw() / 180.f * PI,
					&sx,
					&sy,
					&sa
				);
			}
			else
			{
				sx = 0;
				sy = 0;
				sa = 0;
			}
			
			omni_chassis.Set_Chassis_World_Spd(sx, sy, sa, -radar.Get_Yaw() / 180.f * PI);
		}
		
		
		static uint8_t flag = 0;
		static uint32_t last_time = 0;
		
		if (path_plan.Is_End() == true && flag == 0)
		{
			last_time = timer_us.Get_TimeStamp();
			flag = 1;
		}
		else if (flag == 1 && timer_us.Get_DeltaTime(last_time) >= 3000000)// 延时3s
		{
			path_plan.Next_Path();
			flag = 0;
		}
		
		osDelay(1);
	}
}

task::TaskCreator path_task("test", 27, 512, path_teat, NULL);
