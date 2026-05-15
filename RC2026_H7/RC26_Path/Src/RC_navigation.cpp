#include "RC_navigation.h"

namespace path
{
	Navigation::Navigation(GraphPlan& plan_) : plan(plan_), task::ManagedTask("NaviTask", 19, 256, task::TASK_DELAY, 2)
	{
		head = 0;
		tail = 0;
		is_start = false;
	}
	
	bool Navigation::Add_Dst(const NavPoint& nav_, DstType type_, Event3_t event_)
	{
		if (Dst_FreeSpace() == 0) return false;
		
		dst[tail].nav = nav_;
		dst[tail].type = type_;
		dst[tail].event = event_;
		
		tail = (tail + 1) % NAVIGATION_MAX_DESTINATION;
		return true;
	}
	
	void Navigation::Delete_Dst()
	{
		if (Dst_Num() != 0)
		{
			head = (head + 1) % NAVIGATION_MAX_DESTINATION;
		}
	}
	
	constexpr float GET_KFS_OFFSET = MapGraph::MF_SIZE / 2.f + MapGraph::CHASSIS_SIZE / 2.f + 0.05f;
	
	/* 
		去夹取KFS 
		kfs_node : KFS所属结点(几号MF)
		get_dir : 从哪个方向夹取
	*/
	bool Navigation::Go_To_Get_KFS(uint8_t kfs_node, Direction get_dir)
	{
		if (kfs_node < 1 || kfs_node > 12) return false;
		
		vector2d::Vector2D chassis_pos = MapGraph::Get_MF_Center(kfs_node);
		
		chassis_pos = MapGraph::Offset_On_Dir(chassis_pos, get_dir, GET_KFS_OFFSET);
		
		uint8_t chassis_node = MapGraph::Get_Node_On_Pos(chassis_pos);
		
		if (chassis_node == GRAPH_INVALID) return false;

		Event3_t event = 0;
		
		switch (MapGraph::height[kfs_node] - MapGraph::height[chassis_node])
		{
			case 4:
				event = GET_HIGH_40_KFS_EVENT;
				break;
			
			case 2:
				event = GET_HIGH_20_KFS_EVENT;
				break;
			
			case -2:
				event = GET_LOW_20_KFS_EVENT;
				break;
			
			default:
				return false;
		}
		
		float yaw = MapGraph::Yaw_On_Dir(-get_dir);
		
		return Go_To_Do(chassis_pos, yaw, event);
	}
	
	
	
	void Navigation::Task_Process()
	{
		if (!is_start)
		{
			//	全局起点
			vector2d::Vector2D start_point(0.42, -4.53);
			plan.plan.Add_Start_Point(start_point);
			
			last_navp.p = start_point;
			last_navp.yaw = 0;
			
			is_start = true;
		}
		else
		{
			if (Dst_Num() != 0)
			{
				if (plan.Plan(last_navp, dst[head]))
				{
					last_navp = dst[head].nav;
				}
				
				Delete_Dst();
			}
		}
	}
}