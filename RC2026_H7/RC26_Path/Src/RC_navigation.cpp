#include "RC_navigation.h"

namespace path
{
	Navigation::Navigation(GraphPlan& plan_) : plan(plan_), task::ManagedTask("NaviTask", 19, 256, task::TASK_DELAY, 2)
	{
		head = 0;
		tail = 0;
		is_start = false;
		is_close = false;
	}
	
	bool Navigation::Add_Dst(const NavPoint& nav_, DstType type_, Event3_t event_)
	{
		if (Dst_FreeSpace() == 0) return false;
		if (is_close) return false; /*关闭*/
		
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
	
	constexpr float GET_KFS_OFFSET = MapGraph::MF_SIZE / 2.f + MapGraph::CHASSIS_SIZE / 2.f + 0.02f;
	
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
				event = GET_HIGH_40_KFS_READY_EVENT;
				break;
			
			case 2:
				event = GET_HIGH_20_KFS_READY_EVENT;
				break;
			
			case -2:
				event = GET_LOW_20_KFS_READY_EVENT;
				break;
			
			default:
				return false;
		}
		
		float yaw = MapGraph::Yaw_On_Dir(-get_dir);
		
		
		
		return Go_To_Do(chassis_pos, yaw, event | GET_PICK_KFS_EVENT | GraphPlan::Head_Check_Id(-get_dir));
	}
	
	constexpr float PUT_KFS_DIS = 0.99;
	
	bool Navigation::Go_To_Put_KFS_2L(uint8_t col)
	{
		if (col < 1 || col > 3) return false;
		
		vector2d::Vector2D put_p;
		float yaw;
		
		if (col == 1)
			put_p.x() = MapGraph::SUDOKU_COL_1_X;
		else if (col == 2)
			put_p.x() = MapGraph::SUDOKU_COL_2_X;
		else
			put_p.x() = MapGraph::SUDOKU_COL_3_X;
		
		
		if (data::Side::Is_Blue_Left_Side())
		{
			put_p.y() = -(MapGraph::FIELD_WIDTH - PUT_KFS_DIS);
			yaw = -HALF_PI;
		}
		else
		{
			put_p.y() = (MapGraph::FIELD_WIDTH - PUT_KFS_DIS);
			yaw = HALF_PI;
		}
			
		return Go_To_Do(put_p, yaw, EVENT_PUT_KFS_2L_READY | EVENT_PUT_KFS_PUT);
	}
	
	constexpr float GET_WEAPON_HEAD_X = 0.95;
	constexpr float GET_WEAPON_HEAD_DIS = 1.0;
	
	bool Navigation::Go_To_Get_Weapon_Head()
	{
		float yaw;
		vector2d::Vector2D p;
		
		p.x() = GET_WEAPON_HEAD_X;
		
		if (data::Side::Is_Blue_Left_Side())
		{
			yaw = -HALF_PI;
			p.y() = -(MapGraph::FIELD_WIDTH - GET_WEAPON_HEAD_DIS);
		}
		else
		{
			yaw = HALF_PI;
			p.y() = (MapGraph::FIELD_WIDTH - GET_WEAPON_HEAD_DIS);
		}
		
		return Go_To_Do(p, yaw, EVENT_GET_WEAPON_HEAD);
	}
	
	bool Navigation::Go_To_Dock()
	{
		float yaw;
		vector2d::Vector2D p;
		
		if (data::Side::Is_Blue_Left_Side())
		{
			yaw = HALF_PI;
			p = vector2d::Vector2D(1, -4);
		}
		else
		{
			yaw = -HALF_PI;
			p = vector2d::Vector2D(1, 4);
		}
		
		return Go_To_Do(p, yaw, EVENT_DOCK);
	}
	
	
	
	
	bool Navigation::Go_To_Combine_Ready()
	{
		float yaw;
		vector2d::Vector2D p;
		
		if (data::Side::Is_Blue_Left_Side())
		{
			yaw = -HALF_PI;
			p = vector2d::Vector2D(12 - 0.98 - 0.8, -4.9);
		}
		else
		{
			yaw = HALF_PI;
			p = vector2d::Vector2D(12 - 0.98 - 0.8, 4.9);
		}
		
		return Go_To_Do(p, yaw, EVENT_UP_4_READY_L);
	}

	
	
	
	
	bool Navigation::Go_To_Combine()
	{
		float yaw;
		vector2d::Vector2D p;
		
		if (data::Side::Is_Blue_Left_Side())
		{
			yaw = -HALF_PI;
			p = vector2d::Vector2D(12 - 0.98 + 0.41, -4.9);
		}
		else
		{
			yaw = -HALF_PI;
			p = vector2d::Vector2D(12 - 0.98 + 0.41, 4.9);
		}

		bool success = Go_To_Do(p, yaw, EVENT_COMBINE);
		
		Close(); // 禁止添加目的地
		
		return success;
	}
	
	
	
	bool Navigation::Uncombine(vector2d::Vector2D p, float yaw)
	{
		plan.plan.Force_End(); // 强制上一路径结束
		
		Update_Last_Navp(p, yaw); // 更新起点
		Open(); // 允许添加目的地
		vector2d::Vector2D dir = vector2d::Vector2D(yaw - HALF_PI) * 1.2f; // 移动方向
		
		Event3::Trig_Event(EVENT_COMBINE | EVENT_DOWN_4_READY_R);// 直接触发合体事件和下台阶事件，实现解体
		
		return Go_To_Do(p + dir, yaw, EVENT3_NULL);
	}
	
	
	bool Navigation::Go_To_Stick_Edge()
	{
		float yaw;
		vector2d::Vector2D p;
		
		Event3_t event;
		
		if (data::Side::Is_Blue_Left_Side())
		{
			event = EVENT_HEAD_CHECK_L;
			yaw = HALF_PI;
			p = vector2d::Vector2D(0.8, -5.2);
		}
		else
		{
			event = EVENT_HEAD_CHECK_R;
			yaw = -HALF_PI;
			p = vector2d::Vector2D(0.8, 5.2);
		}
		
		if (!Go_To_Do(p, yaw, event))
			return false;
		
		if (data::Side::Is_Blue_Left_Side())
		{
			p = vector2d::Vector2D(0.46, -5.2);
		}
		else
		{
			p = vector2d::Vector2D(0.46, 5.2);
		}
		
		
		return Go_To_Do(p, yaw, EVENT_STICK_L_EDGE);
	}

	
	// 避让r1在对抗区
	bool Navigation::Go_To_Avoid_R1_In_ARENA()
	{
		float yaw;
		vector2d::Vector2D p;
		
		if (data::Side::Is_Blue_Left_Side())
		{
			yaw = -HALF_PI;
			p = vector2d::Vector2D(10.3, -4);
		}
		else
		{
			yaw = HALF_PI;
			p = vector2d::Vector2D(10.3, 4);
		}
		
		return Go_To_Do(p, yaw, EVENT3_NULL);
	}
	
	
	void Navigation::Task_Process()
	{
		if (is_start)
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