#include "RC_graph_plan.h"
#include "RC_best_path.h"

namespace path
{
	constexpr float NAV_FLAT_MOVE_BLEND_DIS = 0.4;
	
	
	GraphPlan::GraphPlan(PathPlan3& plan_) : plan(plan_)
	{
		
	}
	
	bool GraphPlan::Add_Point_Wait(vector2d::Vector2D p, float blend_dis, LonConstr3* l, HeadConstr3* h, Event3_t e, bool end)
	{
		for (;;)
		{
			AddPointReturn state = plan.Add_Point(p, blend_dis, l, h, e, end);
			
			if (state == ADD_FULL)
			{
				osDelay(5); /*等待*/
				continue;
			}
			else if (state == ADD_SUCCESS)
			{
				last_nav.p = p;
				
				if (h)
				{
					last_nav.yaw = h->yaw;
				}
				
				return true;
			}
			else
			{
				return false;
			}
		}
	}
	
	constexpr uint8_t GRAPH_PLAN_MAX_PATH_LEN = 15;
	
	bool GraphPlan::Plan(NavPoint start, Destination dst_)
	{
		/*终点*/
		dst = dst_;
		
		/*起点*/
		last_nav.p = start.p;
		last_nav.yaw = start.yaw;
		
		uint8_t path[GRAPH_PLAN_MAX_PATH_LEN];
		uint8_t len = 0;
		
		if (!MapGraph::Get_Path(last_nav.p, dst.nav.p, path, len)) return false;
		if (len > GRAPH_PLAN_MAX_PATH_LEN) return false;
		if (len == 0) return false;
		
		for (uint8_t i = 0; i < len - 1; i++)
		{
			if (!Action_Plan(path[i], path[i + 1])) return false;
		}
		
		return Flat_Move(dst);
	}
	
	Action GraphPlan::Get_Action(uint8_t s, uint8_t e, int8_t *h) const
	{
		*h = MapGraph::height[e] - MapGraph::height[s];
		if (*h == -2 || *h == -4) return ACT_DOWN_STAIR;
		if (*h == 2 || *h == 4) return ACT_UP_STAIR;
		if (*h == -1) return ACT_DOWN_HILL;
		if (*h == 1) return ACT_UP_HILL;
		
		return ACT_NULL;
	}
	
	bool GraphPlan::Action_Plan(uint8_t s, uint8_t e)
	{
		int8_t h = 0;
		Action act = Get_Action(s, e, &h);
		
		switch(act)
		{
			case ACT_UP_STAIR:
			{
				return Up_Stair(s, e, h);
			}
			
			case ACT_DOWN_STAIR:
			{
				return Down_Stair(s, e, h);
			}
			
			case ACT_UP_HILL:
			{
				return Up_Down_Hill(h);
			}
			
			case ACT_DOWN_HILL:
			{
				return Up_Down_Hill(h);
			}
			
			default:
			{
				return false;
			}
		}
	}
	
	Event3_t GraphPlan::Head_Check_Id(Direction dir)
	{
		switch (dir)
		{
			case DIR_F:
			{
				return EVENT_HEAD_CHECK_F;
			}
			
			case DIR_B:
			{
				return EVENT_HEAD_CHECK_B;
			}
			
			case DIR_L:
			{
				return EVENT_HEAD_CHECK_L;
			}
			
			case DIR_R:
			{
				return EVENT_HEAD_CHECK_R;
			}
			
			default:
			{
				return EVENT3_NULL;
			}
		}
	}
	
	bool GraphPlan::Flat_Move(Destination dst)
	{
		HeadConstr3 h = plan.plan.head_m;
		h.yaw = dst.nav.yaw;
		return Add_Point_Wait(dst.nav.p, NAV_FLAT_MOVE_BLEND_DIS, NULL, &h, dst.event, (bool)dst.type);
	}
	
	
	Event3_t GraphPlan::Up_Down_Ready_Id_Dir(Direction move_dir, int8_t h, Direction& head_dir, Direction& L_or_R) const
	{
		float yaw_L = MapGraph::Yaw_On_Dir(move_dir - 1); /*从左边上下的航向*/
		float yaw_R = MapGraph::Yaw_On_Dir(move_dir + 1); /*从右边上下的航向*/
		
		float delta_L; /*从左边上下的旋转角度*/
		float delta_R; /*从右边上下的旋转角度*/
		
		delta_L = yaw_L - last_nav.yaw;
		if (delta_L < -PI) delta_L += TWO_PI;
		else if (delta_L > PI) delta_L -= TWO_PI;
		
		delta_R = yaw_R - last_nav.yaw;
		if (delta_R < -PI) delta_R += TWO_PI;
		else if (delta_R > PI) delta_R -= TWO_PI;
		
		delta_L = fabsf(delta_L);
		delta_R = fabsf(delta_R);
		
		if (fabsf(delta_L - delta_R) < (10.f / 180.f * PI)) /*10度内算相同*/
		{
			delta_L = yaw_L - dst.nav.yaw;
			if (delta_L < -PI) delta_L += TWO_PI;
			else if (delta_L > PI) delta_L -= TWO_PI;
			
			delta_R = yaw_R - dst.nav.yaw;
			if (delta_R < -PI) delta_R += TWO_PI;
			else if (delta_R > PI) delta_R -= TWO_PI;
			
			delta_L = fabsf(delta_L);
			delta_R = fabsf(delta_R);
		}

		if (delta_L < delta_R)
		{
			head_dir = move_dir - 1;
			L_or_R = DIR_L;
			switch (h)
			{
				case 2:
					return EVENT_UP_2_READY_L;
				case 4:
					return EVENT_UP_4_READY_L;
				case -2:
					return EVENT_DOWN_2_READY_L;
				case -4:
					return EVENT_DOWN_4_READY_L;
				default:
					return EVENT3_NULL;
			}
		}
		else
		{
			head_dir = move_dir + 1;
			L_or_R = DIR_R;
			switch (h)
			{
				case 2:
					return EVENT_UP_2_READY_R;
				case 4:
					return EVENT_UP_4_READY_R;
				case -2:
					return EVENT_DOWN_2_READY_R;
				case -4:
					return EVENT_DOWN_4_READY_R;
				default:
					return EVENT3_NULL;
			}
		}
	}
	
	
	/*========上台阶=========*/
	constexpr float UP_STAIR_HEAD_CHECK_OFFSET = -MapGraph::MF_SIZE / 2.f - MapGraph::CHASSIS_SIZE / 2.f - 0.18f;
	constexpr float UP_STAIR_HEAD_CHECK_BLEND_DIS = 0.3f;
	
	constexpr float UP_STAIR_SLOW_OFFSET = -MapGraph::MF_SIZE / 2.f - MapGraph::CHASSIS_SIZE / 2.f - 0.07f;
	//constexpr float UP_STAIR_SLOW_VEL = 0.4f;
	constexpr float UP_STAIR_SLOW_ACC = 2.f;
	
	constexpr float UP_STAIR_FINISH_OFFSET = -MapGraph::MF_SIZE / 2.f + MapGraph::CHASSIS_SIZE / 2.f;
	
	constexpr float UP_STAIR_HEAD_OFFSET = -MapGraph::MF_SIZE / 2.f + MapGraph::CHASSIS_SIZE / 2.f + 0.12f;
	
	bool GraphPlan::Up_Stair(uint8_t s, uint8_t e, int8_t h)
	{
		const vector2d::Vector2D e_center = MapGraph::Get_MF_Center(e); /*台阶中心坐标*/
		const Direction move_dir = MapGraph::Dir_From_To(s, e); /*上台阶方向*/
		vector2d::Vector2D p;
		
		Direction dir; /*航向方向*/
		Direction L_or_R; /* 左边或右边上 */
		Event3_t ready_event = Up_Down_Ready_Id_Dir(move_dir, h, dir, L_or_R); /*准备事件*/
		
		/* 距离上台阶后台阶中心距离 */
		float dis = (last_nav.p - e_center).length();
		if (dis > 1.85f) /* 是否太远 */ 
		{
			p = MapGraph::Offset_On_Dir(e_center, move_dir, UP_STAIR_HEAD_CHECK_OFFSET - 0.6f); /*航向检查，触发点坐标*/
		}
		else if (dis < 0.67f) /* 是否太近 */ 
		{
			p = MapGraph::Offset_On_Dir(e_center, move_dir, UP_STAIR_HEAD_CHECK_OFFSET + 0.08f); /*航向检查，触发点坐标*/
		}
		else
		{
			p = MapGraph::Offset_On_Dir(e_center, move_dir, UP_STAIR_HEAD_CHECK_OFFSET); /*航向检查，触发点坐标*/
		}
		
		Event3_t wait_R1_event = EVENT3_NULL;
		if (ros::BestPath::Is_Wait_R1_Pos(e))
		{
			if (L_or_R == DIR_L)
			{
				wait_R1_event = EVENT_WAIT_R1_L;
			}
			else
			{
				wait_R1_event = EVENT_WAIT_R1_R;
			}
		}
		
		/*-------*/
		if (!Add_Point_Wait(
			p, 
			UP_STAIR_HEAD_CHECK_BLEND_DIS, 
			NULL, 
			NULL, 
			ready_event | Head_Check_Id(dir) | wait_R1_event, 
			false
		)) return false; /*航向检查，触发点*/
		
		
		/*约束*/
		LonConstr3 lon = plan.plan.lon_m;
		HeadConstr3 head = plan.plan.head_m;	
			
			
		/*-------*/
		//head.yaw = MapGraph::Yaw_On_Dir(dir);
		p = MapGraph::Offset_On_Dir(e_center, move_dir, UP_STAIR_SLOW_OFFSET); /*减速点坐标*/
		if (!Add_Point_Wait(
			p, 
			0, 
			&lon, 
			NULL, 
			EVENT3_NULL, 
			false
		)) return false; /*减速点*/
		
		/*-------*/
		//lon.v = UP_STAIR_SLOW_VEL;
		lon.a = UP_STAIR_SLOW_ACC;
		//head.w = 0; /*禁止转向*/
		p = MapGraph::Offset_On_Dir(e_center, move_dir, UP_STAIR_FINISH_OFFSET); /*完成点坐标*/
		if (!Add_Point_Wait(
			p, 
			0, 
			&lon, 
			NULL, 
			EVENT3_NULL, 
			false
		)) return false; /*完成点*/

		/*-------*/
		lon = plan.plan.lon_m;
		head = plan.plan.head_m;
		head.yaw = MapGraph::Yaw_On_Dir(dir);
		p = MapGraph::Offset_On_Dir(e_center, move_dir, UP_STAIR_HEAD_OFFSET); /*向前防止碰撞*/
		if (!Add_Point_Wait(
			p, 
			0, 
			&lon, 
			&head, 
			EVENT3_NULL, 
			false
		)) return false;
		
		return true;
	}
	
	/*========下台阶=========*/
	constexpr float DOWN_STAIR_HEAD_CHECK_OFFSET = MapGraph::MF_SIZE / 2.f - MapGraph::CHASSIS_SIZE / 2.f - 0.18f;
	constexpr float DOWN_STAIR_HEAD_CHECK_BLEND_DIS = 0.3f;
	
	constexpr float DOWN_STAIR_SLOW_OFFSET = MapGraph::MF_SIZE / 2.f - MapGraph::CHASSIS_SIZE / 2.f - 0.07f;
	//constexpr float DOWN_STAIR_SLOW_VEL = 0.36f;
	constexpr float DOWN_STAIR_SLOW_ACC = 2.f;
	
	constexpr float DOWN_STAIR_FINISH_OFFSET = MapGraph::MF_SIZE / 2.f + MapGraph::CHASSIS_SIZE / 2.f;
	
	constexpr float DOWN_STAIR_HEAD_OFFSET = MapGraph::MF_SIZE / 2.f + MapGraph::CHASSIS_SIZE / 2.f + 0.12f;
	
	bool GraphPlan::Down_Stair(uint8_t s, uint8_t e, int8_t h)
	{
		const vector2d::Vector2D s_center = MapGraph::Get_MF_Center(s); /*台阶中心坐标*/
		const Direction move_dir = MapGraph::Dir_From_To(s, e); /*下台阶方向*/
		vector2d::Vector2D p;
		
		Direction dir;
		Direction L_or_R; /* 左边或右边下 */
		Event3_t ready_event = Up_Down_Ready_Id_Dir(move_dir, h, dir, L_or_R);
		
		
		
		/* 距离下台阶后台阶中心距离 */
		float dis = (last_nav.p - MapGraph::Offset_On_Dir(s_center, move_dir, MapGraph::MF_SIZE)).length();
		if (dis < 0.67f) /* 是否太近 */ 
		{
			p = MapGraph::Offset_On_Dir(s_center, move_dir, DOWN_STAIR_HEAD_CHECK_OFFSET + 0.08f); /*航向检查，触发点坐标*/
		}
		else
		{
			p = MapGraph::Offset_On_Dir(s_center, move_dir, DOWN_STAIR_HEAD_CHECK_OFFSET); /*航向检查，触发点坐标*/
		}
		
		Event3_t wait_R1_event = EVENT3_NULL;
		if (ros::BestPath::Is_Wait_R1_Pos(e))
		{
			if (L_or_R == DIR_L)
			{
				wait_R1_event = EVENT_WAIT_R1_L;
			}
			else
			{
				wait_R1_event = EVENT_WAIT_R1_R;
			}
		}
		
		
		/*-------*/
		if (!Add_Point_Wait(
			p, 
			DOWN_STAIR_HEAD_CHECK_BLEND_DIS, 
			NULL, 
			NULL, 
			ready_event | Head_Check_Id(dir) | wait_R1_event, 
			false
		)) return false; /*航向检查点*/
		
		/*约束*/
		LonConstr3 lon = plan.plan.lon_m;
		HeadConstr3 head = plan.plan.head_m;
			
			
		/*-------*/
		//head.yaw = MapGraph::Yaw_On_Dir(dir);
		p = MapGraph::Offset_On_Dir(s_center, move_dir, DOWN_STAIR_SLOW_OFFSET); /*减速点坐标*/
		if (!Add_Point_Wait(
			p, 
			0, 
			&lon, 
			NULL, 
			EVENT3_NULL, 
			false
		)) return false; /*减速点*/
		
		/*-------*/
		//lon.v = DOWN_STAIR_SLOW_VEL;
		lon.a = DOWN_STAIR_SLOW_ACC;
		//head.w = 0; /*禁止转向*/
		p = MapGraph::Offset_On_Dir(s_center, move_dir, DOWN_STAIR_FINISH_OFFSET); /*完成点坐标*/
		if (!Add_Point_Wait(
			p, 
			0, 
			&lon, 
			NULL, 
			EVENT3_NULL, 
			false
		)) return false; /*完成点*/
		
		/*-------*/
		lon = plan.plan.lon_m;
		head = plan.plan.head_m;
		head.yaw = MapGraph::Yaw_On_Dir(dir);
		if (e == 13)
		{
			p = MapGraph::Offset_On_Dir(s_center, move_dir, DOWN_STAIR_HEAD_OFFSET + 0.08); /*向前防止碰撞*/
		}
		else
		{
			p = MapGraph::Offset_On_Dir(s_center, move_dir, DOWN_STAIR_HEAD_OFFSET); /*向前防止碰撞*/
		}
		if (!Add_Point_Wait(
			p, 
			0, 
			&lon, 
			&head, 
			EVENT3_NULL, 
			false
		)) return false;
		
		return true;
	}
	
	
	bool GraphPlan::Up_Down_Hill(int8_t h)
	{
		vector2d::Vector2D low;
		vector2d::Vector2D high;
		
		uint8_t dx = data::Is_Blue_Left_Side();
		
		if (dx)
		{
			low = MapGraph::Offset_On_Dir(MapGraph::EXIT[dx].Get_A(), DIR_F, 0.65f);
			low = MapGraph::Offset_On_Dir(low, DIR_R, 0.75f);
			
			high = MapGraph::Offset_On_Dir(MapGraph::ARENA[dx].Get_A() + MapGraph::ARENA[dx].Get_AB(), DIR_B, 0.6f);
			high = MapGraph::Offset_On_Dir(high, DIR_R, 0.75f);
		}
		else
		{
			low = MapGraph::Offset_On_Dir(MapGraph::EXIT[dx].Get_A() + MapGraph::EXIT[dx].Get_AC(), DIR_F, 0.65f);
			low = MapGraph::Offset_On_Dir(low, DIR_L, 0.75f);
			
			high = MapGraph::Offset_On_Dir(MapGraph::ARENA[dx].Get_A() + MapGraph::EXIT[dx].Get_AC() + MapGraph::ARENA[dx].Get_AB(), DIR_B, 0.6f);
			high = MapGraph::Offset_On_Dir(high, DIR_L, 0.75f);
		}
		
		LonConstr3 lon = plan.plan.lon_m;
		
		if (h == 1)
		{
			if (!Add_Point_Wait(low, 0.35f, &lon, NULL, EVENT3_NULL, false)) return false;
			
			lon.v = 1.8;
			
			HeadConstr3 head = plan.plan.head_m;
			head.yaw = -HALF_PI;
			if (!Add_Point_Wait(high, 0.35f, &lon, &head, EVENT3_NULL, false)) return false;
		}
		else
		{
			if (!Add_Point_Wait(high, 0.35f, &lon, NULL, EVENT3_NULL, false)) return false;
			
			lon.v = 1.8;
			
			HeadConstr3 head = plan.plan.head_m;
			head.yaw = -HALF_PI;
			if (!Add_Point_Wait(low, 0.35f, &lon, &head, EVENT3_NULL, false)) return false;
		}
		
		return true;
	}
}