#include "RC_path_plan3.h"

namespace path
{
	PathPlan3::PathPlan3(LonConstr3 l, HeadConstr3 h, TrajTrack3& track_)
	: task::ManagedTask("PathPlan3Task", 26, 512, task::TASK_DELAY, 2), plan(l, h), track(track_)
	{
		head = 0;
		tail = 0;
		dx = 0;
		
		plan.Load_Path(&path[0]);
		is_enable = false;
	}
	
	/*任务函数*/
	void PathPlan3::Task_Process()
	{
		/*生成路径*/
		Plan_Path();
		
		/*切换路径*/
		Next_Path();
	}
	
	
	AddPointReturn PathPlan3::Add_Point(vector2d::Vector2D p, float blend_dis, const LonConstr3* l, const HeadConstr3* h, Event3_t e, bool end)
	{
		if (Point_FreeSpace() == 0) return ADD_FULL; /*无空间*/
		
		if (Point_Num() != 0 && point[(tail - 1) % PATHPLAN3_MAX_POINT_NUM].point == p) return ADD_FAIL; /*与上一个点坐标相同*/
		
		point[tail].point = p; /*坐标*/
		point[tail].blend_dis = blend_dis; /*圆角距离*/
		point[tail].event = e; /*事件组*/
		
		if (l != nullptr)
		{
			point[tail].Set_Have_LonCon();
			point[tail].lon = *l; /*纵向约束*/
		}
		
		if (h != nullptr)
		{
			point[tail].Set_Have_HeadCon();
			point[tail].head = *h; /*航向约束*/
		}
		
		if (end)
		{
			point[tail].Set_Is_End(); /*设为结束点*/
		}
		
		tail = (tail + 1) % PATHPLAN3_MAX_POINT_NUM;
		return ADD_SUCCESS;
	}
	
	void PathPlan3::Next_Path()
	{
		if ((track.Is_End() || !track.Is_Load()) && path[dx].Is_Init())
		{
			if (track.Load_Path(&path[dx]))
			{
				dx = (dx + 1) % 2;
				plan.Load_Path(&path[dx]);
			}
		}
	}
	
	void PathPlan3::Plan_Path()
	{
		while(Point_Num() != 0 && !path[dx].Is_Init())
		{
			TrajPlanReturn3 state = plan.Add_Point(point[head]);
			
			if (state == TRAJPLAN_OK)
			{
				/*点添加成功*/
				head = (head + 1) % PATHPLAN3_MAX_POINT_NUM;
			}
			else if (state == TRAJPLAN_END)
			{
				/*添加结束*/
				break;
			}
			else
			{
				/*点添加失败*/
				break;
			}
		}
	}
	
	
	uint8_t PathPlan3::Point_FreeSpace() const
	{
		return (head - tail - 1 + PATHPLAN3_MAX_POINT_NUM) % PATHPLAN3_MAX_POINT_NUM;
	}
	
	
	uint8_t PathPlan3::Point_Num() const
	{
		return (tail - head + PATHPLAN3_MAX_POINT_NUM) % PATHPLAN3_MAX_POINT_NUM;
	}
	
	
	void PathPlan3::Delete_Point()
	{
		if (Point_Num() != 0)
		{
			head = (head + 1) % PATHPLAN3_MAX_POINT_NUM;
		}
	}
}