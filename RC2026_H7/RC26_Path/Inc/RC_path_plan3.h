/*路径规划整合*/
#pragma once
#include "RC_task.h"
#include "RC_data_pool.h"
#include "RC_chassis.h"
#include "RC_vector2d.h"

#include "RC_path3.h"
#include "RC_traj_plan3.h"
#include "RC_traj_track3.h"

#ifdef __cplusplus
namespace path
{
	constexpr uint8_t PATHPLAN3_MAX_POINT_NUM = 60;
	
	
	enum AddPointReturn : uint8_t
	{
		ADD_SUCCESS = 0,
		ADD_FAIL,
		ADD_FULL,
	};
	
	
	
	class PathPlan3// : public task::ManagedTask
    {
    public:
		PathPlan3(LonConstr3 l, HeadConstr3 h, TrajTrack3& track_);
		~PathPlan3() = default;
		
		inline AddPointReturn Add_Point(vector2d::Vector2D p, float blend_dis, const LonConstr3* l, const HeadConstr3* h, Event3_t e, bool end)
		{
			if (!is_start) return ADD_FULL; // 等待开始点生成
			return Add_One_Point(p, blend_dis, l, h, e, end);
		}
		
		inline AddPointReturn Add_Start_Point(vector2d::Vector2D p)
		{
			if (is_start) return ADD_FAIL;
			AddPointReturn rt = Add_One_Point(p, 0, NULL, NULL, EVENT3_NULL, false);
			is_start = true;
			return rt;
		}
	
		void Enable()
		{
			is_enable = true;
			track.Enable();
		}
		
		void Disable()
		{
			is_enable = false;
			track.Disable();
		}
		
		/*任务函数*/
		inline void Plan()
		{
			/*生成路径*/
			Plan_Path();
			
			/*切换路径*/
			Next_Path();
		}
		
		uint8_t Point_FreeSpace() const;
		uint8_t Point_Num() const;
		Path3 path[2]; /*路径，一个跟踪、一个规划*/
    protected:
		
		
    private:
		AddPointReturn Add_One_Point(vector2d::Vector2D p, float blend_dis, const LonConstr3* l, const HeadConstr3* h, Event3_t e, bool end);
	
		Point3 point[PATHPLAN3_MAX_POINT_NUM]; /*储存路径点，循环数组*/
		
		TrajPlan3 plan; /*轨迹规划*/
		TrajTrack3& track; /*轨迹跟踪*/

		uint8_t dx; /*生成中的路径*/
	
		uint8_t head;
		uint8_t tail;

		bool is_enable;
		bool is_start;
		
		void Delete_Point();
	
		inline void Next_Path()
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
		
		inline void Plan_Path()
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
	
		friend class GraphPlan;
    };
}
#endif
