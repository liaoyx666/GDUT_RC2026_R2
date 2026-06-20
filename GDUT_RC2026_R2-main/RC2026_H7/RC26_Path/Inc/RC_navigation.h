#pragma once
#include "RC_data_pool.h"
#include "RC_map_graph.h"
#include "RC_graph_plan.h"
#include "RC_task.h"

#ifdef __cplusplus
namespace path
{
	constexpr uint8_t NAVIGATION_MAX_DESTINATION = 15;
	
	class Navigation : public task::ManagedTask
    {
    public:
        Navigation(GraphPlan& plan_);
        ~Navigation() = default;
	
		bool Add_Dst(const NavPoint& nav_, DstType type_, Event3_t event_);
		
		inline bool Go_To_Do(vector2d::Vector2D p, float yaw, Event3_t event)
		{
			NavPoint nav;
			nav.p = p;
			nav.yaw = yaw;
			return Add_Dst(nav, DST_END, event);
		}
		
		
		inline bool Pass_Do(vector2d::Vector2D p, float yaw, Event3_t event)
		{
			NavPoint nav;
			nav.p = p;
			nav.yaw = yaw;
			return Add_Dst(nav, DST_PASS, event);
		}
		
		
		void Update_Last_Navp(vector2d::Vector2D p, float yaw)
		{
			last_navp.p = p;
			last_navp.yaw = yaw;
		}
		
		
		void Add_Start(vector2d::Vector2D start_point, float yaw)
		{
			if (!is_start)
			{
				plan.plan.Add_Start_Point(start_point);
				
				Update_Last_Navp(start_point, yaw);
				
				is_start = true;
			}
		}
		
		/*------------------------------------------------------------*/
		bool Go_To_Get_KFS(uint8_t kfs_node, Direction get_dir);
		bool Go_To_Put_KFS_2L(uint8_t col); /*放二层， 1 ~ 3 列，靠近梅林大*/
		bool Go_To_Get_Weapon_Head();
		bool Go_To_Dock();
		bool Go_To_Combine();
		bool Go_To_Combine_Ready();
		bool Uncombine(vector2d::Vector2D p, float yaw);
		
		bool Go_To_Stick_Edge();
		/*------------------------------------------------------------*/
		
    private:
		uint8_t Dst_FreeSpace() const { return (head - tail - 1 + NAVIGATION_MAX_DESTINATION) % NAVIGATION_MAX_DESTINATION; }
		uint8_t Dst_Num() const { return (tail - head + NAVIGATION_MAX_DESTINATION) % NAVIGATION_MAX_DESTINATION; }
	
		void Close() { is_close = true; }
		void Open() { is_close = false; }
	
		void Task_Process() override;
		
		void Delete_Dst();
		
        Destination dst[NAVIGATION_MAX_DESTINATION];
		uint8_t head;
		uint8_t tail;
		
		GraphPlan& plan;
		NavPoint last_navp; /*储存上次终点作为下次起点*/
	
		bool is_start; /*是否添加全局起点*/
		bool is_close; /*禁止添加目标点*/
    };
}
#endif
