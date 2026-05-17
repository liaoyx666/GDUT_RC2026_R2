#pragma once
#include "RC_data_pool.h"
#include "RC_map_graph.h"
#include "RC_graph_plan.h"
#include "RC_task.h"

#ifdef __cplusplus
namespace path
{
	constexpr uint8_t NAVIGATION_MAX_DESTINATION = 8;
	
	constexpr Event3_t GET_HIGH_20_KFS_READY_EVENT = EVENT3_ID_13;
	constexpr Event3_t GET_HIGH_40_KFS_READY_EVENT = EVENT3_ID_14;
	constexpr Event3_t GET_LOW_20_KFS_READY_EVENT = EVENT3_ID_15;
	constexpr Event3_t GET_PICK_KFS_EVENT = EVENT3_ID_16;
	
	
	class Navigation : public task::ManagedTask
    {
    public:
        Navigation(GraphPlan& plan_);
        ~Navigation() = default;
	
		bool Add_Dst(const NavPoint& nav_, DstType type_, Event3_t event_);
		uint8_t Dst_FreeSpace() const { return (head - tail - 1 + NAVIGATION_MAX_DESTINATION) % NAVIGATION_MAX_DESTINATION; }
		uint8_t Dst_Num() const { return (tail - head + NAVIGATION_MAX_DESTINATION) % NAVIGATION_MAX_DESTINATION; }
		
		inline bool Go_To_Do(vector2d::Vector2D p, float yaw, Event3_t event)
		{
			NavPoint nav;
			nav.p = p;
			nav.yaw = yaw;
			return Add_Dst(nav, DST_END, event);
		}
		
		
		bool Go_To_Get_KFS(uint8_t kfs_pos, Direction get_dir);
		
    private:
		void Task_Process() override;
		
		void Delete_Dst();
		
        Destination dst[NAVIGATION_MAX_DESTINATION];
		uint8_t head;
		uint8_t tail;
	
		//data::RobotPose& pose;
		GraphPlan& plan;
		
		NavPoint last_navp;
	
		bool is_start;
    };
}
#endif
