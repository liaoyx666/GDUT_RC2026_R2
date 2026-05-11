#pragma once
#include "RC_data_pool.h"
#include "RC_map_graph.h"
#include "RC_graph_plan.h"
#include "RC_task.h"

#ifdef __cplusplus
namespace path
{
	constexpr uint8_t NAVIGATION_MAX_DESTINATION = 8;
	
	class Navigation : public task::ManagedTask
    {
    public:
        Navigation(GraphPlan& plan_);
        ~Navigation() = default;
	
		bool Add_Dst(NavPoint nav_, DstType type_, Event3_t event_);
		uint8_t Dst_FreeSpace() const { return (head - tail - 1 + NAVIGATION_MAX_DESTINATION) % NAVIGATION_MAX_DESTINATION; }
		uint8_t Dst_Num() const { return (tail - head + NAVIGATION_MAX_DESTINATION) % NAVIGATION_MAX_DESTINATION; }
	
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
