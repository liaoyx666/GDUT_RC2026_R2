#pragma once
#include "RC_data_pool.h"
#include "RC_map_graph.h"
#include "RC_graph_plan.h"

#ifdef __cplusplus
namespace path
{
	
	
	
	constexpr uint8_t NAVIGATION_MAX_DESTINATION = 8;
	
	
	class Navigation
    {
    public:
        Navigation(data::RobotPose& pose_);
        ~Navigation() = default;
	
		uint8_t Dst_FreeSpace() const { return (head - tail - 1 + NAVIGATION_MAX_DESTINATION) % NAVIGATION_MAX_DESTINATION; }
		uint8_t Dst_Num() const { return (tail - head + NAVIGATION_MAX_DESTINATION) % NAVIGATION_MAX_DESTINATION; }
	
    private:
		bool Add_Dst(NavPoint nav_, DstType type_, Event3_t event_);
        
		void Delete_Dst();
	
        Destination dst[NAVIGATION_MAX_DESTINATION];
		uint8_t head;
		uint8_t tail;
	
		data::RobotPose& pose;
		
    };
}
#endif
