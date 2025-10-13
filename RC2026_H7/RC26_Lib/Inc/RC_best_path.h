#pragma once
#include "RC_cdc.h"
#include "RC_path.h"
#include "RC_map.h"

#ifdef __cplusplus

#define MF_SIZE			1.2f// m
#define CHASSIS_MOVE	0.2f// m 抓取kfs时底盘偏离格子中心的距离

namespace ros
{

	
	class BestPath : cdc::CDCHandler
    {
    public:
		BestPath(cdc::CDC &cdc_, uint8_t rx_id_);
		virtual ~BestPath() {}
		
		uint8_t step[6] = {2, 5, 6, 9, 12};
		uint8_t step_num = 5;
		
		
		vector2d::Vector2D Get_MF_Location(uint8_t n);
		
		void MF_Best_Path_Plan(Map& map, path::PathPlan& path_plan);
		
		Dir Dir_From_To(uint8_t from, uint8_t to);
		
		float Dir_To_Yaw(Dir d);
		
		
		
		
    protected:
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;

		vector2d::Vector2D MF_location[12] = 
		{
			vector2d::Vector2D(-2.7f, 3.5f), vector2d::Vector2D(-1.5f, 3.5f) ,vector2d::Vector2D(-0.3f, 3.5f),
			vector2d::Vector2D(-2.7f, 4.7f), vector2d::Vector2D(-1.5f, 4.7f) ,vector2d::Vector2D(-0.3f, 4.7f),
			vector2d::Vector2D(-2.7f, 5.9f), vector2d::Vector2D(-1.5f, 5.9f) ,vector2d::Vector2D(-0.3f, 5.9f),
			vector2d::Vector2D(-2.7f, 7.1f), vector2d::Vector2D(-1.5f, 7.1f) ,vector2d::Vector2D(-0.3f, 7.1f)
		};
	
	
    private:
		bool is_init = true;/// 
	
    };
}
#endif
