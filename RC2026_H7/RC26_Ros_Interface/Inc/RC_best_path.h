#pragma once
#include "RC_cdc.h"
#include "RC_path2.h"
#include "RC_map.h"

#ifdef __cplusplus

#define MF_SIZE_HALF	0.6f// m
#define CHASSIS_HALF	0.32f// m 抓取kfs时底盘偏离格子中心的距离
#define MIN_DIS 0.1f// m 夹取时最小距离
#define CHASSIS_OFFSET 0.05f

#define CHASSIS_MOVE (MF_SIZE_HALF - CHASSIS_HALF - MIN_DIS)

namespace ros
{
	class BestPath : cdc::CDCHandler
    {
    public:
		BestPath(cdc::CDC &cdc_, uint8_t rx_id_);
		virtual ~BestPath() {}
		
		uint8_t step[9] = {2, 5, 4, 7, 10};
		uint8_t step_num = 5;
		
		vector2d::Vector2D Get_MF_Location(uint8_t n);
		
		void MF_Best_Path_Plan(Map& map, path::PathPlan2& path_plan);
		
		Dir Dir_From_To(uint8_t from, uint8_t to);
		
		float Dir_To_Yaw(Dir d);
		
    protected:
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override;

		//1,  2,  3, 
		//4,  5,  6, 
		//7,  8,  9, 
		//10, 11, 12
		const vector2d::Vector2D MF_location[12] = 
		{
			vector2d::Vector2D(3.473f, 2.749f), vector2d::Vector2D(3.465f, 1.561f), vector2d::Vector2D(3.471f, 0.340f),
			vector2d::Vector2D(4.698f, 2.756f), vector2d::Vector2D(4.685f, 1.565f), vector2d::Vector2D(4.695f, 0.372f),
			vector2d::Vector2D(5.879f, 2.763f), vector2d::Vector2D(5.871f, 1.589f), vector2d::Vector2D(5.871f, 0.345f),
			vector2d::Vector2D(7.079f, 2.771f), vector2d::Vector2D(7.077f, 1.570f), vector2d::Vector2D(7.091f, 0.371f )
		};
		
		
		// 0 : 20mm
		// 1 : 40mm
		// 2 : 60mm
		const int8_t MF_high[12] = 
		{
			1, 0, 1, 
			0, 1, 2, 
			1, 2, 1, 
			0, 1, 0
		};
	
	
    private:
		bool is_init = true;/// 
	
		float last_yaw = 0;
	
    };
}
#endif
