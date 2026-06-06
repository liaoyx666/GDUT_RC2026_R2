#pragma once
#include "RC_map_graph.h"
#include "RC_vector2d.h"
#include "RC_event3.h"
#include "RC_data_pool.h"
#include "RC_path_plan3.h"
//#include "RC_best_path.h"

#ifdef __cplusplus
namespace path
{
	enum DstType
	{
		DST_END = 1, /*结束*/
		DST_PASS = 0, /*经过*/
	};
	
	struct NavPoint
	{
		vector2d::Vector2D p;
		float yaw;
	};
	
	struct Destination
	{
		NavPoint nav;
		DstType type;
		Event3_t event;
	};
	
	enum Action : uint8_t
	{
		ACT_UP_STAIR = 0,
		ACT_DOWN_STAIR,
		ACT_UP_HILL,
		ACT_DOWN_HILL,
		ACT_NULL,
	};

	class GraphPlan
    {
    public:
		GraphPlan(PathPlan3& plan_);
		~GraphPlan() = default;
        
		bool Plan(NavPoint start, Destination dst);
		static Event3_t Head_Check_Id(Direction dir);
	
		bool Add_Point_Wait(vector2d::Vector2D p, float blend_dis, LonConstr3* l, HeadConstr3* h, Event3_t e, bool end);
	
		PathPlan3& plan;
    private:
		
		
		Event3_t Up_Down_Ready_Id_Dir(Direction move_dir, int8_t h, Direction& head_dir, Direction& L_or_R) const;
	
		
	
		Action Get_Action(uint8_t s, uint8_t e, int8_t *h) const;
		bool Action_Plan(uint8_t s, uint8_t e);
	
		bool Up_Stair(uint8_t s, uint8_t e, int8_t h);
		bool Down_Stair(uint8_t s, uint8_t e, int8_t h);
		bool Up_Down_Hill(int8_t h);
	
		bool Flat_Move(Destination dst);
		
		Destination dst;
		NavPoint last_nav;
		
		//data::RobotPose& pose;
    };
}
#endif
