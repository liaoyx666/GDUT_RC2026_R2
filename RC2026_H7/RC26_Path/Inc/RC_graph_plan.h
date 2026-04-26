#pragma once
#include "RC_map_graph.h"
#include "RC_vector2d.h"
#include "RC_event3.h"
#include "RC_data_pool.h"
#include "RC_path_plan3.h"

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
	
	constexpr Event3_t EVENT_HEAD_CHECK_F = EVENT3_ID_1;
	constexpr Event3_t EVENT_HEAD_CHECK_B = EVENT3_ID_2;
	constexpr Event3_t EVENT_HEAD_CHECK_L = EVENT3_ID_3;
	constexpr Event3_t EVENT_HEAD_CHECK_R = EVENT3_ID_4;
	
	constexpr Event3_t EVENT_UP_2_READY_L = EVENT3_ID_5;
	constexpr Event3_t EVENT_UP_4_READY_L = EVENT3_ID_6;
	constexpr Event3_t EVENT_UP_2_READY_R = EVENT3_ID_7;
	constexpr Event3_t EVENT_UP_4_READY_R = EVENT3_ID_8;
	constexpr Event3_t EVENT_DOWN_2_READY_L = EVENT3_ID_9;
	constexpr Event3_t EVENT_DOWN_4_READY_L = EVENT3_ID_10;
	constexpr Event3_t EVENT_DOWN_2_READY_R = EVENT3_ID_11;
	constexpr Event3_t EVENT_DOWN_4_READY_R = EVENT3_ID_12;
	
	
	class GraphPlan
    {
    public:
		GraphPlan(data::RobotPose& pose_, PathPlan3& plan_);
		~GraphPlan() = default;
        
		bool Plan(Destination dst);
		PathPlan3& plan;
    private:
		bool Add_Point_Wait(vector2d::Vector2D p, float blend_dis, LonConstr3* l, HeadConstr3* h, Event3_t e, bool end) const;
		
	
		Event3_t Up_Down_Ready_Id_Dir(Direction move_dir, uint8_t h, Direction& head_dir) const;
	
		Event3_t Head_Check_Id(Direction dir) const;
	
		Action Get_Action(uint8_t s, uint8_t e, int8_t *h) const;
		bool Action_Plan(uint8_t s, uint8_t e);
	
		bool Up_Stair(uint8_t s, uint8_t e, int8_t h);
		bool Down_Stair(uint8_t s, uint8_t e, int8_t h);
		bool Up_Down_Hill(int8_t h);
	
		bool Flat_Move(Destination dst);
		
		Destination dst;
		NavPoint last_nav;
		
		data::RobotPose& pose;
		
    };
}
#endif
