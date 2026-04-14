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
	constexpr uint8_t PATHPLAN3_MAX_POINT_NUM = 50;
	
	class PathPlan3 : public task::ManagedTask
    {
    public:
		PathPlan3(LonConstr3 l, HeadConstr3 h, TrajTrack3& track_);
		virtual ~PathPlan3() {}
		
		bool Add_Point(vector2d::Vector2D p, float blend_dis, LonConstr3* l, HeadConstr3* h, Event3_t e, bool end);
		
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
		
		uint8_t Point_FreeSpace() const;
		uint8_t Point_Num() const;
		
    protected:
		void Task_Process() override;
		
    private:
		Point3 point[PATHPLAN3_MAX_POINT_NUM]; /*储存路径点，循环数组*/
		Path3 path[2]; /*路径，一个跟踪、一个规划*/
		TrajPlan3 plan; /*轨迹规划*/
		TrajTrack3& track; /*轨迹跟踪*/

		uint8_t dx; /*生成中的路径*/
	
		uint8_t head;
		uint8_t tail;

		bool is_enable;
		
		void Delete_Point();
		inline void Next_Path();
		inline void Plan_Path();
    };
}
#endif
