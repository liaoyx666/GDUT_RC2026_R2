#pragma once
#include "RC_curve.h"
#include "RC_event3.h"

#ifdef __cplusplus
namespace path
{
	constexpr uint8_t PATH3_MAX_LINE_NUM   = 20; /*最大直线数*/
	constexpr uint8_t PATH3_MAX_ARC_NUM	   = 20; /*最大圆弧数*/
	constexpr float PATH3_MAX_LIN_VEL      = 4.f; /*最大线速度*/

	constexpr float PATH3_CURVE_SWITCH_THRESHOLD = 0.03f; /*切换曲线阈值 m*/
	
	constexpr uint8_t PATHLONCON3_MAX_NUM  = 15; /*最大纵向约束数*/
	constexpr uint8_t PATHHEADCON3_MAX_NUM = 15; /*最大航向约束数*/
	constexpr uint8_t PATHEVENT3_MAX_NUM   = 10; /*最大事件组数*/

	/*纵向约束条件*/
	struct LonConstr3
	{
		LonConstr3();
		LonConstr3(float v_, float a_) : v(v_), a(a_) {}
		static LonConstr3 min(const LonConstr3& c1, const LonConstr3& c2);
		float v;
		float a;
	};
	
	/*航向约束条件*/
	struct HeadConstr3
	{
		HeadConstr3();
		HeadConstr3(float y_, float w_, float wa_, bool t_) : yaw(y_), w(w_), wa(wa_), tan_head(t_) {}
		void min(const HeadConstr3& c1);
		float yaw;
		float w;
		float wa;
		bool tan_head; /*朝向路径切向*/
	};
	
	/*路径纵向分段约束*/
	struct PathLonCon3
	{
		PathLonCon3();
		float len; /*约束范围，从上一个位置到len处*/
		LonConstr3 c;
	};
	
	/*路径机器人航向分段约束*/
	struct PathHeadCon3
	{
		PathHeadCon3();
		float len; /*约束范围，从上一个位置到len处*/
		HeadConstr3 c;
	};
	
	/*路径触发事件组的位置*/
	struct PathEvent3
	{
		PathEvent3();
		float len; /*触发位置*/
		Event3_t event; /*事件组*/
	};
	
	/*路径*/
	class Path3
    {
    public:
		Path3();
		~Path3() = default;
		
		uint8_t Curve_Num() const {return line_num + arc_num;}
		float Len() const {return len[line_num + arc_num - 1];}
		void Reset();
		const bool& Is_Init() const {return is_init;}
		
		void Get_Point_On_T(float t, vector2d::Vector2D* p) const;
		void Get_Constr_On_Len(float l, LonConstr3* lon, HeadConstr3* head) const;
		bool Get_Point_On_Len(float len_, vector2d::Vector2D* p) const;
		
		void Get_Near_Point_T_Len_Dis_Tan_Nor_Vel_Cur_Lon_Head(
			vector2d::Vector2D p, 
			vector2d::Vector2D* near_p, 
			float* near_t, 
			float* near_l, 
			float* near_d, 
			vector2d::Vector2D* tan_, 
			vector2d::Vector2D* nor_,
			float* v,
			float* cur_,
			LonConstr3* lon,
			HeadConstr3* head
		) const;
		
		void Trig_Event_On_Len(float l_); /*触发事件*/
		
		bool Add_PathLonCon(float l, LonConstr3 c);
		bool Add_PathHeadCon(float l, HeadConstr3 c);
		bool Add_PathEvent(float l, Event3_t e);
		
		uint8_t Line_FreeSpace() const {return PATH3_MAX_LINE_NUM - line_num;}
		uint8_t Arc_FreeSpace() const {return PATH3_MAX_ARC_NUM - arc_num;}
		
		const bool& Pre_Align() const {return pre_align;}
    protected:
		
    private:
		bool Add_Line(vector2d::Vector2D start_, vector2d::Vector2D end_);
		bool Add_Arc(vector2d::Vector2D start_, vector2d::Vector2D end_, float radius_, bool is_counter_clockwise);
	
		void Add_Wait_Event_At_End(Event3_t e);
	
		bool Check_Near_Point(vector2d::Vector2D p_, uint8_t check_dx_, uint8_t* ndx_, vector2d::Vector2D* np, float* nt, float* nd) const;

		curve::Line2D line[PATH3_MAX_LINE_NUM];
		curve::Arc2D arc[PATH3_MAX_ARC_NUM];
		curve::Curve2D* curve[PATH3_MAX_LINE_NUM + PATH3_MAX_ARC_NUM];
		
		PathLonCon3 lon[PATHLONCON3_MAX_NUM]; /*路径速度、加速度分段约束*/
		PathHeadCon3 head[PATHHEADCON3_MAX_NUM]; /*路径机器人yaw朝向分段约束*/
		PathEvent3 event[PATHEVENT3_MAX_NUM]; /*路径触发事件组的位置*/
		float len[PATH3_MAX_LINE_NUM + PATH3_MAX_ARC_NUM]; /*保存所有曲线终点的路程*/
	
		uint8_t lon_num;
		uint8_t head_num;
		uint8_t event_num;
	
		mutable uint8_t lon_dx; /*保存上一次查找的索引，提升效率*/
		mutable uint8_t head_dx; /*保存上一次查找的索引，提升效率*/
		mutable uint8_t ndx; /*保存上一次查找的索引，提升效率*/
		mutable uint8_t len_dx; /*保存上一次查找的索引，提升效率*/
		mutable uint8_t event_dx; /*保存上一次查找的索引*/
		
		uint8_t line_num;
		uint8_t arc_num;
		
		bool is_init;
		Event3_t wait_event; /*终点处等待完成的事件*/
		bool pre_align; /*是否出发前对齐航向*/
	
		friend class TrajPlan3;
		friend class TrajTrack3;
    };
	
}

extern float calcVel(float dis, float end_v, float a);

#endif
