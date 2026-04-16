#pragma once
#include "RC_path3.h"
#include "RC_event3.h"

#ifdef __cplusplus

namespace path
{
	class LonConstr3;
    class HeadConstr3;
    class Path3;
    class Point3;
	
	constexpr uint8_t POINT3_END           = (1 << 0);
	constexpr uint8_t POINT3_HAVE_LONCON   = (1 << 1);
	constexpr uint8_t POINT3_HAVE_HEADCON  = (1 << 2);
	
	constexpr float TRAJPLAN3_MAX_RADIUS = 20.f; /*生成圆弧的最大半径*/
	constexpr float TRAJPLAN3_MIN_RADIUS = 0.05f; /*生成圆弧的最小半径*/
	
	/*路径点*/
	struct Point3
	{
		Point3();
		
		constexpr bool Is_End()       {return (bool)(param & POINT3_END);}
		constexpr bool Have_LonCon()  {return (bool)(param & POINT3_HAVE_LONCON);} /*是否有速度约束*/
		constexpr bool Have_HeadCon() {return (bool)(param & POINT3_HAVE_HEADCON);} /*是否有yaw约束*/
		constexpr bool Have_Event()   {return (bool)(event);} /*0x00为无事件，第几位上是1，即为存在id为几的事件*/
		
		constexpr void Set_Is_End()           {param = param | POINT3_END;}
		constexpr void Set_Have_LonCon()      {param = param | POINT3_HAVE_LONCON;} /*是否有速度约束*/
		constexpr void Set_Have_HeadCon()     {param = param | POINT3_HAVE_HEADCON;} /*是否有yaw约束*/
		constexpr void Set_Event(uint8_t id_) {event = event | (1 << (id_ - 1));}
		
		vector2d::Vector2D point; /*坐标*/
		float blend_dis; /*圆角距离，转角顶点到切点的距离*/
		uint8_t param; /*控制参数*/
		Event3_t event; /*事件组*/
		LonConstr3 lon; /*约束条件*/
		HeadConstr3 head;
	};
	
	/*路径生成状态*/
	enum TrajPlanState3 : uint8_t
	{
		TRAJPLAN_NULL,
		TRAJPLAN_HAVE_START,
		TRAJPLAN_HAVE_START_CORNER,
	};
	
	enum TrajPlanReturn3 : uint8_t
	{
		TRAJPLAN_FAIL,
		TRAJPLAN_END,
		TRAJPLAN_OK,
	};
	
	/*轨迹规划*/
	class TrajPlan3
    {
    public:
		TrajPlan3(LonConstr3 l, HeadConstr3 h);
		~TrajPlan3() = default;
		
		TrajPlanReturn3 Add_Point(Point3 p);
		bool Load_Path(Path3* path_);
		
    protected:
		
    private:
		void Reset();
		float Calc_CornerVel(float cor_ag);
		bool Add_Constr(Point3 p, float len);
		bool Add_MaxConstr(float len);
		void Calc_End_Vel();
		
		TrajPlanState3 state;

		vector2d::Vector2D dir;
	
		float blend_dis;
		
		vector2d::Vector2D tmp_s_p;
		Point3 tmp_c;
		
		Path3* path;
	
		LonConstr3 lon_m; /*约束最大值*/
		HeadConstr3 head_m; /*约束最大值*/
    };
	
	/*根据圆角距离，折角角度，计算相切圆半径*/
	float calcFilletRadius(float theta, float blend_dis);
	
}

float calcVel(float dis, float end_v, float a);

#endif
