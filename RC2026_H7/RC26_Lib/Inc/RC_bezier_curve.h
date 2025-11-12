// 贝塞尔曲线库
#pragma once
#include "RC_vector2d.h"

#ifdef __cplusplus

#define FIND_NEAREST_DISTANCE_STEP_COUNT 10// 查找次数
#define BEZIER_SAMPLE_NUM	10// 不包含起点
#define GOLDEN_RATIO (sqrtf(5.f) - 1.f) / 2.f  // 黄金分割比例 (~0.618)

namespace curve
{
	typedef enum BezierOrder : uint8_t
	{
		FIRST_ORDER_BEZIER,
		SECOND_ORDER_BEZIER
	} BezierOrder;


	class BezierCurve
    {
    public:
		BezierCurve();
		BezierCurve(vector2d::Vector2D start_point_, vector2d::Vector2D end_point_);
		BezierCurve(vector2d::Vector2D start_point_, vector2d::Vector2D control_point_, vector2d::Vector2D end_point_);
		
		void Bezier_Update(vector2d::Vector2D start_point_, vector2d::Vector2D end_point_);
		void Bezier_Update(vector2d::Vector2D start_point_, vector2d::Vector2D control_point_, vector2d::Vector2D end_point_);
		
		virtual ~BezierCurve() {}
		
		
		vector2d::Vector2D Get_Control_Point() const {return control_point;}
		
		vector2d::Vector2D Get_Point(float t);
		
		float Get_Nearest_Distance(vector2d::Vector2D point, float* t);
		
		float Get_Current_Len(float t);
		
		vector2d::Vector2D Get_Tangent_Vector(float t);
		vector2d::Vector2D Get_Normal_Vector(const vector2d::Vector2D& point, const float t);
			
		vector2d::Vector2D Get_Start_Point() {return start_point;}
		vector2d::Vector2D Get_End_Point() {return end_point;}
		
		float Get_Curvature(float t);
		
		float Get_len() const {return len;}
		
		BezierOrder Get_Bezier_Order() {return order;}
		
		void Set_End_Vel(float end_vel_)
		{
			end_vel = fabsf(end_vel_);
		}
		
		
		float Get_Max_Vel(float t);
		
    protected:
		float distance_list[BEZIER_SAMPLE_NUM];
		const float bezier_sample_step = 1.f / (float)BEZIER_SAMPLE_NUM;// 采样步长
		
		float len = 0;// 总长度
		BezierOrder order;// 阶数
		
		vector2d::Vector2D start_point, end_point;
		vector2d::Vector2D control_point;
	
		vector2d::Vector2D tangent_vector;// 切向量
		vector2d::Vector2D normal_vector;// 法向量
		
    private:
		float end_vel = 0;
		float max_vel_list[BEZIER_SAMPLE_NUM + 1];
	
		float max_curvature_len;// 最大曲率处
		float max_curvature_max_vel;// 最大曲率处的最大速度
	
		float current_max_vel;
    };
}
#endif
