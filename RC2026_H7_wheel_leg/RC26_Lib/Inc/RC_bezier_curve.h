// 贝塞尔曲线库
#pragma once
#include "RC_vector2d.h"

#ifdef __cplusplus

#define FIND_NEAREST_DISTANCE_STEP_COUNT 10// 查找最近点迭代次数
#define BEZIER_SAMPLE_NUM	10// 不包含起点
#define GOLDEN_RATIO (sqrtf(5.f) - 1.f) / 2.f  // 黄金分割比例 (~0.618)

namespace curve
{
	
	// 曲线阶数
	typedef enum BezierOrder : uint8_t
	{
		FIRST_ORDER_BEZIER,
		SECOND_ORDER_BEZIER
	} BezierOrder;


	class BezierCurve
    {
    public:
		/*--------------------------------------------------------------------------------------------*/
	
		// 构造函数
		BezierCurve();
		BezierCurve(vector2d::Vector2D start_point_, vector2d::Vector2D end_point_);
		BezierCurve(vector2d::Vector2D start_point_, vector2d::Vector2D control_point_, vector2d::Vector2D end_point_);
	
		virtual ~BezierCurve() {}
		
		/*--------------------------------------------------------------------------------------------*/
			
		// 更新曲线（一阶）
		void Bezier_Update(vector2d::Vector2D start_point_, vector2d::Vector2D end_point_);
		
		// 更新曲线（二阶）
		void Bezier_Update(vector2d::Vector2D start_point_, vector2d::Vector2D control_point_, vector2d::Vector2D end_point_);
		
		// 设置结束速度
		void Set_End_Vel(float end_vel_) {end_vel = fabsf(end_vel_);}
		
		void Set_Num(uint16_t end_point_num_);
		
		void Set_Num(uint16_t control_point_num_, uint16_t end_point_num_);
		
		/*-----------------------------------------------------*/
		
		void Set_Throughout_Max_Vel(float throughout_max_vel_) {throughout_max_vel = throughout_max_vel_;}
		
		void Set_Decel(float decel_)  {decel = decel_;}
		
		/*-----------------------------------------------------*/
		
		/*--------------------------------------------------------------------------------------------*/
			
		// 获取控制点坐标
		vector2d::Vector2D Get_Control_Point() const {return control_point;}
		
		// 获取起始点坐标
		vector2d::Vector2D Get_Start_Point() const {return start_point;}
		
		// 获取结束点坐标
		vector2d::Vector2D Get_End_Point() const {return end_point;}		
		
		// 获取点坐标
		vector2d::Vector2D Get_Point(float t) const;
		
		// 获取最近点及其距离
		float Get_Nearest_Distance(vector2d::Vector2D point, float* t) const;
		
		// 获取当前路程
		float Get_Current_Len(float t) const;

		// 获取曲率
		float Get_Curvature(float t) const;
		
		// 获取总长度
		float Get_len() const {return len;}
		
		// 获取曲线阶数
		BezierOrder Get_Bezier_Order() const {return order;}
		
		/*-----------------------------------------------------*/
		
		float Get_Throughout_Max_Vel() {return throughout_max_vel;}
		
		float Get_Decel() const {return decel;}
		
		/*-----------------------------------------------------*/
		
		uint16_t Get_Control_Point_Num() {return control_point_num;}
		
		uint16_t Get_End_Point_Num() {return end_point_num;}
		
		/*--------------------------------------------------------------------------------------------*/
		
		// 获取切向量
		vector2d::Vector2D Get_Tangent_Vector(float t);
		
		// 获取法向量
		vector2d::Vector2D Get_Normal_Vector(const vector2d::Vector2D& point, const float t);
		
		// 获取当前最大速度
		float Get_Max_Vel(float t);

    protected:

    private:
		/*--------------------------------------------------------------------------------------------*/
		
		vector2d::Vector2D tangent_vector;// 当前切向量
		vector2d::Vector2D normal_vector;// 当前法向量
		
		
		/*--------------------------------------------------------------------------------------------*/
		
		float end_vel = 0;// 结束速度
		float max_vel_list[BEZIER_SAMPLE_NUM + 1];// 采样点最大速度
	
		float max_curvature_len;// 最大曲率处
		float max_curvature_max_vel;// 最大曲率处的最大速度
	
		float distance_list[BEZIER_SAMPLE_NUM];// 采样点路程
		const float bezier_sample_step = 1.f / (float)BEZIER_SAMPLE_NUM;// 采样步长
		
		float len = 0;// 总长度	
	
	
		/*------------------------------------------曲线基本参数--------------------------------------------------*/

		BezierOrder order;// 阶数
		
		vector2d::Vector2D start_point;// 起始点
		vector2d::Vector2D end_point;// 结束点
		vector2d::Vector2D control_point;// 控制点
		
		uint16_t control_point_num;
		uint16_t end_point_num;
		
		/*-----------------------------------------------------*/
		
		float throughout_max_vel = 0.f;// 全程最大速度
		
		float decel = 1;
		
		/*-----------------------------------------------------*/
		
    };
}
#endif
