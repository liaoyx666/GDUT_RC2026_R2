#pragma once
#include "RC_vector2d.h"
#include "RC_pid.h"
#include "RC_bezier_curve.h"

#ifdef __cplusplus
namespace path
{
	// 路径点
	class PathPoint
    {
    public:
		PathPoint();
		PathPoint(
			vector2d::Vector2D point_, 
			bool stop_on_arrival_, 
			float smoothness_ = 0.f, // 0~1
			float arrive_angle_ = 0.f, 
			bool have_leave_angle_ = false, 
			float leave_angle_ = 0.f
		);
		
		void Path_Point_Update(
			vector2d::Vector2D point_, 
			bool stop_on_arrival_, 
			float smoothness_ = 0.f, // 0~1
			float arrive_angle_ = 0.f, 
			bool have_leave_angle_ = false, 
			float leave_angle_ = 0.f
		);
		
		virtual ~PathPoint() {}
		
		
		
    protected:
		vector2d::Vector2D point;
		bool stop_on_arrival;
		float smoothness = 0.f; // 0~1
		float arrive_angle = 0.f; 
		bool have_leave_angle = false; 
		float leave_angle = 0.f;
		
		bool is_init = false;
	
    private:
		
		
    };
	
	
#define MAX_CURVE_NUM 8
	
	typedef enum Generate_Curve_Status
	{
		GENERATE_WAIT_FIRST_POINT = 0,
		GENERATE_FINISHED_STRAIGHT,
		GENERATE_WAIT_LAST_CURVE_POINT
	} Generate_Curve_Status;
	
	
	
	// 路径（从静止启动到静止）
	class Path
    {
    public:
		Path();
		virtual ~Path() {}
			
		bool Add_Point(
			vector2d::Vector2D point_, 
			float smoothness_// 0~0.5
		);
		
		bool Add_Start_Point(
			vector2d::Vector2D point_, 
			bool have_start_angle_, 
			float start_angle_
		);
		
		bool Add_End_Point(
			vector2d::Vector2D point_, 
			float end_angle_
		);
		
		
		
		bool Get_Error_And_Vector(
			vector2d::Vector2D location_, 
			float yaw,
			float* target_yaw,
			float* normal_error, 
			float* tangent_error, 
			vector2d::Vector2D* normal_vector, 
			vector2d::Vector2D* tangent_vector
		);
		
		
		bool Is_End() {return is_end;}

    protected:
		bool Generate_Curve(vector2d::Vector2D point_, float smoothness_);
	
	
	
		curve::BezierCurve bezier_curve_list[MAX_CURVE_NUM];
		uint8_t bezier_curve_num = 0;
		
		vector2d::Vector2D point_list[3];
		
		Generate_Curve_Status generate_status = GENERATE_WAIT_FIRST_POINT;
		
		bool have_start_angle = 0;
		float start_angle = 0;
		float end_angle = 0;
		float total_len = 0;// 路线总长度
		
		float currnet_target_angle = 0;
	
	
	
	
	
		bool is_init = false;
		
		uint8_t current_bezier_curve_dx = 0;// 
		float current_t = 0;// 
		
		float current_finished_len = 0;// 已完成的曲线的总长度
		float current_curve_len = 0;// 当前曲线走过的长度
		
		float last_smoothness;
    private:
		bool is_end = false;
		bool is_start = false;
    
    };
	
	
	

//#define MAX_PATH_POINT_NUM	20
	
	// 路径规划器
	class PathPlan
    {
    public:
		PathPlan(float max_speed_, float max_accel_, float max_decel_);
		virtual ~PathPlan() {}
		
		bool Add_Path_Point(
			vector2d::Vector2D point_, 
			bool stop_on_arrival_, 
			float smoothness_ = 0.f, // 0~1
			float arrive_angle_ = 0.f, 
			bool have_leave_angle_ = false, 
			float leave_angle_ = 0.f
		);
		
		void Delete_Path_Point();
		
		
    protected:
//		PathPoint path_point_list[MAX_PATH_POINT_NUM];
//	
//		uint8_t path_point_head = 0;
//		uint8_t path_point_tail = 0;
//	
//		uint8_t current_target_point_dx = 0;
//		
//		uint8_t Get_Point_Space();
	
	
	
	
	
	
	
	
    private:
    
    };
}
#endif
