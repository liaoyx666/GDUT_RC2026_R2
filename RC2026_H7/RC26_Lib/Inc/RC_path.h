#pragma once
#include "RC_vector2d.h"
#include "RC_pid.h"
#include "RC_bezier_curve.h"

#ifdef __cplusplus
namespace path
{
	/*-------------------------------------------------------------------------------*/
	
	#define MAX_CURVE_NUM 10// 最大曲线数
	
	// 生成曲线状态
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
			
		bool Add_Point(vector2d::Vector2D point_, float smoothness_);// smoothness_ 0~0.5
		
		bool Add_Start_Point(vector2d::Vector2D point_, bool have_start_angle_, float start_angle_);
		
		bool Add_End_Point(vector2d::Vector2D point_, float end_angle_);
		
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
		
		void Reset();
		
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
	
	
	/*-------------------------------------------------------------------------------*/


	#define MAX_PATH_NUM	10
	
	// 规划路径状态
	typedef enum Planning_Path_Status
	{
		PLANNING_WAIT_START_POINT,
		PLANNING_WAIT_OTHER_POINT
		
	} Planning_Path_Status;
	
	
	// 路径规划器
	class PathPlan
    {
    public:
		PathPlan(float max_speed_, float max_accel_, float max_decel_);
		virtual ~PathPlan() {}
		
		bool Add_Path_Point(
			vector2d::Vector2D point_, 
			bool stop_on_arrival_, 
			float smoothness_ = 0.f, // 0~0.5
			float arrive_angle_ = 0.f, 
			bool have_leave_angle_ = false, 
			float leave_angle_ = 0.f
		);
		
		
		bool Get_Speed(
			vector2d::Vector2D location_, 
			float angle, 
			float* speed_x, 
			float* speed_y, 
			float* speed_angle
		);
		
		bool Next_Path();
		
    protected:
		pid::Pid normal_pid, tangent_pid, angle_pid;
	
		Path path_list[MAX_PATH_NUM];
		uint8_t path_num = 0;
		
		uint8_t current_path = 0;
		uint8_t current_planning_path = 0;
	
		bool is_init = false;
		
		float current_max_spd = 0;// 当前最大速度限制
	
    private:
		vector2d::Vector2D current_tangent_vector;// 切向量
		vector2d::Vector2D current_normal_vector;// 法向量
	
		float current_tangent_error = 0;
		float current_normal_error = 0;
		float target_angle = 0;
		
		uint8_t Get_Path_Space();
		
		Planning_Path_Status planning_status = PLANNING_WAIT_START_POINT;
    };
}
#endif
