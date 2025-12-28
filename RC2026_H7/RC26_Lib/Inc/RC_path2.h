#pragma once
#include "RC_bezier_curve.h"
#include "RC_vector2d.h"
#include "RC_data_pool.h"
#include "RC_chassis.h"
#include "RC_task.h"

#ifdef __cplusplus


#define PATH_TANGENT_YAW 	1000000.f
#define PATH_MAX_PARAM 		2000000.f
#define PATH_NO_TARGET_YAW 	3000000.f

namespace path
{
	class PathEvent2;
	class PathPlan2;
	class Point2;
	class Path2;
	
	/*------------------------------------------------------------------------------------------*/
	#define MAX_PATH_EVENT_NUM 10
	
	class PathEvent2
    {
    public:
		PathEvent2(uint8_t id_);
		virtual ~PathEvent2() {}
		
		
    protected:
		bool Is_Arrived();
		
		
    private:
		PathPlan2* path_plan = nullptr;
		
		uint8_t id;
    };
	
	/*------------------------------------------------------------------------------------------*/
	
	// 
	class Point2
    {
    public:
		vector2d::Vector2D coordinate;
		
		float smoothness;// 平滑度
		float target_yaw;// 到达前目标yaw
		float leave_target_yaw;// 离开前目标yaw
	
		// 到达前参数
		float linear_vel = 0;
		float linear_accel = 0;
		float linear_decel = 0;
	
		float angular_vel = 0;
		float angular_accel = 0;
		float angular_decel = 0;
		
		bool use_tangent_yaw;
		
		bool is_end;
		bool have_event;
		bool have_target_yaw;
		bool have_leave_target_yaw;
		
		uint16_t point_num = 0;
		
    protected:
		
    private:

    };
	
	/*------------------------------------------------------------------------------------------*/
	
	#define MAX_CURVE_NUM 30
	
	
	// 生成曲线状态
	typedef enum Path2_Generate_Curve_Status
	{
		GENERATE_JUST_FINISHED = 0,
		GENERATE_WAIT_LAST_CURVE_POINT
	} Path2_Generate_Curve_Status;
	
	
	// 
	class Path2
    {
    public:
		
		bool Generate_Path(Point2 point_);
		
		bool Get_Error_And_Vector(
			data::RobotPose * robot_pose_,
			float * target_yaw,
			float * normal_error, 
			float * tangent_error, 
			vector2d::Vector2D * normal_vector, 
			vector2d::Vector2D * tangent_vector,
			float * max_vel,
			uint16_t * current_point_num
		);
		
    protected:
		
    private:
		void Calc_End_Vel();
	
		curve::BezierCurve curve[MAX_CURVE_NUM];
		
		bool have_start_target_angle = false;
		float start_target_angle;// 开始前目标yaw
		
		uint8_t curve_num = 0;
		uint8_t point_num = 0;
	
		float total_len = 0;// 路径总长度
	
		bool is_init = false;
		bool is_end = false;
		bool is_start = false;
	
		uint16_t start_point_num;
	
		/*---------------------------------------------------*/
	
		uint8_t current_curve_dx = 0;// 当前路段对应曲线的索引
		
		float current_t = 0;// 当前坐标对应当前路段的t值
		
		float current_finished_len = 0;// 已完成的曲线的总长度
		
		float current_curve_len = 0;// 当前曲线走过的长度
	
		bool curve_more_than_half = false;// 曲线路程是否过半
	
		/*---------------------------------------------------*/
		
		vector2d::Vector2D temp_start_point;
		vector2d::Vector2D temp_control_point;
		vector2d::Vector2D temp_end_point;
		
		uint16_t temp_control_point_num = 0;
		uint16_t temp_end_point_num = 0;
	
		float last_smoothness;
	
		Path2_Generate_Curve_Status generate_status = GENERATE_JUST_FINISHED;
		
		/*---------------------------------------------------*/
    };
	
	/*------------------------------------------------------------------------------------------*/
	
	#define MAX_PATHPOINT_NUM 50
	#define MAX_TOTAL_PATHPOINT_NUM 16000
	
	// 
	class PathPlan2 : public task::ManagedTask
    {
    public:
		PathPlan2(data::RobotPose& robot_pose_, chassis::Chassis& robot_chassis_);
		virtual ~PathPlan2() {}
		
		void Enable();
		void Disable();
		
		bool Add_End_Point(
			vector2d::Vector2D end_point_, 
			bool have_event_,
			float target_yaw_, 
			float leave_target_yaw_,
			float linear_vel_ = 0,
			float linear_accel_ = 0,
			float linear_decel_ = 0,
			float angular_vel_ = 0,
			float angular_accel_ = 0,
			float angular_decel_ = 0
		);
		
		bool Add_Point(
			vector2d::Vector2D point_, 
			float smoothness_,
			float target_yaw_, 
			bool have_event_,
			float linear_vel_ = 0,
			float linear_accel_ = 0,
			float linear_decel_ = 0,
			float angular_vel_ = 0,
			float angular_accel_ = 0,
			float angular_decel_ = 0
		);
		
		
    protected:
		
    private:
		void Task_Process() override;
	
		Point2 point[MAX_PATHPOINT_NUM];
		Path2 path[2];
	
		uint8_t point_head = 0;
		uint8_t point_tail = 0;
	
		uint16_t total_point_num = 0;		
	
		bool Add_One_Point(
			vector2d::Vector2D coordinate,
			float smoothness,// 平滑度
			float target_yaw,// 到达前目标yaw
			float leave_target_angle,// 离开前目标yaw
			float linear_vel,
			float linear_accel,
			float linear_decel,
			float angular_vel,
			float angular_accel,
			float angular_decel,
			bool have_leave_target_yaw,
			bool have_target_yaw,
			bool tangent_yaw,
			bool is_end,
			bool have_event
		);
			
		// 线速度
		float max_linear_vel = 0;
		float max_linear_accel = 0;
		float max_linear_decel = 0;
		
		float current_linear_vel = 0;
		float current_linear_accel = 0;
		float current_linear_decel = 0;
		
		
		// 角速度
		float max_angular_vel = 0;
		float max_angular_accel = 0;
		float max_angular_decel = 0;
		
		float current_angular_vel = 0;
		float current_angular_accel = 0;
		float current_angular_decel = 0;
		
		// 
		data::RobotPose* robot_pose;
		
		// 
		chassis::Chassis* robot_chassis; 
		
		// 
		bool is_enable = false;
		

		
		PathEvent2* path_event_list[MAX_PATH_EVENT_NUM];
    };
	/*------------------------------------------------------------------------------------------*/
}

	

#endif
