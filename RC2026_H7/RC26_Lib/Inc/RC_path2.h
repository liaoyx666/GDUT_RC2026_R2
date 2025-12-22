#pragma once
#include "RC_bezier_curve.h"
#include "RC_vector2d.h"
#include "RC_data_pool.h"
#include "RC_chassis.h"

#ifdef __cplusplus
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
		
    protected:
		
    private:
		vector2d::Vector2D point;
		
		float smoothness;// 平滑度
		float target_yaw;// 到达前目标yaw
		float leave_target_angle;// 离开前目标yaw
	
		// 到达前参数
		float linear_vel = 0;
		float linear_accel = 0;
		float linear_decel = 0;
	
		float angular_vel = 0;
		float angular_accel = 0;
		float angular_decel = 0;
		
    };
	
	/*------------------------------------------------------------------------------------------*/
	
	#define MAX_CURVE_NUM 30
	
	// 
	class Path2
    {
    public:
		
    protected:
		
    private:
		curve::BezierCurve curve[MAX_CURVE_NUM];
	
		float start_target_angle;// 开始前目标yaw
		
		uint16_t path_event_id = 0;
    };
	
	/*------------------------------------------------------------------------------------------*/
	
	#define MAX_PATHPOINT_NUM 50
	#define MAX_TOTAL_PATHPOINT_NUM 16000
	
	// 
	class PathPlan2
    {
    public:
		PathPlan2(data::RobotPose& robot_pose_, chassis::Chassis& robot_chassis_);
		virtual ~PathPlan2() {}
		
		void Enable();
		void Disable();
		
		void Add_End_Point(
			vector2d::Vector2D end_point_, 
			float target_yaw_, 
			float leave_target_yaw_,
			float linear_vel = 0,
			float linear_accel = 0,
			float linear_decel = 0,
			float angular_vel = 0,
			float angular_accel = 0,
			float angular_decel = 0
		);
		
		void Add_Point(
			vector2d::Vector2D point, 
			float target_yaw_, 
			float smoothness_,
			float linear_vel = 0,
			float linear_accel = 0,
			float linear_decel = 0,
			float angular_vel = 0,
			float angular_accel = 0,
			float angular_decel = 0
		);
		
		
    protected:
		
    private:
		Point2 point[MAX_PATHPOINT_NUM];
		Path2 path[2];
	
		uint8_t point_head = 0;
		uint8_t point_tail = 0;
	
		bool Add_One_Point();
		
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
		
		uint16_t total_point_num = 0;
		
		PathEvent2* path_event_list[MAX_PATH_EVENT_NUM];
    };
	/*------------------------------------------------------------------------------------------*/
}

	

#endif
