#pragma once
#include "RC_bezier_curve.h"
#include "RC_vector2d.h"
#include "RC_data_pool.h"
#include "RC_chassis.h"

#ifdef __cplusplus
namespace path
{
	// 
	class Point2
    {
    public:
		
    protected:
		
    private:
		vector2d::Vector2D point;
    };
	
	
	
	#define MAX_CURVE_NUM 20
	// 
	class Path2
    {
    public:
		
    protected:
		
    private:
		curve::BezierCurve curve[MAX_CURVE_NUM];
    };
	
	
	
	#define MAX_PATHPOINT_NUM 30
	// 
	class PathPlan2
    {
    public:
		PathPlan2(data::RobotPose& robot_pose_, chassis::Chassis& robot_chassis_);
		virtual ~PathPlan2() {}
    protected:
		
    private:
		Point2 point[MAX_PATHPOINT_NUM];
		Path2 path[2];
		
		// 线速度
		float max_linear_vel = 0;
		float max_linear_accel = 0;
		float max_linear_decel = 0;
	
		float linear_vel = 0;
		float linear_accel = 0;
		float linear_decel = 0;
		
	
		// 角速度
		float max_angular_vel = 0;
		float max_angular_accel = 0;
		float max_angular_decel = 0;
	
		float angular_vel = 0;
		float angular_accel = 0;
		float angular_decel = 0;
		
		// 
		data::RobotPose* robot_pose;
		
		// 
		chassis::Chassis* robot_chassis;
    };
}
#endif
