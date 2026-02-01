#pragma once
#include "RC_bezier_curve.h"
#include "RC_vector2d.h"
#include "RC_data_pool.h"
#include "RC_chassis.h"
#include "RC_task.h"
#include "RC_nonlinear_pid.h"

#ifdef __cplusplus

#define PATH_TANGENT_YAW 	1000000.f
#define PATH_MAX_PARAM 		2000000.f
#define PATH_NO_TARGET_YAW 	3000000.f
#define PATH_NO_EVENT		0

namespace path
{
	class PathEvent2;
	class PathPlan2;
	class Point2;
	class Path2;
	
	/*===================================================================================================================================================*/
	#define MAX_PATH_EVENT_NUM 20
	
	class PathEvent2
    {
    public:
		PathEvent2(uint8_t id_, PathPlan2 &path_plan_);
		virtual ~PathEvent2() {}
		
		bool Is_Start();
			
		void Continue();
		
    protected:
		
    private:
		PathPlan2* path_plan = nullptr;
		
		uint8_t id;
		
		uint16_t current_point_num = 0;
		uint16_t continue_point_num = 0;
	
		bool is_start = false;
		bool is_continue = false;
		
		friend class PathPlan2;
    };
	
	/*===================================================================================================================================================*/
	
	
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
		bool is_stop;
		bool have_event;
		bool have_target_yaw;
		bool have_leave_target_yaw;
		
		uint16_t point_num = 0;
		uint8_t event_id = 0;
		
    protected:
		
    private:
		friend class PathPlan2;
	
    };
	
	/*===================================================================================================================================================*/
	
	#define PATH2_MAX_CURVE_NUM 34
	
	
	// 生成曲线状态
	typedef enum class Path2_Generate_Curve_Status
	{
		GENERATE_JUST_FINISHED = 0,
		GENERATE_WAIT_LAST_CURVE_POINT
	} Path2_Generate_Curve_Status;
	
	
	// 
	class Path2
    {
    public:
		Path2();
		virtual ~Path2() {}
		
		bool Generate_Path(Point2 point_);
		
		bool Calculate(
			data::RobotPose * robot_pose_,
			float * normal_error, 
			float * tangent_error, 
			vector2d::Vector2D * normal_vector, 
			vector2d::Vector2D * tangent_vector,
			float * max_vel,
			uint16_t * current_point_num,
			uint16_t * arrive_point_num,
			vector2d::Vector2D * tangent_yaw_vector
		);
		
		void Reset();
		bool Is_Init() const {return is_init;}
		bool Is_Start() const {return is_start;}
		bool Is_End() const {return is_end;}
		
		bool have_start_target_angle = false;
		float start_target_angle;// 开始前目标yaw
		
    protected:
		
    private:
		void Calc_End_Vel();
		
		curve::BezierCurve curve[PATH2_MAX_CURVE_NUM];

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
	
		Path2_Generate_Curve_Status generate_status = Path2_Generate_Curve_Status::GENERATE_JUST_FINISHED;
		
		/*---------------------------------------------------*/
		
		friend class PathPlan2;
    };
	
	/*===================================================================================================================================================*/
	
	#define MAX_PATHPOINT_NUM 50
	#define MAX_TOTAL_PATHPOINT_NUM 16000
	
	// 
	class PathPlan2 : public task::ManagedTask
    {
    public:
		PathPlan2(
			data::RobotPose& robot_pose_, chassis::Chassis& robot_chassis_,
			float max_linear_vel_, float linear_accel_, float linear_decel_,
			float max_angular_vel_, float angular_accel_, float angular_decel_,
			float distance_deadzone_, float yaw_deadzone_
		);
		virtual ~PathPlan2() {}
		
		void Enable();
		void Disable();
		
		bool Add_End_Point(
			vector2d::Vector2D coordinate_,					
			float              target_yaw_,						
			float              leave_target_yaw_,				
			float              linear_vel_,				
			float              linear_accel_,											
			float              linear_decel_,												
			float              angular_vel_,											
			float              angular_accel_,														
			float              angular_decel_,													
			bool               is_stop_,																																		
			uint8_t            event_id_	
		);
		
		bool Add_Point(
			vector2d::Vector2D coordinate_,					
			float 	           smoothness_,
			float              target_yaw_,								
			float              linear_vel_,									
			float              linear_accel_,											
			float              linear_decel_,												
			float              angular_vel_,											
			float              angular_accel_,														
			float              angular_decel_,																																			
			uint8_t            event_id_	
		);
		
		bool Next_Path();
			
		bool Add_PathEvent(PathEvent2 * path_event_);
		
    protected:
		
    private:
		void Task_Process() override;
	
		bool Delete_Point(uint16_t num);
	
		bool Add_One_Point(
			vector2d::Vector2D coordinate_,					// 坐标  
			float              smoothness_,					// 平滑度						
			float              target_yaw_,					// 到达前目标yaw			
			float              leave_target_yaw_,			// 离开前目标yaw			
			float              linear_vel_,					// 到达前最大线速度						
			float              linear_accel_,				// 到达前最大线加速度								
			float              linear_decel_,				// 到达前最大线减速度									
			float              angular_vel_,				// 到达前最大角速度						
			float              angular_accel_,				// 到达前最大角加速度											
			float              angular_decel_,				// 到达前最大角减速度
			bool               use_tangent_yaw_,			// yaw是否朝切线方向												
			bool               is_end_,						// 是否为结束点		
			bool               is_stop_,					// 是否停止			
			bool               have_event_,					// 是否包含事件							
			bool               have_target_yaw_,			// 是否有到达前目标yaw						
			bool               have_leave_target_yaw_,		// 是否有离开前目标yaw											
			uint8_t            event_id_					// 事件id
		);
		
		Point2 point[MAX_PATHPOINT_NUM];
		Path2 path[2];
		
		uint16_t total_path_num = 0;
		uint16_t current_path_num = 0;

		uint8_t point_head = 0;
		uint8_t point_tail = 1;// 第一个点为全局起点
	
		uint16_t total_point_num = 0;
		uint16_t generate_point_num = 0;

		// 线速度
		float max_linear_vel = 0;
		float max_linear_accel = 0;
		float max_linear_decel = 0;	
		
		// 角速度
		float max_angular_vel = 0;
		float max_angular_accel = 0;
		float max_angular_decel = 0;
		
		// 机器人位姿
		data::RobotPose* robot_pose;
		
		// 底盘
		chassis::Chassis* robot_chassis; 
		
		// 是否使能
		bool is_enable = false;
		
		// 是否是全局的第一个点
		bool is_first_point = true;
		
		float last_vw;
		
		uint32_t last_time = 0;
		
		float last_tangent_v = 0;
		float last_normal_v = 0;
		
		uint16_t last_current_point_num = 0;// 上一次当前前一个点
		uint16_t last_arrive_point_num = 0;// 上一次最新到达的点
		
//		float distance_deadzone = 0;
//		float yaw_deadzone = 0;
		
		pid::NonlinearPid tangent_pid;
		pid::NonlinearPid normal_pid;
		pid::NonlinearPid yaw_pid;
		
		PathEvent2 * path_event_list[MAX_PATH_EVENT_NUM] = {nullptr};
    };
	/*------------------------------------------------------------------------------------------*/
}
#endif
