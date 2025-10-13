#pragma once
#include "RC_vector2d.h"
#include "RC_pid.h"
#include "RC_bezier_curve.h"
#include "RC_chassis.h"

#ifdef __cplusplus

#define MAX_CURVE_NUM 	9// 最大曲线数
#define MAX_PATH_NUM	20// 最大路径数

namespace path
{
	/*----------------------------------------------------------------------------------------------------------------------*/
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
		
		// 添加途径点
		bool Add_Point(vector2d::Vector2D point_, float smoothness_);// smoothness_ 0~0.5
		
		// 添加起始点
		bool Add_Start_Point(vector2d::Vector2D point_, bool have_start_angle_, float start_angle_);
		
		// 添加结束点
		bool Add_End_Point(vector2d::Vector2D point_, float end_angle_);
		
		// 获取法向，切向误差和向量，获取目标yaw和最大速度限制
		////////////////////////////////////////////////
		bool Get_Error_And_Vector(
			vector2d::Vector2D location_, 
			float yaw,
			float* target_yaw,
			float* normal_error, 
			float* tangent_error, 
			vector2d::Vector2D* normal_vector, 
			vector2d::Vector2D* tangent_vector,
			float* max_vel
		);
		
		// 获取路径是否结束
		bool Is_End() {return is_end;}
		
		// 重置路径
		void Reset();
		
    protected:
		// 计算各路段的结束路段
		void Calc_End_Vel();
		
		// 生成各路段的曲线
		bool Generate_Curve(vector2d::Vector2D point_, float smoothness_);
		
		curve::BezierCurve bezier_curve_list[MAX_CURVE_NUM];// 储存各路段曲线
		uint8_t bezier_curve_num = 0;// 总曲线数量
		
		/*------------------------------路径参数-----------------------------------*/
		bool have_start_angle = 0;// 是否要求达到起始角度后再启动
		float start_angle = 0;// 起始角度
		float end_angle = 0;// 结束角度
		float total_len = 0;// 路线总长度
		

		/*------------------------------过程变量-----------------------------------*/
		float currnet_target_angle = 0;// 当前目标角度
		
		uint8_t current_bezier_curve_dx = 0;// 当前路段对应曲线的索引
		
		float current_t = 0;// 当前坐标对应当前路段的t值
		
		float current_finished_len = 0;// 已完成的曲线的总长度
		
		float current_curve_len = 0;// 当前曲线走过的长度

    private:
		/*---------------------------------状态-------------------------------------*/
		bool is_end = false;// 是否开始
		bool is_start = false;// 是否结束
	
		bool is_init = false;// 是否初始化
	
	
		/*----------------------------生成路径的临时变量-------------------------------------*/
		Generate_Curve_Status generate_status = GENERATE_WAIT_FIRST_POINT;// 生成曲线的状态
		
		vector2d::Vector2D point_list[3];// 生成曲线时临时存放点坐标
	
		float last_smoothness;// 生成曲线时临时存放上一个点的平滑程度

    };
	

	/*--------------------------------------------------------------------------------------------------------------------*/
	// 规划路径状态
	typedef enum Planning_Path_Status
	{
		PLANNING_WAIT_START_POINT,
		PLANNING_WAIT_OTHER_POINT
		
	} Planning_Path_Status;
	
	
	// 路径规划器（可以同时控制运动和规划路径）
	class PathPlan
    {
    public:
		PathPlan(float max_speed_, float max_accel_);
		virtual ~PathPlan() {}
		
		// 添加路径点
		bool Add_Path_Point(
			vector2d::Vector2D point_, // 坐标
			bool stop_on_arrival_, // 到达时是否停止
			float smoothness_ = 0.f, // 平滑程度0~0.5
			float arrive_angle_ = 0.f, // 到达时期望角度
			bool have_leave_angle_ = false, // 是否要求离开时到达离开时期望角度
			float leave_angle_ = 0.f //离开时期望角度
		);
			
		// 机器人启动点，只有一个
		bool Add_Start_Point(vector2d::Vector2D point_);
			
		// 途径点
		bool Add_Point(vector2d::Vector2D point_, float smoothness_);
			
		// 结束点，下一段起始点
		bool Add_End_Point(vector2d::Vector2D point_, float end_angle_, bool have_start_angle_ = false, float start_angle_ = 0.f);
		
		// 获取目标x，y，yaw速度
		bool Get_Speed(
			vector2d::Vector2D location_, // 机器人坐标
			float angle, // yaw
			float* speed_x, 
			float* speed_y, 
			float* speed_angle
		);
		
		// 切换下一路径
		bool Next_Path();
			
		bool Is_End();
		
    protected:
		// 获取剩余储存路径的空间
		uint8_t Get_Path_Space();
	
		Path path_list[MAX_PATH_NUM];// 储存所有路径
		uint8_t path_num = 0;// 路径数量
	
	
		/*---------------------------------路径规划参数--------------------------------------*/
		pid::Pid normal_pid, tangent_pid, angle_pid;// 法向，切向，yaw的pid

		float max_speed = 0;// 最大速度
		float max_accel = 0;// 最大加速度

		Planning_Path_Status planning_status = PLANNING_WAIT_START_POINT;// 规划路径的状态

	
		/*---------------------------------控制过程中的变量----------------------------------------*/
		vector2d::Vector2D current_tangent_vector;// 当前切向量
		vector2d::Vector2D current_normal_vector;// 当前法向量
	
		float current_tangent_error = 0;// 当前切向误差
		float current_normal_error = 0;// 当前法向误差
		
		float target_angle = 0;// 当前期望角度

		uint8_t current_path = 0;// 机器人当前路径

		float current_max_tangent_spd = 0;// 当前最大切向速度
	
		uint8_t last_time = 0;
		
		float target_normal_spd, target_tangent_spd;
		float last_target_tangent_spd = 0;
    private:
		/*----------------------------状态--------------------------*/
		bool is_init = false;
		uint8_t current_planning_path = 0;// 当前处于规划中规划的路径
	
    };
}
#endif
