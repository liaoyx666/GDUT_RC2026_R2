#include "RC_path.h"

namespace path
{
	/*--------------------------------------------------------------------------*/
	Path::Path()
	{
		bezier_curve_num = 0;

		generate_status = GENERATE_WAIT_FIRST_POINT;
		
		have_start_angle = 0;
		start_angle = 0;
		end_angle = 0;
		total_len = 0;

		last_smoothness = 0;
	}

	
	// 增加途径点
	bool Path::Add_Point(
		vector2d::Vector2D point_, 
		float smoothness_// 0~0.5
	)
	{
		return Generate_Curve(point_, smoothness_);
	}
	
	// 增加起始点
	bool Path::Add_Start_Point(
		vector2d::Vector2D point_, 
		bool have_start_angle_, 
		float start_angle_
	)
	{
		have_start_angle = have_start_angle_;
		start_angle = start_angle_;
		
		return Generate_Curve(point_, 0);
	}

	// 增加结束点
	bool Path::Add_End_Point(
		vector2d::Vector2D point_, 
		float end_angle_
	)
	{
		end_angle = end_angle_;
		
		is_init = true;
		
		return Generate_Curve(point_, 0);
	}
	
	// 生成曲线或直线
	bool Path::Generate_Curve(vector2d::Vector2D point_, float smoothness_)
	{
		if (bezier_curve_num >= MAX_CURVE_NUM - 2) return false;
		
		if (smoothness_ < 0.f) smoothness_ = 0;
		else if (smoothness_ > 0.5f) smoothness_ = 0.5f;
		
		switch(generate_status)
		{
			case GENERATE_WAIT_FIRST_POINT:// 等待第一个点
				point_list[0] = point_;
				generate_status = GENERATE_FINISHED_STRAIGHT;
				break;
			
			case GENERATE_FINISHED_STRAIGHT:// 刚生成完直线
				if (smoothness_ == 0)
				{
					bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], point_);// 直线（一阶贝塞尔）
					total_len += bezier_curve_list[bezier_curve_num].Get_len();
					bezier_curve_num++;
					
					
					point_list[0] = point_;
					
					generate_status = GENERATE_FINISHED_STRAIGHT;
				}
				else
				{
					vector2d::Vector2D temp_point = vector2d::Vector2D::lerp(point_, point_list[0], smoothness_);// 直线衔接曲线的过渡点（曲线起始点）
					last_smoothness = smoothness_;
					
					bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], temp_point);// 曲线前的衔接直线
					total_len += bezier_curve_list[bezier_curve_num].Get_len();
					bezier_curve_num++;
					
					point_list[0] = temp_point;// 曲线起始点
					point_list[1] = point_;// 曲线控制点
					
					generate_status = GENERATE_WAIT_LAST_CURVE_POINT;
				}
				break;
				
			case GENERATE_WAIT_LAST_CURVE_POINT:// 等待曲线最后的结束点
				if (smoothness_ == 0)
				{
					vector2d::Vector2D temp_point = vector2d::Vector2D::lerp(point_list[1], point_, last_smoothness);// 曲线结束点，衔接直线过渡点
					
					bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], point_list[1], temp_point);// 曲线（二阶贝塞尔）
					total_len += bezier_curve_list[bezier_curve_num].Get_len();
					bezier_curve_num++;
					
					bezier_curve_list[bezier_curve_num].Bezier_Update(temp_point, point_);// 直线
					total_len += bezier_curve_list[bezier_curve_num].Get_len();
					bezier_curve_num++;
					
					generate_status = GENERATE_FINISHED_STRAIGHT;
				}
				else
				{
					if (last_smoothness == 0.5 && smoothness_ == 0.5)// 曲线间无需直线过渡
					{
						vector2d::Vector2D temp_point = vector2d::Vector2D::lerp(point_, point_list[1], last_smoothness);// 曲线结束点（下一段曲线起始点）
					
						bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], point_list[1], temp_point);// 曲线
						total_len += bezier_curve_list[bezier_curve_num].Get_len();
						bezier_curve_num++;
						
						point_list[0] = temp_point;// 下一段曲线起始点
						point_list[1] = point_;// 下一段曲线控制点
						
						generate_status = GENERATE_WAIT_LAST_CURVE_POINT;
					}
					else// 曲线间需要直线过渡
					{
						vector2d::Vector2D temp_point = vector2d::Vector2D::lerp(point_list[1], point_, last_smoothness);// 曲线衔接直线的过渡点
						
						bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], point_list[1], temp_point);// 曲线
						total_len += bezier_curve_list[bezier_curve_num].Get_len();
						bezier_curve_num++;
						
						vector2d::Vector2D temp_point_1 = vector2d::Vector2D::lerp(point_, point_list[1], smoothness_);// 直线衔接下一段曲线的过渡点
						last_smoothness = smoothness_;

						bezier_curve_list[bezier_curve_num].Bezier_Update(temp_point, temp_point_1);// 曲线间的直线过渡
						total_len += bezier_curve_list[bezier_curve_num].Get_len();
						bezier_curve_num++;
						
						point_list[0] = temp_point_1;// 下一段曲线起始点
						point_list[1] = point_;// 下一段曲线控制点
						
						generate_status = GENERATE_WAIT_LAST_CURVE_POINT;
					}
				}
				break;
				
			default:
				generate_status = GENERATE_FINISHED_STRAIGHT;
		}
		return true;
	}
	
	
	
	#define CURVE_FINISHED_THRESHOLD  0.05f// m
	#define START_ANGLE_THRESHOLD 2.f / 360.f * TWO_PI// 4度
	
	
	
	bool Path::Get_Error_And_Vector(
		vector2d::Vector2D location_, 
		float yaw,
		float* target_yaw, 
		float* normal_error, 
		float* tangent_error, 
		vector2d::Vector2D* normal_vector, 
		vector2d::Vector2D* tangent_vector
	)
	{
		if (is_init == false) return false;
		
		
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		
		if (have_start_angle == true)// 
		{
			if (is_start == false && fabsf(yaw - start_angle) > START_ANGLE_THRESHOLD)
			{
				currnet_target_angle = start_angle;
			}
			else
			{
				is_start = true;
			}
		}
		else
		{
			is_start = true;
		}
		
		if (is_start == true)
		{
			currnet_target_angle = end_angle;
		}
		
		*target_yaw = currnet_target_angle;
		
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		
		*normal_error = 0.f - bezier_curve_list[current_bezier_curve_dx].Get_Nearest_Distance(location_, &current_t);// 获取最近点的t值和最近点距离
		current_curve_len = bezier_curve_list[current_bezier_curve_dx].Get_Current_Len(current_t);// 计算当前路程
		
		// 判断是否切换曲线
		while (current_bezier_curve_dx < bezier_curve_num && bezier_curve_list[current_bezier_curve_dx].Get_len() - current_curve_len < CURVE_FINISHED_THRESHOLD)
		{
			if (current_bezier_curve_dx < bezier_curve_num - 1)// 
			{
				current_finished_len += bezier_curve_list[current_bezier_curve_dx].Get_len();// 更新已完成曲线的累计路程
	
				current_bezier_curve_dx++;// 切换路线
				
				*normal_error = bezier_curve_list[current_bezier_curve_dx].Get_Nearest_Distance(location_, &current_t);// 重新获取最近点的t值和最近点距离
				current_curve_len = bezier_curve_list[current_bezier_curve_dx].Get_Current_Len(current_t);// 重新计算当前路程
			}
			else// 路径结束
			{
				is_end = true;
			}
		}
		
		
		if (is_start == false)// 直接锁定起点
		{
			*tangent_error = 0;
			*tangent_vector = vector2d::Vector2D(0, 0);// 切向量为0
			
			*normal_vector = bezier_curve_list[current_bezier_curve_dx].Get_Start_Point() - location_;// 法向量指向起点
			*normal_error = (*normal_vector).length();
			
			*normal_vector = (*normal_vector).normalize();
		}
		else if (current_t >= 1)// 直接锁定终点
		{
			*tangent_error = 0;
			*tangent_vector = vector2d::Vector2D(0, 0);// 切向量为0
			
			*normal_vector = bezier_curve_list[current_bezier_curve_dx].Get_End_Point() - location_;// 法向量指向终点
			*normal_error = (*normal_vector).length();
			
			*normal_vector = (*normal_vector).normalize();
		}
		else// 在曲线中
		{
			current_curve_len = bezier_curve_list[current_bezier_curve_dx].Get_Current_Len(current_t);// 计算当前路程
			
			*tangent_error = total_len - current_curve_len - current_finished_len;
			*tangent_vector = bezier_curve_list[current_bezier_curve_dx].Get_Tangent_Vector(current_t);
			
			*normal_vector = bezier_curve_list[current_bezier_curve_dx].Get_Normal_Vector(location_, current_t);
		}
		
		return true;
	}
	
	// 重置路径
	void Path::Reset()
	{
		is_init = false;
		
		bezier_curve_num = 0;
		
		generate_status = GENERATE_WAIT_FIRST_POINT;
		
		have_start_angle = 0;
		start_angle = 0;
		end_angle = 0;
		total_len = 0;// 路线总长度
		
		currnet_target_angle = 0;
		
		current_bezier_curve_dx = 0;// 
		current_t = 0;// 
		
		current_finished_len = 0;// 已完成的曲线的总长度
		current_curve_len = 0;// 当前曲线走过的长度
		
		last_smoothness = 0;

		is_end = false;
		is_start = false;
	}
	
	
	/*--------------------------------------------------------------------------*/
	
	PathPlan::PathPlan(float max_speed_, float max_accel_, float max_decel_)
	{
		normal_pid.Pid_Mode_Init(false, false, 0);
		normal_pid.Pid_Param_Init(3, 0, 0, 0, 0.001, 0, 1, 1, 0.5, 0.5, 0.5);
		

		tangent_pid.Pid_Mode_Init(false, false, 0);
		tangent_pid.Pid_Param_Init(1, 0, 0, 0, 0.001, 0, 0.5, 0.5, 0.25, 0.25, 0.25);
		
		
		angle_pid.Pid_Mode_Init(false, false, 0);
		angle_pid.Pid_Param_Init(0, 0, 0, 0, 0.001, 0, 1, 1, 0.5, 0.5, 0.5);
		
	}
	
	
	// 增加路径点
	bool PathPlan::Add_Path_Point(
		vector2d::Vector2D point_, 
		bool stop_on_arrival_, 
		float smoothness_,
		float arrive_angle_, 
		bool have_leave_angle_, 
		float leave_angle_
	)
	{
		if (Get_Path_Space() < 1) return false;
		
		switch(planning_status)
		{
			case PLANNING_WAIT_START_POINT:
				if (path_list[current_planning_path].Add_Start_Point(point_, have_leave_angle_, leave_angle_) == false)
				{
					path_list[current_planning_path].Reset();
					planning_status = PLANNING_WAIT_START_POINT;
				}
				else
				{
					planning_status = PLANNING_WAIT_OTHER_POINT;
				}
				break;
			
			case PLANNING_WAIT_OTHER_POINT:
				if (stop_on_arrival_ == true)
				{
					if (path_list[current_planning_path].Add_End_Point(point_, arrive_angle_) == false)
					{
						path_list[current_planning_path].Reset();
						planning_status = PLANNING_WAIT_START_POINT;
					}
					else
					{
						if (current_planning_path >= MAX_PATH_NUM - 1)
						{
							current_planning_path = 0;
						}
						else
						{
							current_planning_path++;
						}
						
						path_num++;
						
						planning_status = PLANNING_WAIT_START_POINT;
					}
				}
				else
				{
					if (path_list[current_planning_path].Add_Point(point_, smoothness_) == false)
					{
						path_list[current_planning_path].Reset();
						planning_status = PLANNING_WAIT_START_POINT;
					}
					else
					{
						planning_status = PLANNING_WAIT_OTHER_POINT;
					}
				}
				break;
			
			default:
				planning_status = PLANNING_WAIT_START_POINT;
				break;
		}
		
		return true;
	}

	
	// 获取底盘速度
	bool PathPlan::Get_Speed(
		vector2d::Vector2D location_, 
		float angle, 
		float* speed_x, 
		float* speed_y, 
		float* speed_angle
	)
	{
		if (path_num < 1) return false;
		
		path_list[current_path].Get_Error_And_Vector(
			location_,
			angle,
			&target_angle,
			&current_normal_error,
			&current_tangent_error,
			&current_normal_vector,
			&current_tangent_vector
		);
		
		float target_normal_spd, target_tangent_spd;
		
		normal_pid.Update_Real(-current_normal_error);
		normal_pid.Update_Target(0);
		target_normal_spd = normal_pid.Pid_Calculate();
		
		
		tangent_pid.Update_Real(-current_tangent_error);
		tangent_pid.Update_Target(0);
		target_tangent_spd = tangent_pid.Pid_Calculate();
		
		current_normal_vector = current_normal_vector * target_normal_spd;
		current_tangent_vector = current_tangent_vector * target_tangent_spd;
		
		*speed_x = current_normal_vector.data()[0] + current_tangent_vector.data()[0];
		*speed_y = current_normal_vector.data()[1] + current_tangent_vector.data()[1];
		

		return true;
	}
	
	
	// 切换下一条路径
	bool PathPlan::Next_Path()
	{
		if (path_list[current_path].Is_End() != true) return false;
		
		if (path_num < 2) return false;
		
		path_list[current_path].Reset();// 重置
		path_num--;
		
		if (current_path == MAX_PATH_NUM - 1)
		{
			current_path = 0;
		}
		else
		{
			current_path++;
		}
		
		return true;
	}

	
	// 获取剩余路径存放空间
	uint8_t PathPlan::Get_Path_Space()
	{
		return MAX_PATH_NUM - path_num - 1;// - 1防止首尾相遇
	}
}