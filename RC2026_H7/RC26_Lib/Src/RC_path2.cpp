#include "RC_path2.h"

namespace path
{
	
	// 使用点就返回true
	bool Path2::Generate_Path(Point2 point_)
	{
		if (is_init == true)
		{
			return false;
		}
		
		// 曲线空间不足
		if (curve_num >= MAX_CURVE_NUM - 2)
		{
			point_.is_end = true;
		}
		
		// 限制 0 ~ 1
		point_.smoothness = fminf(fmaxf(point_.smoothness, 0.f), 1.f);
		
		// 第一个点
		if (point_num == 0)
		{
			have_start_target_angle = point_.have_leave_target_yaw;
			start_target_angle = point_.leave_target_yaw;
			temp_start_point = point_.coordinate;
			
			generate_status = GENERATE_JUST_FINISHED;
		}
		
		point_num++;
		
		switch(generate_status)
		{
			case GENERATE_JUST_FINISHED:// 刚生成完
				if (point_.is_end == true)
				{
					curve[curve_num].Bezier_Update(temp_start_point, point_.coordinate);// 生成最后直线
					total_len += curve[curve_num].Get_len();
					curve[curve_num].Set_Throughout_Max_Vel(point_.linear_vel);
					curve[curve_num].Set_Num(point_.point_num);
					curve_num++;
					
					is_init = true;
				}
				else
				{
					if (point_.smoothness < 1e-2f)
					{
						curve[curve_num].Bezier_Update(temp_start_point, point_.coordinate);// 生成直线
						total_len += curve[curve_num].Get_len();
						curve[curve_num].Set_Throughout_Max_Vel(point_.linear_vel);
						curve[curve_num].Set_Num(point_.point_num);
						curve_num++;
						
						temp_start_point = point_.coordinate;// 结束点为下一曲线起点
						generate_status = GENERATE_JUST_FINISHED;
						return true;
					}
					else
					{
						if (point_.smoothness > 1.f - 1e-2f)
						{
							// temp_start_point不变
						}
						else
						{
							temp_end_point = vector2d::Vector2D::lerp(point_.coordinate, temp_start_point, point_.smoothness);// 过渡点
							curve[curve_num].Bezier_Update(temp_start_point, temp_end_point);// 生成过渡直线
							total_len += curve[curve_num].Get_len();
							curve[curve_num].Set_Throughout_Max_Vel(PATH_MAX_PARAM);
							curve[curve_num].Set_Num(0);
							curve_num++;
							
							temp_start_point = temp_end_point;
						}
						last_smoothness = point_.smoothness;
						temp_control_point = point_.coordinate;// 控制点
						temp_control_point_num = point_.point_num;
						generate_status = GENERATE_WAIT_LAST_CURVE_POINT;
						return true;
					}
				}
				break;
			
			case GENERATE_WAIT_LAST_CURVE_POINT:// 等待最后一个曲线点
				if (point_.is_end == true)
				{
					if (last_smoothness < 1.f - 1e-2f)
					{
						temp_end_point = vector2d::Vector2D::lerp(temp_control_point, point_.coordinate, last_smoothness);
						curve[curve_num].Bezier_Update(temp_start_point, temp_control_point, temp_end_point);// 生成最后曲线
						total_len += curve[curve_num].Get_len();
						curve[curve_num].Set_Throughout_Max_Vel(PATH_MAX_PARAM);
						curve[curve_num].Set_Num(temp_control_point_num, 0);
						curve_num++;
						
						curve[curve_num].Bezier_Update(temp_end_point, point_.coordinate);// 生成最后过渡直线
						total_len += curve[curve_num].Get_len();
						curve[curve_num].Set_Throughout_Max_Vel(point_.linear_vel);
						curve[curve_num].Set_Num(0, point_.point_num);
						curve_num++;
						
					}
					else
					{
						curve[curve_num].Bezier_Update(temp_start_point, temp_control_point, point_.coordinate);// 生成最后曲线
						total_len += curve[curve_num].Get_len();
						curve[curve_num].Set_Throughout_Max_Vel(point_.linear_vel);
						curve[curve_num].Set_Num(temp_control_point_num, point_.point_num);
						curve_num++;
						
					}
					is_init = true;
				}
				else
				{
					if (point_.smoothness < 1e-2f)
					{
						if (last_smoothness < 1.f - 1e-2f)
						{
							temp_end_point = vector2d::Vector2D::lerp(temp_control_point, point_.coordinate, last_smoothness);// 过渡点
							curve[curve_num].Bezier_Update(temp_start_point, temp_control_point, temp_end_point);// 生成曲线
							total_len += curve[curve_num].Get_len();
							curve[curve_num].Set_Throughout_Max_Vel(PATH_MAX_PARAM);
							curve[curve_num].Set_Num(temp_control_point_num, 0);
							curve_num++;
							
							curve[curve_num].Bezier_Update(temp_end_point, point_.coordinate);// 生成过渡直线
							total_len += curve[curve_num].Get_len();
							curve[curve_num].Set_Throughout_Max_Vel(point_.linear_vel);
							curve[curve_num].Set_Num(0, point_.point_num);
							curve_num++;
							
						}
						else
						{
							curve[curve_num].Bezier_Update(temp_start_point, temp_control_point, point_.coordinate);// 生成曲线
							total_len += curve[curve_num].Get_len();
							curve[curve_num].Set_Throughout_Max_Vel(point_.linear_vel);
							curve[curve_num].Set_Num(temp_control_point_num, point_.point_num);
							curve_num++;
							
						}
						temp_start_point = point_.coordinate;// 结束点为下一曲线起点
						generate_status = GENERATE_JUST_FINISHED;
						return true;
					}
					else
					{
						if (last_smoothness + point_.smoothness > 1.f - 1e-2f)
						{
							temp_end_point = vector2d::Vector2D::lerp(temp_control_point, point_.coordinate, 1.f * last_smoothness / (point_.smoothness + last_smoothness));// 曲线过渡点
							curve[curve_num].Bezier_Update(temp_start_point, temp_control_point, temp_end_point);// 生成曲线
							total_len += curve[curve_num].Get_len();
							curve[curve_num].Set_Throughout_Max_Vel(PATH_MAX_PARAM);
							curve[curve_num].Set_Num(temp_control_point_num, 0);
							curve_num++;
							
							temp_start_point = temp_end_point;
							temp_control_point = point_.coordinate;
							temp_control_point_num = point_.point_num;
							generate_status = GENERATE_WAIT_LAST_CURVE_POINT;
							return true;
						}
						else
						{
							temp_end_point = vector2d::Vector2D::lerp(temp_control_point, point_.coordinate, last_smoothness);
							curve[curve_num].Bezier_Update(temp_start_point, temp_control_point, temp_end_point);// 生成曲线
							total_len += curve[curve_num].Get_len();
							curve[curve_num].Set_Throughout_Max_Vel(PATH_MAX_PARAM);
							curve[curve_num].Set_Num(temp_control_point_num, 0);
							curve_num++;
							
							temp_start_point = vector2d::Vector2D::lerp(point_.coordinate, temp_control_point, point_.smoothness);// 过度直线结束点
							curve[curve_num].Bezier_Update(temp_end_point, temp_start_point);// 曲线间过渡直线
							total_len += curve[curve_num].Get_len();
							curve[curve_num].Set_Throughout_Max_Vel(PATH_MAX_PARAM);
							curve[curve_num].Set_Num(0, 0);
							curve_num++;
							
							temp_control_point = point_.coordinate;
							temp_control_point_num = point_.point_num;
							generate_status = GENERATE_WAIT_LAST_CURVE_POINT;
							return true;
						}
					}
				}
				break;
			
			default:
				return false;
				break;
		}
		
		Calc_End_Vel();
		return true;
	}
	
	
	#define PATH2_CURVATURE_SAMPLE_STEP 0.02// m 计算曲率时的三个点采样步长
	
	
	// 计算每一段结束时最大速度
	void Path2::Calc_End_Vel()
	{
		for (int16_t i = curve_num - 1; i >= 0; i--)
		{
			if (i == curve_num - 1)
			{
				curve[i].Set_End_Vel(0.f);// 终点一定速度为0
			}
			else
			{
				float temp_vel_1, temp_vel_2, temp_vel_3;
				
				temp_vel_1 = curve[i + 1].Get_Max_Vel(0.f);// 后一段曲线起点最大速度
				
				temp_vel_3 = curve[i + 1].Get_Throughout_Max_Vel();// 后一段曲线全程最大速度
				
				// 为兼容版本1
				if (temp_vel_3 != 0.f)
				{
					temp_vel_1 = (temp_vel_1 > temp_vel_3 ? temp_vel_3 : temp_vel_1);// 取最小
				}
				
				float temp_curvature = vector2d::Vector2D::curvatureFromThreePoints(
					curve[i].Get_Point((curve[i].Get_len() - PATH2_CURVATURE_SAMPLE_STEP) / curve[i].Get_len()), 
					curve[i].Get_Point(1.f), 
					curve[i + 1].Get_Point(PATH2_CURVATURE_SAMPLE_STEP / curve[i + 1].Get_len())
				);
				
				arm_sqrt_f32(1.f / temp_curvature, &temp_vel_2);// 曲率下的最大速度

				curve[i].Set_End_Vel(temp_vel_1 > temp_vel_2 ? temp_vel_2 : temp_vel_1);// 取最小速度为终点速度
			}
		}
	}

	#define PATH2_CURVE_FINISHED_THRESHOLD  0.05f// m
	#define PATH2_START_ANGLE_THRESHOLD 2.f / 360.f * TWO_PI// 4度

	bool Path2::Get_Error_And_Vector(
		data::RobotPose * robot_pose_,
		float * target_yaw,
		float * normal_error, 
		float * tangent_error, 
		vector2d::Vector2D * normal_vector, 
		vector2d::Vector2D * tangent_vector,
		float * max_vel,
		uint16_t * current_point_num
	)
	{
		if (is_init == false) return false;
		
		vector2d::Vector2D coordinate = vector2d::Vector2D(robot_pose_->x, robot_pose_->y);
		
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		
		if (is_start == false)
		{
			if (have_start_target_angle == true)
			{
				if (fabsf(robot_pose_->yaw - start_target_angle) < PATH2_START_ANGLE_THRESHOLD)
				{
					is_start = true;
				}
			}
			else
			{
				is_start = true;// 直接开始
			}
		}

		/*--------------------------------------------------------------------------------------------------------------------------------*/
		
		*normal_error = 0.f - curve[current_curve_dx].Get_Nearest_Distance(coordinate, &current_t);// 获取最近点的t值和最近点距离
		current_curve_len = curve[current_curve_dx].Get_Current_Len(current_t);// 计算当前路程
		
		// 判断是否切换曲线
		while (current_curve_dx < curve_num && curve[current_curve_dx].Get_len() - current_curve_len < PATH2_CURVE_FINISHED_THRESHOLD && is_end == false)
		{
			if (current_curve_dx < curve_num - 1)// 
			{
				current_finished_len += curve[current_curve_dx].Get_len();// 更新已完成曲线的累计路程
	
				current_curve_dx++;// 切换路线
				curve_more_than_half = false;
				
				*normal_error = curve[current_curve_dx].Get_Nearest_Distance(coordinate, &current_t);// 重新获取最近点的t值和最近点距离
				current_curve_len = curve[current_curve_dx].Get_Current_Len(current_t);// 重新计算当前路程
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
			
			*normal_vector = curve[current_curve_dx].Get_Start_Point() - coordinate;// 法向量指向起点
			*normal_error = (*normal_vector).length();
			
			*normal_vector = (*normal_vector).normalize();

		}
		else if (current_t >= 1)// 直接锁定终点
		{
			*tangent_error = 0;
			*tangent_vector = vector2d::Vector2D(0, 0);// 切向量为0
			
			*normal_vector = curve[current_curve_dx].Get_End_Point() - coordinate;// 法向量指向终点
			*normal_error = (*normal_vector).length();
			
			*normal_vector = (*normal_vector).normalize();

		}
		else// 在曲线中
		{
			current_curve_len = curve[current_curve_dx].Get_Current_Len(current_t);// 计算当前路程
			
			*tangent_error = total_len - current_curve_len - current_finished_len;
			*tangent_vector = curve[current_curve_dx].Get_Tangent_Vector(current_t);
			
			*normal_vector = curve[current_curve_dx].Get_Normal_Vector(coordinate, current_t);

		}
		
		*max_vel = curve[current_curve_dx].Get_Max_Vel(current_t);
		
		
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		
		if (current_t >= 0.5f)
		{
			curve_more_than_half = true;
		}		
		
		
		if (is_start == false)
		{
			*current_point_num = start_point_num;
		}
		else
		{
			if (curve[current_curve_dx].Get_Bezier_Order() == curve::FIRST_ORDER_BEZIER)
			{
				*current_point_num = curve[current_curve_dx].Get_End_Point_Num();
			}
			else
			{
				if (curve_more_than_half == false)
				{
					*current_point_num = curve[current_curve_dx].Get_Control_Point_Num();
				}
				else
				{
					*current_point_num = curve[current_curve_dx].Get_End_Point_Num();
				}
			}
		}
		
		
		
		
		
		return true;
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	/*-------------------------------------------------------------------------------------------*/
	
	PathPlan2::PathPlan2(data::RobotPose& robot_pose_, chassis::Chassis& robot_chassis_)
	: ManagedTask("PathPlan2Task", 30, 256, task::TASK_DELAY, 1)
	{
		
	}
	
	
	
	void PathPlan2::Task_Process()
	{
		if (path[total_path_num % 2].Is_Init() == false)
		{
			while(generate_point_num <= total_point_num)
			{
				if (path[total_path_num % 2].Generate_Path(point[generate_point_num % MAX_PATHPOINT_NUM]) == true)
				{
					generate_point_num++;
				}
				else
				{                                               
					total_path_num++;
					break;//生成已经结束 is_init = true
				}
			}
		}
		
		
	                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           	
		
		
		                                           
		

	}
	
	
	
	
	                  
	
	
	bool PathPlan2::Add_One_Point(
		vector2d::Vector2D coordinate_,	// 坐标
		float smoothness_,				// 平滑度
		float target_yaw_,				// 到达前目标yaw
		float leave_target_yaw_,		// 离开前目标yaw
		float linear_vel_,				// 到达前最大线速度
		float linear_accel_,			// 到达前最大线加速度
		float linear_decel_,			// 到达前最大线减速度
		float angular_vel_,				// 到达前最大角速度
		float angular_accel_,			// 到达前最大角加速度
		float angular_decel_,			// 到达前最大角减速度
		bool have_leave_target_yaw_,	// 是否有离开前目标yaw
		bool have_target_yaw_,			// 是否有到达前目标yaw
		bool use_tangent_yaw_,			// yaw是否朝切线方向
		bool is_end_,					// 是否为结束点
		bool have_event_				// 是否包含事件
	)
	{
		uint16_t free_space = (point_head - point_tail - 1 + MAX_PATHPOINT_NUM) % MAX_PATHPOINT_NUM;
		
		if (free_space <= 0)
		{
			return false;
		}
		else
		{
			point[point_tail].point_num = total_point_num;
			
			total_point_num++;
			
			point[point_tail].coordinate = coordinate_;// 坐标
			point[point_tail].smoothness = smoothness_;// 平滑度
			point[point_tail].target_yaw = target_yaw_;// 到达前目标yaw
			point[point_tail].leave_target_yaw = leave_target_yaw_;// 离开前目标yaw
			point[point_tail].linear_vel = (linear_vel_ > max_linear_vel ? max_linear_vel : linear_vel_);
			point[point_tail].linear_accel = (linear_accel_ > max_linear_accel ? max_linear_accel : linear_accel_);
			point[point_tail].linear_decel = (linear_decel_ > max_linear_decel ? max_linear_decel : linear_decel_);                                                                                   
			point[point_tail].angular_vel = (angular_vel_ > max_angular_vel ? max_angular_vel : angular_vel_);
			point[point_tail].angular_accel = (angular_accel_ > max_angular_accel ? max_angular_accel : angular_accel_);
			point[point_tail].angular_decel = (angular_decel_ > max_angular_decel ? max_angular_decel : angular_decel_);
			point[point_tail].use_tangent_yaw = use_tangent_yaw_;
			point[point_tail].is_end = is_end_;
			point[point_tail].have_event = have_event_;
			point[point_tail].have_leave_target_yaw = have_leave_target_yaw_;		// 是否有离开前目标yaw
			point[point_tail].have_target_yaw = have_target_yaw_;			// 是否有到达前目标yaw
			
			
			
			point_tail = (point_tail + 1) % MAX_PATHPOINT_NUM;
			
			return true;
		}
	}
	
	
	
	
	// 添加结束点
	bool PathPlan2::Add_End_Point(
		vector2d::Vector2D coordinate_, 
		bool have_event_,
		float target_yaw_,
		float leave_target_yaw_,
		float linear_vel_,
		float linear_accel_,
		float linear_decel_,
		float angular_vel_,
		float angular_accel_,
		float angular_decel_
	)
	{
		return Add_One_Point(
			coordinate_,	// 坐标
			0,				// 平滑度
			target_yaw_,				// 到达前目标yaw
			leave_target_yaw_,		// 离开前目标yaw
			linear_vel_,				// 到达前最大线速度
			linear_accel_,			// 到达前最大线加速度
			linear_decel_,			// 到达前最大线减速度
			angular_vel_,				// 到达前最大角速度
			angular_accel_,			// 到达前最大角加速度
			angular_decel_,			// 到达前最大角减速度
			(leave_target_yaw_ != PATH_NO_TARGET_YAW),	// 是否有离开前目标yaw
			(target_yaw_ != PATH_NO_TARGET_YAW),			// 是否有到达前目标yaw
			(target_yaw_ != PATH_TANGENT_YAW),				// yaw是否朝切线方向
			true,					// 是否为结束点
			have_event_				// 是否包含事件
		);
	}
	
	
	
	bool PathPlan2::Add_Point(
		vector2d::Vector2D coordinate_, 
		float smoothness_,
		float target_yaw_,
		bool have_event_,
		float linear_vel_,
		float linear_accel_,
		float linear_decel_,
		float angular_vel_,
		float angular_accel_,
		float angular_decel_
	)
	{
		return Add_One_Point(
			coordinate_,	// 坐标
			smoothness_,				// 平滑度
			target_yaw_,				// 到达前目标yaw
			PATH_NO_TARGET_YAW,		// 离开前目标yaw
			linear_vel_,				// 到达前最大线速度
			linear_accel_,			// 到达前最大线加速度
			linear_decel_,			// 到达前最大线减速度
			angular_vel_,				// 到达前最大角速度
			angular_accel_,			// 到达前最大角加速度
			angular_decel_,			// 到达前最大角减速度
			false,					// 是否有离开前目标yaw
			(target_yaw_ != PATH_NO_TARGET_YAW),			// 是否有到达前目标yaw
			(target_yaw_ != PATH_TANGENT_YAW),				// yaw是否朝切线方向
			false,					// 是否为结束点
			have_event_				// 是否包含事件
		);
	}
	
	
	
	
	
	
}