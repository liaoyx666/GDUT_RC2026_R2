#include "RC_path2.h"

namespace path
{
	
	PathEvent2::PathEvent2(uint8_t id_)
	{
	
	}
	
	bool PathEvent2::Is_Start()
	{
		if (is_start)
		{
			is_start = false;
			return true;
		}
		else
		{
			return false;
		}
	}
	
	
	void PathEvent2::Continue()
	{
		if (current_point_num != continue_point_num)// 防止多次触发
		{
			is_continue = true;
			continue_point_num = current_point_num;
		}
	}
	
	
	/*--------------------------------------------------------------------------------*/
	
	
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
					curve[curve_num].Set_Decel(point_.linear_decel);
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
						curve[curve_num].Set_Decel(point_.linear_decel);
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
							
							curve[curve_num].Set_Decel(PATH_MAX_PARAM);
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
						
						curve[curve_num].Set_Decel(PATH_MAX_PARAM);
						curve[curve_num].Bezier_Update(temp_start_point, temp_control_point, temp_end_point);// 生成最后曲线
						total_len += curve[curve_num].Get_len();
						curve[curve_num].Set_Throughout_Max_Vel(PATH_MAX_PARAM);
						curve[curve_num].Set_Num(temp_control_point_num, 0);
						curve_num++;
						
						curve[curve_num].Set_Decel(point_.linear_decel);
						curve[curve_num].Bezier_Update(temp_end_point, point_.coordinate);// 生成最后过渡直线
						total_len += curve[curve_num].Get_len();
						curve[curve_num].Set_Throughout_Max_Vel(point_.linear_vel);
						curve[curve_num].Set_Num(0, point_.point_num);
						curve_num++;
						
					}
					else
					{
						curve[curve_num].Set_Decel(point_.linear_decel);
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
							
							curve[curve_num].Set_Decel(PATH_MAX_PARAM);
							curve[curve_num].Bezier_Update(temp_start_point, temp_control_point, temp_end_point);// 生成曲线
							total_len += curve[curve_num].Get_len();
							curve[curve_num].Set_Throughout_Max_Vel(PATH_MAX_PARAM);
							curve[curve_num].Set_Num(temp_control_point_num, 0);
							curve_num++;
							
							curve[curve_num].Set_Decel(point_.linear_decel);
							curve[curve_num].Bezier_Update(temp_end_point, point_.coordinate);// 生成过渡直线
							total_len += curve[curve_num].Get_len();
							curve[curve_num].Set_Throughout_Max_Vel(point_.linear_vel);
							curve[curve_num].Set_Num(0, point_.point_num);
							curve_num++;
							
						}
						else
						{
							curve[curve_num].Set_Decel(point_.linear_decel);
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
							
							curve[curve_num].Set_Decel(PATH_MAX_PARAM);
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
							
							curve[curve_num].Set_Decel(PATH_MAX_PARAM);
							curve[curve_num].Bezier_Update(temp_start_point, temp_control_point, temp_end_point);// 生成曲线
							total_len += curve[curve_num].Get_len();
							curve[curve_num].Set_Throughout_Max_Vel(PATH_MAX_PARAM);
							curve[curve_num].Set_Num(temp_control_point_num, 0);
							curve_num++;
							
							temp_start_point = vector2d::Vector2D::lerp(point_.coordinate, temp_control_point, point_.smoothness);// 过度直线结束点
							
							curve[curve_num].Set_Decel(PATH_MAX_PARAM);
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
		float * normal_error, 
		float * tangent_error, 
		vector2d::Vector2D * normal_vector, 
		vector2d::Vector2D * tangent_vector,
		float * max_vel,
		uint16_t * current_point_num,
		uint16_t * arrive_point_num,
		vector2d::Vector2D * tangent_yaw_vector
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
			
			*tangent_yaw_vector = curve[current_curve_dx].Get_Tangent_Vector(0);
		}
		else if (current_t >= 1)// 直接锁定终点
		{
			*tangent_error = 0;
			*tangent_vector = vector2d::Vector2D(0, 0);// 切向量为0
			
			*normal_vector = curve[current_curve_dx].Get_End_Point() - coordinate;// 法向量指向终点
			*normal_error = (*normal_vector).length();
			
			*normal_vector = (*normal_vector).normalize();

			*tangent_yaw_vector = curve[current_curve_dx].Get_Tangent_Vector(1);
		}
		else// 在曲线中
		{
			current_curve_len = curve[current_curve_dx].Get_Current_Len(current_t);// 计算当前路程
			
			*tangent_error = total_len - current_curve_len - current_finished_len;
			*tangent_vector = curve[current_curve_dx].Get_Tangent_Vector(current_t);
			
			*normal_vector = curve[current_curve_dx].Get_Normal_Vector(coordinate, current_t);
			
			*tangent_yaw_vector = *tangent_vector;
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
			*arrive_point_num = start_point_num;
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
					
					if (current_curve_dx > 1)
					{
						*arrive_point_num = curve[current_curve_dx - 1].Get_End_Point_Num();
					}
					else
					{
						*arrive_point_num = start_point_num;
					}
				}
				else
				{
					*current_point_num = curve[current_curve_dx].Get_End_Point_Num();
					*arrive_point_num = curve[current_curve_dx].Get_Control_Point_Num();
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
	
	bool PathPlan2::Next_Path()
	{
		if (path[current_path_num % 2].Is_End() && path[(current_path_num + 1) % 2].Is_Init())
		{
			current_path_num++;
			return true;
		}
		else
		{
			return false;
		}
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
		
		/*---------------------------------------------------------------------------*/
		
		
		float normal_error;
		float tangent_error;
		
		vector2d::Vector2D normal_vector;
		vector2d::Vector2D tangent_vector;
		vector2d::Vector2D tangent_yaw_vector;
		
		float max_vel;
		uint16_t current_point_num;
		uint16_t arrive_point_num;
		
		path[current_path_num % 2].Get_Error_And_Vector(
			robot_pose,
			&normal_error, 
			&tangent_error, 
			&normal_vector, 
			&tangent_vector,
			&max_vel,
			&current_point_num,
			&arrive_point_num,
			&tangent_yaw_vector
		);
		
		/*---------------------------------------------------------------------------*/
		
		// 判断是否到达终点
		if (path[current_path_num % 2].Is_End())
		{
			// 判断是否有事件发生
			if (point[current_point_num % MAX_PATHPOINT_NUM].have_event && point[current_point_num % MAX_PATHPOINT_NUM].is_stop)
			{
				// 有事件
				if (path_event_list[point[current_point_num % MAX_PATHPOINT_NUM].event_id - 1]->current_point_num != current_point_num)// 防止重复触发
				{
					path_event_list[point[current_point_num % MAX_PATHPOINT_NUM].event_id - 1]->is_start = true;// 事件开始
					path_event_list[point[current_point_num % MAX_PATHPOINT_NUM].event_id - 1]->current_point_num = current_point_num;// 更新事件点
				}
				
				if (path_event_list[point[current_point_num % MAX_PATHPOINT_NUM].event_id - 1]->is_continue)
				{
					Next_Path();
					path_event_list[point[current_point_num % MAX_PATHPOINT_NUM].event_id - 1]->is_continue = false;
				}
			}
			else
			{
				// 无事件，直接切换下一条路径
				Next_Path();
			}
		}
		else
		{
			if (point[arrive_point_num % MAX_PATHPOINT_NUM].have_event)
			{
				if (path_event_list[point[current_point_num % MAX_PATHPOINT_NUM].event_id - 1]->current_point_num != current_point_num)// 防止重复触发
				{
					path_event_list[point[current_point_num % MAX_PATHPOINT_NUM].event_id - 1]->is_start = true;// 事件开始
					path_event_list[point[current_point_num % MAX_PATHPOINT_NUM].event_id - 1]->current_point_num = current_point_num;// 更新
				}
			}
		}

		/*-------------------------------------------------------------------------------------------*/
		
		float current_linear_vel    = point[current_point_num % MAX_PATHPOINT_NUM].linear_vel;
		float current_linear_accel  = point[current_point_num % MAX_PATHPOINT_NUM].linear_accel;
		float current_linear_decel  = point[current_point_num % MAX_PATHPOINT_NUM].linear_decel;
		
		float current_angular_vel   = point[current_point_num % MAX_PATHPOINT_NUM].angular_vel;
		float current_angular_accel = point[current_point_num % MAX_PATHPOINT_NUM].angular_accel;
		float current_angular_decel = point[current_point_num % MAX_PATHPOINT_NUM].angular_decel;
		
		bool use_tangent_yaw        = point[current_point_num % MAX_PATHPOINT_NUM].use_tangent_yaw;
		
		bool have_leave_target_yaw  = point[current_point_num % MAX_PATHPOINT_NUM].have_leave_target_yaw;


        
		float target_yaw = 0;

		
		if (have_leave_target_yaw == true && path[current_path_num % 2].Is_Start() == false)
		{
			target_yaw = path[current_path_num % 2].start_target_angle;// 起始目标角度
		}
		else if (use_tangent_yaw == true)
		{
			target_yaw = tangent_yaw_vector.angle();// 切向角度
		}
		else
		{
			target_yaw = point[current_point_num % MAX_PATHPOINT_NUM].target_yaw;// 目标角度
		}

		
		float dt = (float)timer::Timer::Get_DeltaTime(last_time) / 1000000.f;// us->s
		last_time = timer::Timer::Get_TimeStamp();



		// 计算法向速度
		float normal_v  = normal_pid.NPid_Calculate(0, -normal_error);
		
		// 计算切向速度
		float tangent_v = tangent_pid.NPid_Calculate(0, -tangent_error);

		// 计算角速度
		float vw        = yaw_pid.NPid_Calculate(target_yaw, robot_pose->yaw, true, PI);



		float delta = 0;

		delta = chassis::Limit_Accel(tangent_v - last_tangent_v, current_linear_accel, dt);
		
		if (delta < 0)
		{
			// 不限制减速
		}
		else
		{
			tangent_v = last_tangent_v + delta;// 限制加速
		}

		delta = chassis::Limit_Accel(vw - last_vw, current_angular_accel, dt);
		
		if (delta < 0)
		{
			// 不限制减速
		}
		else
		{
			vw = last_vw + delta;// 限制加速
		}
		
		
		last_tangent_v = tangent_v;
		last_normal_v  = normal_v;
		last_vw        = vw;


	
		
		tangent_v = tangent_v > max_vel             ? max_vel             : tangent_v;
		tangent_v = tangent_v > current_linear_vel  ? current_linear_vel  : tangent_v;
		normal_v  = normal_v  > max_linear_vel      ? max_linear_vel      : normal_v;
		vw        = vw        > current_angular_vel ? current_angular_vel : vw;
		
		

		// 向量化
		normal_vector  = normal_vector  * normal_v;
		tangent_vector = tangent_vector * tangent_v;
		
		vector2d::Vector2D v = normal_vector + tangent_vector;
		
		robot_chassis->Set_World_Vel(v, vw, robot_pose->yaw);
	}
	

	bool PathPlan2::Add_One_Point(
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
	)								
	{
		uint16_t free_space = (point_head - point_tail - 1 + MAX_PATHPOINT_NUM) % MAX_PATHPOINT_NUM;
		
		if (free_space <= 0)
		{
			return false;
		}
		else
		{
			target_yaw_       = fmaxf(fminf(target_yaw_, PI), -PI);
			leave_target_yaw_ = fmaxf(fminf(leave_target_yaw_, PI), -PI);

			
			point[point_tail].coordinate            = coordinate_;// 坐标
											       
			point[point_tail].smoothness            = smoothness_;// 平滑度
			point[point_tail].target_yaw            = target_yaw_;// 到达前目标yaw
			point[point_tail].leave_target_yaw      = leave_target_yaw_;// 离开前目标yaw 
			point[point_tail].linear_vel            = (linear_vel_    > max_linear_vel    ? max_linear_vel    : linear_vel_   );
			point[point_tail].linear_accel          = (linear_accel_  > max_linear_accel  ? max_linear_accel  : linear_accel_ );
			point[point_tail].linear_decel          = (linear_decel_  > max_linear_decel  ? max_linear_decel  : linear_decel_ );                                                                                   
			point[point_tail].angular_vel           = (angular_vel_   > max_angular_vel   ? max_angular_vel   : angular_vel_  );
			point[point_tail].angular_accel         = (angular_accel_ > max_angular_accel ? max_angular_accel : angular_accel_);
			point[point_tail].angular_decel         = (angular_decel_ > max_angular_decel ? max_angular_decel : angular_decel_);

			point[point_tail].use_tangent_yaw       = use_tangent_yaw_;
			point[point_tail].is_end                = is_end_;
			point[point_tail].is_stop               = is_stop_;
			point[point_tail].have_event            = have_event_;
			point[point_tail].have_target_yaw       = have_target_yaw_;			// 是否有到达前目标yaw
			point[point_tail].have_leave_target_yaw = have_leave_target_yaw_;		// 是否有离开前目标yaw
			
			point[point_tail].point_num             = total_point_num;
			point[point_tail].event_id              = event_id_;


			total_point_num++;
	
			point_tail = (point_tail + 1) % MAX_PATHPOINT_NUM;
			
			return true;
		}
	}
	
	
	
	

	
	
	
	
	
	// 添加结束点
	bool PathPlan2::Add_End_Point(
		vector2d::Vector2D coordinate_,				// 坐标  
					
		float              target_yaw_,				// 到达前目标yaw					
		float              leave_target_yaw_,		// 离开前目标yaw					
		float              linear_vel_,				// 到达前最大线速度							
		float              linear_accel_,			// 到达前最大线加速度									
		float              linear_decel_,			// 到达前最大线减速度										
		float              angular_vel_,			// 到达前最大角速度										
		float              angular_accel_,			// 到达前最大角加速度												
		float              angular_decel_,		    // 到达前最大角减速度
																					
		bool               is_stop_,				// 是否停止																				
														
		uint8_t            event_id_				// 事件id
	)
	{
		return Add_One_Point(
			coordinate_,									// 坐标  
		                                                    
			0,												// 平滑度						
			target_yaw_,				                    // 到达前目标yaw			
			leave_target_yaw_,			                    // 离开前目标yaw			
			linear_vel_,				                    // 到达前最大线速度			
			linear_accel_,				                    // 到达前最大线加速度		
			linear_decel_,				                    // 到达前最大线减速度		
			angular_vel_,				                    // 到达前最大角速度			
			angular_accel_,				                    // 到达前最大角加速度		
			angular_decel_,				                    // 到达前最大角减速度
		                                                    
			(target_yaw_ == PATH_TANGENT_YAW),              // yaw是否朝切线方向		
			true,									        // 是否为结束点		
			is_stop_,                                       // 是否停止			
			(event_id_ != PATH_NO_EVENT),					// 是否包含事件				
			(target_yaw_ != PATH_NO_TARGET_YAW),	        // 是否有到达前目标yaw		
			(leave_target_yaw_ != PATH_NO_TARGET_YAW),		// 是否有离开前目标yaw
			                                                
			event_id_                                       // 事件id
		);
	}
	
	
	
	bool PathPlan2::Add_Point(
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
	)
	{
		return Add_One_Point(
			coordinate_,								// 坐标  
		                                                
			smoothness_,			                    // 平滑度					
			target_yaw_,			                    // 到达前目标yaw			
			0,		                   				    // 离开前目标yaw			
			linear_vel_,			                    // 到达前最大线速度			
			linear_accel_,			                    // 到达前最大线加速度		
			linear_decel_,			                    // 到达前最大线减速度		
			angular_vel_,			                    // 到达前最大角速度			
			angular_accel_,			                    // 到达前最大角加速度		
			angular_decel_,			                    // 到达前最大角减速度
			                                            
			(target_yaw_ == PATH_TANGENT_YAW),          // yaw是否朝切线方向		
			false,		                                // 是否为结束点		
			false,                                      // 是否停止			
			(event_id_ != PATH_NO_EVENT),			    // 是否包含事件				
			(target_yaw_ != PATH_NO_TARGET_YAW),	    // 是否有到达前目标yaw		
			false,				                        // 是否有离开前目标yaw
			                                            
			event_id_                                   // 事件id
		);
	}
	
	
	
	
	
	
}