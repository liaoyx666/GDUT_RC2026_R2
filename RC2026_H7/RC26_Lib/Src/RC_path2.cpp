#include "RC_path2.h"

namespace path
{
	PathEvent2::PathEvent2(uint8_t id_, PathPlan2 &path_plan_) : path_plan(&path_plan_)
	{
		if (id_ >= 1 && id_ <= MAX_PATH_EVENT_NUM)
		{
			id = id_;
			
			path_plan->Add_PathEvent(this);
		}
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
	Path2::Path2()
	{
		Reset();
	}
	
	void Path2::Reset()
	{
		curve_num = 0;
		point_num = 0;
	
		total_len = 0;// 路径总长度
	
		is_init = false;
		is_end = false;
		is_start = false;

		current_curve_dx = 0;// 当前路段对应曲线的索引
		current_t = 0;// 当前坐标对应当前路段的t值
		current_finished_len = 0;// 已完成的曲线的总长度
		current_curve_len = 0;// 当前曲线走过的长度
		
		have_start_target_angle = false;
		
		curve_more_than_half = false;// 曲线路程是否过半
		temp_control_point_num = 0;
		temp_end_point_num = 0;
		last_smoothness = 0;
		generate_status = Path2_Generate_Curve_Status::GENERATE_JUST_FINISHED;
	}

	// 使用点就返回true
	bool Path2::Generate_Path(Point2 point_)
	{
		if (is_init == true)
		{
			return false;
		}
		
		// 曲线空间不足（每次最多增加2条路径）
		if (curve_num >= PATH2_MAX_CURVE_NUM - 4)
		{
			// 强制设为结束点
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
			start_point_num = point_.point_num;
			
			generate_status = Path2_Generate_Curve_Status::GENERATE_JUST_FINISHED;

			point_num++;
			return true;
		}
		
		point_num++;
		
		switch(generate_status)
		{
			case Path2_Generate_Curve_Status::GENERATE_JUST_FINISHED:// 刚生成完
				if (point_.is_end == true)
				{
					curve[curve_num].Set_Decel(point_.linear_decel);
					curve[curve_num].Bezier_Update(temp_start_point, point_.coordinate);// 生成最后直线
					total_len += curve[curve_num].Get_len();
					curve[curve_num].Set_Throughout_Max_Vel(point_.linear_vel);
					curve[curve_num].Set_Num(point_.point_num);
					curve_num++;
					////// +1（END）
					
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
						////// +1
						
						temp_start_point = point_.coordinate;// 结束点为下一曲线起点
						generate_status = Path2_Generate_Curve_Status::GENERATE_JUST_FINISHED;
						
						is_init = false;
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
							////// +1
							
							temp_start_point = temp_end_point;
						}
						
						last_smoothness = point_.smoothness;
						temp_control_point = point_.coordinate;// 控制点
						temp_control_point_num = point_.point_num;
						generate_status = Path2_Generate_Curve_Status::GENERATE_WAIT_LAST_CURVE_POINT;
						
						is_init = false;
					}
				}
				break;
			
			case Path2_Generate_Curve_Status::GENERATE_WAIT_LAST_CURVE_POINT:// 等待最后一个曲线点
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
						////// +2（END）
					}
					else
					{
						curve[curve_num].Set_Decel(point_.linear_decel);
						curve[curve_num].Bezier_Update(temp_start_point, temp_control_point, point_.coordinate);// 生成最后曲线
						total_len += curve[curve_num].Get_len();
						curve[curve_num].Set_Throughout_Max_Vel(point_.linear_vel);
						curve[curve_num].Set_Num(temp_control_point_num, point_.point_num);
						curve_num++;
						////// +1（END）
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
							////// +2
						}
						else
						{
							curve[curve_num].Set_Decel(point_.linear_decel);
							curve[curve_num].Bezier_Update(temp_start_point, temp_control_point, point_.coordinate);// 生成曲线
							total_len += curve[curve_num].Get_len();
							curve[curve_num].Set_Throughout_Max_Vel(point_.linear_vel);
							curve[curve_num].Set_Num(temp_control_point_num, point_.point_num);
							curve_num++;
							////// +1
						}
						
						temp_start_point = point_.coordinate;// 结束点为下一曲线起点
						generate_status = Path2_Generate_Curve_Status::GENERATE_JUST_FINISHED;
						
						is_init = false;
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
							////// +1
							
							temp_start_point = temp_end_point;
							temp_control_point = point_.coordinate;
							temp_control_point_num = point_.point_num;
							generate_status = Path2_Generate_Curve_Status::GENERATE_WAIT_LAST_CURVE_POINT;
							
							is_init = false;
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
							////// +2
							
							temp_control_point = point_.coordinate;
							temp_control_point_num = point_.point_num;
							generate_status = Path2_Generate_Curve_Status::GENERATE_WAIT_LAST_CURVE_POINT;
							
							is_init = false;
						}
					}
				}
				break;
			
			default:
				return false;
				break;
		}
		
		if (is_init)
		{
			Calc_End_Vel();
		}
			
		return true;
	}
	
	
	#define PATH2_CURVATURE_SAMPLE_STEP 0.01// m 计算曲率时的三个点采样步长
	
	
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
				
				temp_curvature = fmaxf(fabsf(temp_curvature), 1e-6f); // 最小曲率限制（避免除零）
				
				arm_sqrt_f32(curve[i].Get_Decel() / temp_curvature, &temp_vel_2);// 曲率下的最大速度

				curve[i].Set_End_Vel(temp_vel_1 > temp_vel_2 ? temp_vel_2 : temp_vel_1);// 取最小速度为终点速度
			}
		}
	}
	
	#define PATH2_CURVE_FINISHED_THRESHOLD  0.02f// m
	#define PATH2_START_ANGLE_THRESHOLD 2.f / 360.f * TWO_PI// 4度

	bool Path2::Calculate(
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
		if (is_init == false)
		{
			*normal_error = 0;
			*tangent_error = 0; 
			*normal_vector = vector2d::Vector2D(); 
			*tangent_vector = vector2d::Vector2D();
			*max_vel = 0;
			*current_point_num = 0;
			*arrive_point_num = 0;
			*tangent_yaw_vector = vector2d::Vector2D();
			
			return false;
		}
		
		// 坐标
		vector2d::Vector2D coordinate = vector2d::Vector2D(*robot_pose_->Get_pX(), *robot_pose_->Get_pY());
		
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		
		if (is_start == false)
		{
			// 是否有发前目标yaw
			if (have_start_target_angle == true)
			{
				float yaw_error = fabsf(*robot_pose_->Get_pYaw() - start_target_angle);
				
				yaw_error = fmodf(yaw_error, TWO_PI);
				
				// 到达出发前目标yaw后出发
				if (yaw_error < PATH2_START_ANGLE_THRESHOLD)
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
		
		// 获取最近点的t值和最近点距离
		*normal_error = 0.f - curve[current_curve_dx].Get_Nearest_Distance(coordinate, &current_t);
		
		// 计算当前路程
		current_curve_len = curve[current_curve_dx].Get_Current_Len(current_t);
		
		// 判断是否切换曲线
		while (
			(current_curve_dx < curve_num) && 															// 防止越界
			(curve[current_curve_dx].Get_len() - current_curve_len < PATH2_CURVE_FINISHED_THRESHOLD) && // 完成曲线
			(!is_end) 																					// 路径还没结束
		)
		{
			// 判断当前不是最后一条曲线
			if (current_curve_dx < curve_num - 1) 
			{
				// 更新已完成曲线的累计路程
				current_finished_len += curve[current_curve_dx].Get_len(); 
				
				// 切换路线
				current_curve_dx++;
				
				// 更新状态，曲线还未过半
				curve_more_than_half = false;
				
				// 重新获取最近点的t值和最近点距离
				*normal_error = curve[current_curve_dx].Get_Nearest_Distance(coordinate, &current_t);
				
				// 重新计算当前路程
				current_curve_len = curve[current_curve_dx].Get_Current_Len(current_t);
				
				break;
			}
			else
			{
				// 路径结束
				is_end = true;
			}
		}
		
		if (is_start == false) // 还没开始，直接锁定起点
		{
			// 切向误差为0
			*tangent_error = 0;
			*tangent_vector = vector2d::Vector2D(0, 0);
			
			// 法向误差指向起点
			*normal_vector = curve[current_curve_dx].Get_Start_Point() - coordinate;
			*normal_error = (*normal_vector).length();
			
			// 单位化法向量
			*normal_vector = (*normal_vector).normalize();
			
			// 切向yaw角为曲线起始切向
			*tangent_yaw_vector = curve[current_curve_dx].Get_Tangent_Vector(0);
		}
		else if (current_t >= 1 - 1e-6f)// 结束，直接锁定终点
		{
			// 切向误差为0
			*tangent_error = 0;
			*tangent_vector = vector2d::Vector2D(0, 0);
			
			// 法向量指向终点
			*normal_vector = curve[current_curve_dx].Get_End_Point() - coordinate;
			*normal_error = (*normal_vector).length();
			
			// 单位化法向量
			*normal_vector = (*normal_vector).normalize();

			// 切向yaw角为曲线结束切向
			*tangent_yaw_vector = curve[current_curve_dx].Get_Tangent_Vector(1);
		}
		else// 在曲线上
		{
			// 计算当前路程
			current_curve_len = curve[current_curve_dx].Get_Current_Len(current_t);
			
			// 计算到终点的误差
			*tangent_error = total_len - current_curve_len - current_finished_len;
			*tangent_vector = curve[current_curve_dx].Get_Tangent_Vector(current_t);
			
			// 计算到曲线的单位法向误差
			*normal_vector = curve[current_curve_dx].Get_Normal_Vector(coordinate, current_t);
			
			// 切向yaw角为当前切向量
			*tangent_yaw_vector = *tangent_vector;
		}
		
		// 获取当前最大速度（根据减速度，曲率计算）
		*max_vel = curve[current_curve_dx].Get_Max_Vel(current_t);
		
		/*--------------------------------------------------------------------------------------------------------------------------------*/
		
		// 判断曲线是否过半
		if (current_t >= 0.5f)
		{
			curve_more_than_half = true;
		}
		
		// 获取当前前一个点，最新到达的点
		if (is_start == false)
		{
			// 还没开始
			if (curve[current_curve_dx].Get_Bezier_Order() == curve::FIRST_ORDER_BEZIER)
			{
				// 前一个点为结束点
				*current_point_num = curve[current_curve_dx].Get_End_Point_Num();
			}
			else
			{
				// 前一个点为控制点
				*current_point_num = curve[current_curve_dx].Get_Control_Point_Num();
			}
			
			// 到达的点为路径起点
			*arrive_point_num = start_point_num;
		}
		else
		{
			if (curve[current_curve_dx].Get_Bezier_Order() == curve::FIRST_ORDER_BEZIER)
			{
				// 前一个点为结束点
				*current_point_num = curve[current_curve_dx].Get_End_Point_Num();
				
				// 判断是不是第一条曲线
				if (current_curve_dx >= 1)
				{
					// 不是第一条，到达的点为上一条曲线终点
					*arrive_point_num = curve[current_curve_dx - 1].Get_End_Point_Num();
				}
				else
				{
					// 第一条，到达的点为路径起点
					*arrive_point_num = start_point_num;
				}
			}
			else
			{
				// 判断二阶贝塞尔曲线是否过半
				if (curve_more_than_half == false)
				{
					// 没过半，前一个点为控制点
					*current_point_num = curve[current_curve_dx].Get_Control_Point_Num();
					
					// 判断是不是第一条曲线
					if (current_curve_dx >= 1)
					{
						// 不是第一条，到达的点为上一条曲线终点
						*arrive_point_num = curve[current_curve_dx - 1].Get_End_Point_Num();
					}
					else
					{
						// 第一条，到达的点为路径起点
						*arrive_point_num = start_point_num;
					}
				}
				else // 过半
				{
					// 前一个点为结束点
					*current_point_num = curve[current_curve_dx].Get_End_Point_Num();
					
					// 到达的点为控制点
					*arrive_point_num = curve[current_curve_dx].Get_Control_Point_Num();
				}
			}
		}

		return true;
	}
	
	/*-------------------------------------------------------------------------------------------*/
	
	PathPlan2::PathPlan2(
		data::RobotPose& robot_pose_, chassis::Chassis& robot_chassis_,
		float max_linear_vel_, float linear_accel_, float linear_decel_,
		float max_angular_vel_, float angular_accel_, float angular_decel_,
		float distance_deadzone_, float yaw_deadzone_
	)
	: ManagedTask("PathPlan2Task", 30, 512, task::TASK_DELAY, 1), robot_pose(&robot_pose_), robot_chassis(&robot_chassis_)
	{
		max_linear_vel    = fabsf(max_linear_vel_);
		max_linear_decel  = fabsf(linear_decel_);
		max_linear_accel  = fabsf(linear_accel_);
		max_angular_vel   = fabsf(max_angular_vel_);
		max_angular_accel = fabsf(angular_accel_);
		max_angular_decel = fabsf(angular_decel_);

		total_path_num = 0;
		current_path_num = 0;

		point_head = 0;
		point_tail = 1;

		total_point_num = 1;// 从dx = 1开始，dx = 0为全局起始点
		generate_point_num = 0;
		
		// 是否使能
		is_enable = false;
		
		is_first_point = true;
		
		last_vw = 0;
		
		last_time = 0;
		
		last_tangent_v = 0;
		last_normal_v = 0;
		
		tangent_pid.Init(1.2, max_linear_decel, 0.1, max_linear_vel, 0);
		normal_pid.Init(1.2, max_linear_decel, 0.5, max_linear_vel, distance_deadzone_);
		yaw_pid.Init(1.6, max_angular_decel, 0.1, max_angular_vel, yaw_deadzone_);
		
		last_current_point_num = 0;// 上一次当前前一个点
		last_arrive_point_num = 0;// 上一次最新到达的点
		
		point[0].coordinate            = vector2d::Vector2D();// 雷达还没起启动，先不初始化坐标
		point[0].target_yaw            = 0;	// 到达前目标yaw
		point[0].leave_target_yaw      = 0;	// 离开前目标yaw 
		point[0].linear_vel            = max_linear_vel;
		point[0].linear_accel          = max_linear_accel;
		point[0].linear_decel          = max_linear_decel;                                                                                   
		point[0].angular_vel           = max_angular_vel ;
		point[0].angular_accel         = max_angular_accel;
		point[0].angular_decel         = max_angular_decel;
		point[0].use_tangent_yaw       = false;
		point[0].is_end                = false;
		point[0].is_stop               = false;
		point[0].have_event            = false;
		point[0].have_target_yaw       = false;		// 是否有到达前目标yaw
		point[0].have_leave_target_yaw = false;		// 是否有离开前目标yaw
		point[0].point_num 		      = 0;// 全局起点无任何功能
	}
	
	void PathPlan2::Enable()
	{
		is_enable = true;
	}
	
	void PathPlan2::Disable()
	{
		is_enable = false;
	}

	bool PathPlan2::Add_PathEvent(PathEvent2 * path_event_)
	{
		if (path_event_->id >= 1 && path_event_->id <= MAX_PATH_EVENT_NUM)
		{
			path_event_list[path_event_->id - 1] = path_event_;
			return true;
		}
		else
		{
			return false;
		}
	}
	
	bool PathPlan2::Next_Path()
	{
		if (path[current_path_num % 2].Is_End() && path[(current_path_num + 1) % 2].Is_Init())
		{
			current_path_num++;
			Delete_Point(path[(current_path_num - 1) % 2].start_point_num + path[(current_path_num - 1) % 2].point_num - 1);// 删除结束点以前的点（不包括结束点）
			path[(current_path_num - 1) % 2].Reset();
			return true;// 切换成功
		}
		else
		{
			return false;// 切换失败
		}
	}
	
	void PathPlan2::Task_Process()
	{
		/*----------------------------------生成路径-----------------------------------------*/
		if (path[total_path_num % 2].Is_Init() == false)
		{
			if (is_first_point)
			{
				if (robot_pose->Is_Position_Valid())
				{
					// 起点为机器人当前位置
					if (!isnan(*robot_pose->Get_pX()) && !isnan(*robot_pose->Get_pY()))
					{
						point[0].coordinate = vector2d::Vector2D(*robot_pose->Get_pX(), *robot_pose->Get_pY());

						is_first_point = false;
					}
				}
			}
			else
			{
				while(generate_point_num <= total_point_num)
				{
					if (path[total_path_num % 2].Generate_Path(point[generate_point_num % MAX_PATHPOINT_NUM]))
					{
						generate_point_num++;
					}
					else
					{
						total_path_num++;
						
						// 第一条路径的起始点已经确定
						if (generate_point_num > 0)
						{
							// 前一条路径的终点作为新路径的起点
							generate_point_num--;
						}
						
						break;//生成已经结束
					}
				}
			}
		}
		
		/*---------------------------------------------------------------------------*/
		
		float normal_error = 0;// 法向误差
		float tangent_error = 0;// 切向误差
		
		vector2d::Vector2D normal_vector = vector2d::Vector2D();// 误差法向量
		vector2d::Vector2D tangent_vector = vector2d::Vector2D();// 误差切向量
		vector2d::Vector2D tangent_yaw_vector = vector2d::Vector2D();// 目标yaw角切向量（不会突变）
		
		float max_vel = 0;// 最大速度
		uint16_t current_point_num = 0;// 当前前一个点
		uint16_t arrive_point_num = 0;// 最新到达的点
		
		// 规划
		if (!path[current_path_num % 2].Calculate(
			robot_pose,
			&normal_error, 
			&tangent_error, 
			&normal_vector, 
			&tangent_vector,
			&max_vel,
			&current_point_num,
			&arrive_point_num,
			&tangent_yaw_vector
		))
		{
			return;
		}
		
		/*---------------------------------------------------------------------------*/
		
		// 0代表无任何功能，遇到直接跳过
		if (current_point_num == 0)
		{
			current_point_num = last_current_point_num;
		}
		
		// 0代表无任何功能，遇到直接跳过
		if (arrive_point_num == 0)
		{
			arrive_point_num = last_arrive_point_num;
		}
		
		last_current_point_num = current_point_num;
		last_arrive_point_num = arrive_point_num;
		
		// 最新到达的点索引
		uint16_t arrive_point_dx = arrive_point_num % MAX_PATHPOINT_NUM;
		
		// 当前前一个点索引
		uint16_t current_point_dx = current_point_num % MAX_PATHPOINT_NUM;
		
		// 判断是否到达终点
		if (path[current_path_num % 2].Is_End())
		{
			// 判断终点是否有事件发生（终点与普通点的事件判断逻辑不一样）
			if (current_point_dx != 0 && point[current_point_dx].have_event && point[current_point_dx].is_stop)
			{
				// 事件索引
				uint8_t event_dx = point[current_point_dx].event_id - 1;
				
				// 有事件
				if (path_event_list[event_dx] != nullptr)
				{
					// 防止重复触发
					if (path_event_list[event_dx]->current_point_num != current_point_num)
					{
						path_event_list[event_dx]->is_continue = false;
						path_event_list[event_dx]->is_start = true;// 事件开始
						path_event_list[event_dx]->current_point_num = current_point_num;// 更新事件点
					}
					
					// 事件是否结束
					if (path_event_list[event_dx]->is_continue)
					{
						Next_Path();
						path_event_list[event_dx]->is_continue = false;
					}
				}
				else
				{
					Next_Path();
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
			// 判断最新到达的点是否有时间发生（普通点）
			if (arrive_point_dx != 0 && point[arrive_point_dx].have_event)
			{
				// 事件索引
				uint8_t event_dx = point[arrive_point_dx].event_id - 1;
				
				// 有事件
				if (path_event_list[event_dx] != nullptr)
				{
					// 防止重复触发
					if (path_event_list[event_dx]->current_point_num != arrive_point_dx)
					{
						path_event_list[event_dx]->is_continue = false;
						path_event_list[event_dx]->is_start = true;// 事件开始
						path_event_list[event_dx]->current_point_num = arrive_point_dx;// 更新
					}
				}
			}
		}

		/*-------------------------------------------------------------------------------------------*/
		
		float current_linear_vel    = point[current_point_dx].linear_vel;
		float current_linear_accel  = point[current_point_dx].linear_accel;
		float current_linear_decel  = point[current_point_dx].linear_decel;
											
		float current_angular_vel   = point[current_point_dx].angular_vel;
		float current_angular_accel = point[current_point_dx].angular_accel;
		float current_angular_decel = point[current_point_dx].angular_decel;
											
		bool have_target_yaw		= point[current_point_dx].have_target_yaw;
		bool use_tangent_yaw        = point[current_point_dx].use_tangent_yaw;					
		bool have_leave_target_yaw  = point[current_point_dx].have_leave_target_yaw;
 
		float target_yaw = 0;
		
		// 获取当前目标yaw
		if ((have_leave_target_yaw == true) && (path[current_path_num % 2].Is_Start() == false))
		{
			target_yaw = path[current_path_num % 2].start_target_angle;// 起始目标角度
		}
		else if (have_target_yaw) // 是否有目标yaw
		{
			if (use_tangent_yaw == true)
			{
				target_yaw = tangent_yaw_vector.angle();// 切向角度
			}
			else
			{
				target_yaw = point[current_point_dx].target_yaw;// 目标角度
			}
		}
		else
		{
			// 不锁角
			current_angular_vel   = 0;
			current_angular_accel = 0;
		    current_angular_decel = 0;
		}

		/*----------------------------------------------------------------------------------------*/
		
		// 计算时间差
		float dt  = (float)timer::Timer::Get_DeltaTime(last_time) / 1000000.f;// us->s
		last_time = timer::Timer::Get_TimeStamp();

		// 计算法向速度
		float normal_v  = normal_pid.NPid_Calculate(0, -normal_error);
		
		// 计算切向速度
		float tangent_v = tangent_pid.NPid_Calculate(0, -tangent_error);

		// 计算角速度
		float vw        = yaw_pid.NPid_Calculate(target_yaw, *robot_pose->Get_pYaw(), true, PI);

		// 增量
		float delta = 0;

		// 限制线加速度
		delta = chassis::Limit_Accel(tangent_v - last_tangent_v, current_linear_accel, dt);
		
		if (delta > 0)
		{
			tangent_v = last_tangent_v + delta;// 只限制加速，不限制减速度
		}

		//限制角加速度
		delta = chassis::Limit_Accel(vw - last_vw, current_angular_accel, dt);
		
		if (delta > 0)
		{
			vw = last_vw + delta;// 只限制加速，不限制减速度
		}
		
		if (!is_enable)
		{
			tangent_v = 0;
			normal_v = 0;
			vw = 0;
		}
		
		// 更新
		last_tangent_v = tangent_v;
		last_normal_v  = normal_v;
		last_vw        = vw;

		// 限制速度
		tangent_v = tangent_v > max_vel             ? max_vel             : tangent_v;
		tangent_v = tangent_v > current_linear_vel  ? current_linear_vel  : tangent_v;
		normal_v  = normal_v  > max_linear_vel      ? max_linear_vel      : normal_v;
		vw        = vw        > current_angular_vel ? current_angular_vel : vw;
		
		// 向量化
		normal_vector  = normal_vector  * normal_v;
		tangent_vector = tangent_vector * tangent_v;
		
		// 计算速度向量
		vector2d::Vector2D v = normal_vector + tangent_vector;
		
		// 底盘控制
		if (is_enable)
		{
			robot_chassis->Set_World_Vel(v, vw, *robot_pose->Get_pYaw());
		}
	}
	
	// 删除head_point_num之前的点
	bool PathPlan2::Delete_Point(uint16_t head_point_num)
	{
		uint16_t head_point_dx = head_point_num % MAX_PATHPOINT_NUM;
    
		// 环形缓冲区有效范围判断：point_head ~ point_tail（左闭右开）
		bool is_point_valid = false;
		
		if (point_head <= point_tail)
		{
			// 缓冲区未绕圈：有效范围 [point_head, point_tail)
			is_point_valid = (head_point_dx >= point_head && head_point_dx < point_tail);
		}
		else
		{
			// 缓冲区绕圈：有效范围 [point_head, MAX_PATHPOINT_NUM) ∪ [0, point_tail)
			is_point_valid = (head_point_dx >= point_head || head_point_dx < point_tail);
		}
		
		if (!is_point_valid)
		{
			return false; // 点不在有效范围内，删除失败
		}
		else
		{
			point_head = head_point_dx;
			return true;
		}
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
		// 计算空闲空间
		uint16_t used_space = (point_tail - point_head + MAX_PATHPOINT_NUM) % MAX_PATHPOINT_NUM;
		uint16_t free_space = MAX_PATHPOINT_NUM - used_space - 1; // 预留1个位置避免满溢
				
		if (free_space <= 0)
		{
			return false;
		}
		else
		{
			target_yaw_       = fmaxf(fminf(target_yaw_, PI), -PI);
			leave_target_yaw_ = fmaxf(fminf(leave_target_yaw_, PI), -PI);

			point[point_tail].coordinate            = coordinate_;			// 坐标
			point[point_tail].smoothness            = smoothness_;			// 平滑度
			point[point_tail].target_yaw            = target_yaw_;			// 到达前目标yaw
			point[point_tail].leave_target_yaw      = leave_target_yaw_;	// 离开前目标yaw 
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
			point[point_tail].have_target_yaw       = have_target_yaw_;				// 是否有到达前目标yaw
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
			coordinate_,															// 坐标  
			0,																		// 平滑度						
			target_yaw_,				                						    // 到达前目标yaw			
			leave_target_yaw_,			                  							// 离开前目标yaw			
			(linear_vel_    == PATH_MAX_PARAM ? max_linear_vel    : linear_vel_),  	// 到达前最大线速度			
			(linear_accel_  == PATH_MAX_PARAM ? max_linear_accel  : linear_accel_),	// 到达前最大线加速度		
			(linear_decel_  == PATH_MAX_PARAM ? max_linear_decel  : linear_decel_),	// 到达前最大线减速度		
			(angular_vel_   == PATH_MAX_PARAM ? max_angular_vel   : angular_vel_),	// 到达前最大角速度			
			(angular_accel_ == PATH_MAX_PARAM ? max_angular_accel : angular_accel_),// 到达前最大角加速度		
			(angular_decel_ == PATH_MAX_PARAM ? max_angular_decel : angular_decel_),// 到达前最大角减速度
			(target_yaw_ == PATH_TANGENT_YAW),              						// yaw是否朝切线方向		
			true,									        						// 是否为结束点		
			is_stop_,                                       						// 是否停止			
			(event_id_ != PATH_NO_EVENT),											// 是否包含事件				
			(target_yaw_ != PATH_NO_TARGET_YAW),	       							// 是否有到达前目标yaw		
			(leave_target_yaw_ != PATH_NO_TARGET_YAW),								// 是否有离开前目标yaw
			event_id_                                       						// 事件id
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
			coordinate_,															// 坐标  
			smoothness_,			                							    // 平滑度					
			target_yaw_,			                 								// 到达前目标yaw			
			0,		                   				    							// 离开前目标yaw			
			(linear_vel_    == PATH_MAX_PARAM ? max_linear_vel    : linear_vel_),  	// 到达前最大线速度			
			(linear_accel_  == PATH_MAX_PARAM ? max_linear_accel  : linear_accel_),	// 到达前最大线加速度		
			(linear_decel_  == PATH_MAX_PARAM ? max_linear_decel  : linear_decel_),	// 到达前最大线减速度		
			(angular_vel_   == PATH_MAX_PARAM ? max_angular_vel   : angular_vel_),	// 到达前最大角速度			
			(angular_accel_ == PATH_MAX_PARAM ? max_angular_accel : angular_accel_),// 到达前最大角加速度		
			(angular_decel_ == PATH_MAX_PARAM ? max_angular_decel : angular_decel_),// 到达前最大角减速度
			(target_yaw_ == PATH_TANGENT_YAW),          							// yaw是否朝切线方向		
			false,		                                							// 是否为结束点		
			false,                                      							// 是否停止			
			(event_id_ != PATH_NO_EVENT),			    							// 是否包含事件				
			(target_yaw_ != PATH_NO_TARGET_YAW),	   								// 是否有到达前目标yaw		
			false,				                        							// 是否有离开前目标yaw
			event_id_                                   							// 事件id
		);
	}
}