#include "RC_path.h"

namespace path
{
	PathPoint::PathPoint() {}
	
	PathPoint::PathPoint(
		vector2d::Vector2D point_, 
		bool stop_on_arrival_, // 到达时是否停止
		float smoothness_, // 转弯平滑程度0~0.5
		float arrive_angle_, // 到达时角度（到达停止时限制）
		bool have_leave_angle_, // 是否限制离开时的角度
		float leave_angle_ // 离开时角度
	)
	{
		point = point_;
		stop_on_arrival = stop_on_arrival_;
		
		if (stop_on_arrival == true)// 到达时停止
		{
			smoothness_ = 0.f;// 停止走直线最快
			
			if (arrive_angle_ > PI) arrive_angle = PI;
			else if (arrive_angle_ <= -PI) arrive_angle = PI;
			else arrive_angle = arrive_angle_;
			
			have_leave_angle = have_leave_angle_;
			
			if (leave_angle_ > PI) leave_angle = PI;
			else if (leave_angle_ <= -PI) leave_angle = PI;
			else leave_angle = leave_angle_;

		}
		else// 到达时不停止（不限制到达和离开时角度，角度由前后点决定）
		{
			have_leave_angle = false;
		}
		
		is_init = true;
	}
	
	
	
	
	void PathPoint::Path_Point_Update(
		vector2d::Vector2D point_, 
		bool stop_on_arrival_, 
		float smoothness_, // 0~1
		float arrive_angle_, 
		bool have_leave_angle_, 
		float leave_angle_
	)
	{
		point = point_;
		stop_on_arrival = stop_on_arrival_;
		
		if (stop_on_arrival == true)// 到达时停止
		{
			smoothness_ = 0.f;// 停止走直线最快
			
			if (arrive_angle_ > PI) arrive_angle = PI;
			else if (arrive_angle_ <= -PI) arrive_angle = PI;
			else arrive_angle = arrive_angle_;
			
			have_leave_angle = have_leave_angle_;
			
			if (leave_angle_ > PI) leave_angle = PI;
			else if (leave_angle_ <= -PI) leave_angle = PI;
			else leave_angle = leave_angle_;

		}
		else// 到达时不停止（不限制到达和离开时角度，角度由前后点决定）
		{
			have_leave_angle = false;
		}
		
		is_init = true;
	}
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

	bool Path::Add_Point(
		vector2d::Vector2D point_, 
		float smoothness_// 0~0.5
	)
	{
		return Generate_Curve(point_, smoothness_);
	}
	
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

	bool Path::Add_End_Point(
		vector2d::Vector2D point_, 
		float end_angle_
	)
	{
		end_angle = end_angle_;
		
		return Generate_Curve(point_, 0);
	}
	
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
					vector2d::Vector2D temp_point = vector2d::Vector2D::lerp(point_, point_list[0], last_smoothness);// 曲线结束点，衔接直线过渡点
					
					bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], point_list[1], temp_point);// 曲线（二阶贝塞尔）
					total_len += bezier_curve_list[bezier_curve_num].Get_len();
					bezier_curve_num++;
					
					bezier_curve_list[bezier_curve_num].Bezier_Update(temp_point, point_list[1], point_);// 直线
					total_len += bezier_curve_list[bezier_curve_num].Get_len();
					bezier_curve_num++;
					
					generate_status = GENERATE_FINISHED_STRAIGHT;
				}
				else
				{
					if (last_smoothness == 0.5 && smoothness_ == 0.5)// 曲线间无需直线过渡
					{
						vector2d::Vector2D temp_point = vector2d::Vector2D::lerp(point_, point_list[0], last_smoothness);// 曲线结束点（下一段曲线起始点）
					
						bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], point_list[1], temp_point);// 曲线
						total_len += bezier_curve_list[bezier_curve_num].Get_len();
						bezier_curve_num++;
						
						point_list[0] = temp_point;// 下一段曲线起始点
						point_list[1] = point_;// 下一段曲线控制点
						
						generate_status = GENERATE_WAIT_LAST_CURVE_POINT;
					}
					else// 曲线间需要直线过渡
					{
						vector2d::Vector2D temp_point = vector2d::Vector2D::lerp(point_, point_list[0], last_smoothness);// 曲线衔接直线的过渡点
						
						bezier_curve_list[bezier_curve_num].Bezier_Update(point_list[0], point_list[1], temp_point);// 曲线
						total_len += bezier_curve_list[bezier_curve_num].Get_len();
						bezier_curve_num++;
						
						vector2d::Vector2D temp_point_1 = vector2d::Vector2D::lerp(point_list[0], point_, smoothness_);// 直线衔接下一段曲线的过渡点
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
	
	
	
	
	
	
	
	
	
	
	
	
	/*--------------------------------------------------------------------------*/
	
	
	PathPlan::PathPlan(float max_speed_, float max_accel_, float max_decel_)
	{
		
	}
	
	
	bool PathPlan::Add_Path_Point(
		vector2d::Vector2D point_, 
		bool stop_on_arrival_, 
		float smoothness_,
		float arrive_angle_, 
		bool have_leave_angle_, 
		float leave_angle_
	)
	{
		if (Get_Point_Space() < 1)
		{
			return false;
		}
		else
		{
			path_point_list[path_point_tail].Path_Point_Update(
				point_, 
				stop_on_arrival_, 
				smoothness_,
				arrive_angle_, 
				have_leave_angle_, 
				leave_angle_
			);
			
			if (path_point_tail == MAX_PATH_POINT_NUM - 1)
			{
				path_point_tail = 0;
			}
			else
			{
				path_point_tail++;
			}

			return true;
		}
	}
	
	
	void PathPlan::Delete_Path_Point()
	{
		if (path_point_head == MAX_PATH_POINT_NUM - 1)
		{
			path_point_head = 0;
		}
		else
		{
			path_point_head++;
		}
	}
	
	
	// 获取剩余路径点存放空间
	uint8_t PathPlan::Get_Point_Space()
	{
		if (path_point_head <= path_point_tail)
		{
			return MAX_PATH_POINT_NUM - (path_point_tail - path_point_head) - 1;// - 1防止首尾相遇
		}
		else
		{
			return path_point_head - path_point_tail - 1;// - 1防止首尾相遇
		}
	}
}