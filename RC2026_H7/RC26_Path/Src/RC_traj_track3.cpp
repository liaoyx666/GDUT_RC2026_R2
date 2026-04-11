#include "RC_traj_track3.h"
#include "RC_path3.h"
#include "RC_chassis.h"

namespace path
{
	TrajTrack3::TrajTrack3(data::RobotPose& pose_, chassis::Chassis& chassis_, float lon_deadzone_, float head_deadzone_)
	: task::ManagedTask("Track3Task", 30, 512, task::TASK_PERIOD, 1), chassis(chassis_)
	{
		path = nullptr;
		pose = &pose_;
		
		tan_pid.Init(4, 3, 0.03, 2.5, lon_deadzone_);
		nor_pid.Init(4, 3, 0.07, 2, 0);
		head_pid.Init(7, 5, 0.02, 3, head_deadzone_);
		
		ld_kf = 0.3f;
		
		ld_min = 0.03;
	
		is_enable = false;
		
		Reset();
	}
	
	void TrajTrack3::Task_Process()
	{
		vector2d::Vector2D v = vector2d::Vector2D();
		float w = 0;
		
		/*轨迹跟踪*/
		if (!Calc_Vel(&v, &w))
		{
			v = vector2d::Vector2D();
			w = 0;
		}

		if (is_enable)
		{
			/*设置底盘*/
			chassis.Set_World_Vel(v, w, *pose->Get_pYaw());
		}
	}
	
	
	void TrajTrack3::Reset()
	{
		last_wa = 0;
		last_a = 0;
		last_v = 0;
		last_w = 0;
		
		last_tan_v = 0;
		last_head_v = 0;
		
		last_time = timer::Timer::Get_TimeStamp();
		
		is_start = false;
		is_end = false;
	}

	bool TrajTrack3::Load_Path(Path3* path_)
	{
		if (path_ == nullptr) return false;
		if (!path_->Is_Init()) return false;
		
		Reset();
		
		path = path_;
		return true;
	}
	
	bool TrajTrack3::Calc_Vel(vector2d::Vector2D* v_, float* vw_) const
	{
		if (path == nullptr) return false;
		if (!path->Is_Init()) return false;
		if (pose == nullptr) return false;
		
		if (is_enable)
		{
			float t; /*参数*/
			float l; /*到终点的距离*/
			float d; /*到最近点的距离*/
			vector2d::Vector2D tan; /*单位切向量*/
			vector2d::Vector2D nor; /*单位法向量*/
			float v; /*规划最大速度*/
			LonConstr3 lon; /*纵向约束*/
			HeadConstr3 head; /*航向约束*/
			float cur; /*带符号曲率*/
			vector2d::Vector2D np;
			
			vector2d::Vector2D p = vector2d::Vector2D(*pose->Get_pX(), *pose->Get_pY()); /*机器人坐标*/
			
			path->Get_Near_Point_T_Len_Dis_Tan_Nor_Vel_Cur_Lon_Head(
				p,
				&np,
				&t,
				&l,
				&d,
				&tan,
				&nor,
				&v,
				&cur,
				&lon,
				&head
			);
			
			path->Trig_Event_On_Len(l); /*触发事件*/
			
			/*计算时间差*/
			float dt  = (float)timer::Timer::Get_DeltaTime(last_time) / 1000000.f; /*us->s*/
			last_time = timer::Timer::Get_TimeStamp();
			if (dt < 0.0001f)
			{
				dt = 0.0001f;
			}
			else if (dt > 0.1f)
			{
				dt = 0.1f;
			}
			
			if (vw_ != nullptr)
			{
				if (last_wa != head.wa)
				{
					head_pid.Set_Accel(head.wa); /*更新加速度*/
					last_wa = head.wa;
				}
				
				if (last_w != head.w)
				{
					head_pid.Set_Max_Out(head.w); /*更新速度*/
					last_w = head.w;
				}
				
				float head_v;
				float target_head;
				
				if (head.tan_head)
				{
					target_head = tan.angle();
				}
				else
				{
					target_head = head.yaw;
				}
				
				head_v = head_pid.NPid_Calculate(target_head, *pose->Get_pYaw(), true, PI);
				
				float delta = chassis::Limit_Accel(head_v - last_head_v, head.wa, dt);
				if (delta > 0) head_v = last_head_v + delta; /*只限制加速，不限制减速，减速靠pid*/
				last_head_v = head_v; /*更新*/
			
				*vw_ = fminf(head_v, head.w);
			}
			
			if (v_ != nullptr)
			{
				if (last_a != lon.a)
				{
					tan_pid.Set_Accel(lon.a); /*更新加速度*/
					last_a = lon.a;
				}
				
				if (last_v != lon.v)
				{
					tan_pid.Set_Max_Out(lon.v); /*更新速度*/
					last_v = lon.v;
				}

				if (!is_start)
				{
					if (path->Pre_Align())
					{
						float delta_yaw = *pose->Get_pYaw() - head.yaw;
						
						if (delta_yaw < -PI)
						{
							delta_yaw += TWO_PI;
						}
						else if (delta_yaw > PI)
						{
							delta_yaw -= TWO_PI;
						}
						
						if (fabsf(delta_yaw) <= TRAJTRACK3_PRE_ALIGN_THRESHOLD) /*yaw对齐后才能出发*/
						{
							is_start = true;
						}
						else
						{
							is_start = false;
						}
					}
					else
					{
						is_start = true;
					}
				}
				
				if (!is_start) /*还没出发*/
				{
					vector2d::Vector2D s; /*起点*/
					path->Get_Point_On_T(0, &s);
					
					vector2d::Vector2D dir = s - p; /*直接锁定起点*/
					l = dir.length();
					tan = dir.normalize();
					d = 0;
				}
				else if (t == 1.f) /*在终点 或 已经超过终点*/
				{
					vector2d::Vector2D e; /*终点*/
					path->Get_Point_On_T(1, &e);
					
					vector2d::Vector2D dir = e - p; /*直接锁定终点*/
					l = dir.length();
					tan = dir.normalize();
					d = 0;
				}
				else
				{
					l = path->Len() - l;
				}
				
				float tan_v = tan_pid.NPid_Calculate(0, -l);
				tan_v = fminf(tan_v, lon.v);
				
				if (t != 1.f)
				{
					tan_v = fminf(tan_v, v);
				}
				
				float delta = chassis::Limit_Accel(tan_v - last_tan_v, lon.a, dt);
				if (delta > 0) tan_v = last_tan_v + delta; /*只限制加速，不限制减速，减速靠pid*/
				last_tan_v = tan_v; /*更新*/
				
				float nor_v = nor_pid.NPid_Calculate(0, -d);
				nor_v = fminf(nor_v, lon.v);
				
				float ld = fmaxf(ld_min, tan_v * tan_v / (lon.a * 2.f)); /*前视点距离*/
				vector2d::Vector2D ld_p; /*前视点*/
				path->Get_Point_On_Len(l + ld, &ld_p); /*获取前视点*/
				
				if (fabsf(path->Len() - l) >= ld_min && is_start && t != 1.f)
				{
					float ld_ag = vector2d::Vector2D::angleBetween(tan, ld_p - np); /*前视点偏角*/
					tan.rotate(ld_ag * ld_kf);
				}
				
				if (path->wait_event)
				{
					for (uint8_t i = 0; i < EVENT3_MAX_EVENT_NUM; i++)
					{
						if (path->wait_event & (1 << i))
						{
							if (Event3::list[i] != nullptr)
							{
								if (Event3::list[i]->Is_Finish())
								{
									path->wait_event = path->wait_event & ~(1 << i);
								}
							}
						}
					}
				}
				
				if (l <= TRAJTRACK3_END_THRESHOLD && !path->wait_event) /*路程达到阈值 且 无需等待的事件*/
				{
					is_end = true; /*已结束*/
				}

				*v_ = tan * tan_v + nor * nor_v; /*速度合成*/
			}
		}
		else
		{
			if (v_ != nullptr)
			{
				*v_ = vector2d::Vector2D();
			}
			
			if (vw_ != nullptr)
			{
				*vw_ = 0;
			}
			
			last_tan_v = 0;
			last_head_v = 0;
			
			last_time = timer::Timer::Get_TimeStamp();
		}
		
		return true;
	}
}