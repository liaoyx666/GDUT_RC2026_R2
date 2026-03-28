#include "RC_traj_track3.h"
#include "RC_path3.h"
#include "RC_chassis.h"

namespace path
{
	TrajTrack3::TrajTrack3(data::RobotPose& pose_)
	{
		path = nullptr;
		pose = &pose_;
		last_wa = 0;
		last_a = 0;
		
		last_tan_v = 0;
		last_head_v = 0;
		
		last_time = 0;
		
		tan_pid.Init(2, 4, 0.05, 2.5);
		nor_pid.Init(2, 4, 0.05, 2.5);
		head_pid.Init(3, 6, 0.09, 3);

		ld_kf = 0.3f;
		
		ld_min = 0.03;
		ld_k = 0.9;
	}

	bool TrajTrack3::Load_Path(Path3* path_)
	{
		if (path_ == nullptr) return false;
		if (!path_->Is_Init()) return false;
		
		path = path_;
		
		return true;
	}
	
	bool TrajTrack3::Calc_Vel_On_P(vector2d::Vector2D* v_, float* vw_) const
	{
		if (path == nullptr) return false;
		if (!path->Is_Init()) return false;
		if (pose == nullptr) return false;
		
		float t; /*参数*/
		float l; /*到终点的距离*/
		float d; /*到最近点的距离*/
		vector2d::Vector2D tan; /*单位切向量*/
		vector2d::Vector2D nor; /*单位法向量*/
		float v; /*规划最大速度*/
		LonConstr3 lon; /*纵向约束*/
		HeadConstr3 head; /*航向约束*/
		//float cur; /*带符号曲率*/
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
			NULL,
			&lon,
			&head
		);
		
		/*计算时间差*/
		float dt  = (float)timer::Timer::Get_DeltaTime(last_time) / 1000000.f; /*us->s*/
		last_time = timer::Timer::Get_TimeStamp();
		if (dt < 0.0001f) dt = 0.001f;
		
		if (vw_ != nullptr)
		{
			if (last_wa != head.wa)
			{
				head_pid.Set_Accel(head.wa); /*更新加速度*/
				
				last_wa = head.wa;
			}
			
			float head_v;
			
			if (head.tan_head)
			{
				head_v = head_pid.NPid_Calculate(tan.angle(), *pose->Get_pYaw(), true, PI);
			}
			else
			{
				head_v = head_pid.NPid_Calculate(head.yaw, *pose->Get_pYaw(), true, PI);
			}
			
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
				nor_pid.Set_Accel(lon.a); /*更新加速度*/
				
				last_a = lon.a;
			}
			
//			if (t == 0.f) /*还在起点，或还没到达起点*/
//			{
//				vector2d::Vector2D s; /*起点*/
//				path->Get_Point_On_T(0.f, &s);
//				
//				vector2d::Vector2D dir = s - p; /*直接锁定起点*/
//				l = dir.length();
//				tan = dir.normalize();
//				d = 0;
//			}
			if (t == 1.f) /*还在终点，或已经超过终点*/
			{
				vector2d::Vector2D e; /*终点*/
				path->Get_Point_On_T(1.f, &e);
				
				vector2d::Vector2D dir = e - p; /*直接锁定终点*/
				l = dir.length();
				tan = dir.normalize();
				d = 0;
			}
		
			float tan_v = tan_pid.NPid_Calculate(0, -(path->Len() - l));
			tan_v = fminf(tan_v, lon.v);
			tan_v = fminf(tan_v, v);
			
			float delta = chassis::Limit_Accel(tan_v - last_tan_v, lon.a, dt);
			if (delta > 0) tan_v = last_tan_v + delta; /*只限制加速，不限制减速，减速靠pid*/
			last_tan_v = tan_v; /*更新*/
			
			float nor_v = nor_pid.NPid_Calculate(0, -d);
			nor_v = fminf(nor_v, lon.v);
			
			float ld = fmaxf(ld_min, tan_v * tan_v / (lon.a * 2.f)); /*前视点距离*/
			vector2d::Vector2D ld_p; /*前视点*/
			path->Get_Point_On_Len(l + ld, &ld_p); /*获取前视点*/
			
			if (fabsf(path->Len() - l) >= ld_min)
			{
				float ld_ag = vector2d::Vector2D::angleBetween(tan, ld_p - np); /*前视点偏角*/

				tan.rotate(ld_ag * ld_kf);
			}

			*v_ = tan * tan_v + nor * nor_v;
		}
		
		return true;
	}
	
}