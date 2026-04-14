#include "RC_curve.h"

namespace curve
{
	Curve2D::Curve2D()
	{
		is_init = false;
		len = 0;
		cur = 0;
		end_vel = 0;
	}
	
	
	float Curve2D::Vel_On_Len(float l_, float a) const
	{
		float l = len - l_;
		
		float v = calcVel(l < 0.f ? 0.f : l, end_vel, a);
		
		float c = fabsf(cur); /*曲率绝对值*/
		
		if (c > 1e-6f)
		{
			v = fminf(sqrtf(a / c), v);
		}
		
		return v; /*输出当前最小速度*/
	}

	/*-------------------------------------------------------------------------------------------------------*/
	
	Line2D::Line2D()
	{
		Reset();
	}
	
	void Line2D::Reset()
	{
		start = vector2d::Vector2D();
		dir = vector2d::Vector2D();
		tan = vector2d::Vector2D();
		nor = vector2d::Vector2D();
	
		len_sq = 0;
		
		len = 0;
		cur = 0;
		is_init = 0;
	}
	
	
	bool Line2D::Init(vector2d::Vector2D start_, vector2d::Vector2D end_)
	{
		if (start_ == end_)
		{
			Reset();
			return false;
		}
		else
		{
			start = start_;
			dir = end_ - start_;
			len = dir.length();/*长度*/
			len_sq = len * len;
			
			tan = dir.normalize();/*单位切向量*/
			nor = tan.perpendicular();/*单位法向量，切向量左侧*/
			
			cur = 0;
			is_init = true;
			return true;
		}
	}
	
	
	
	void Line2D::Get_Point_On_T(float t, vector2d::Vector2D* p) const
	{
		if (!is_init) return;
		
		if (p == nullptr) return;
		
		if (t <= 0.f)
		{
			*p = start;
		}
		else if (t >= 1.f)
		{
			*p = start + dir;
		}
		else
		{
			*p = start + dir * t;
		}
	}
	
	void Line2D::Get_Near_Point_T_Dis(vector2d::Vector2D p, vector2d::Vector2D* near_p, float* near_t, float* near_d) const
	{
		if (!is_init) return;
		
		vector2d::Vector2D sp = p - start;
		
		float t = sp.dot(dir) / len_sq;
	
		if (t < 0.f)
		{
			t = 0.f;
		}
		else if (t > 1.f)
		{
			t = 1.f;
		}
		
		if (near_t != nullptr)
		{	
			*near_t = t;
		}
		
		vector2d::Vector2D np;
		
		Get_Point_On_T(t, &np);
		
		if (near_p != nullptr)
		{
			*near_p = np;
		}
		
		if (near_d != nullptr)
		{
			*near_d = (p - np).length() * (sp.cross(dir) > 0.f ? 1.f : -1.f);/*左正右负*/
		}
	}
	
	void Line2D::Get_Tan_Nor_On_T(float t, vector2d::Vector2D* tan_, vector2d::Vector2D* nor_) const
	{
		if (!is_init) return;
		
		if (tan_ != nullptr)
		{
			*tan_ = tan;
		}
		
		if (nor_ != nullptr)
		{
			*nor_ = nor;
		}
	}
	
	bool Line2D::Get_Point_On_Len(float len_, vector2d::Vector2D* p) const
	{
		if (!is_init) return false;
		
		if (p == nullptr)
		{
			return false;
		}
		
		if (len_ < 0.f)
		{
			Get_Point_On_T(0.f, p);
			
			return false;
		}
		
		if (len_ > len)
		{
			Get_Point_On_T(1.f, p);
			
			return false;
		}
		
		Get_Point_On_T(len_ / len, p);
		
		return true;
	}
	
	float Line2D::Get_Len_On_T(float t) const
	{
		if (!is_init) return 0;
		
		if (t < 0.f)
		{
			t = 0.f;
		}
		else if (t > 1.f)
		{
			t = 1.f;
		}
		
		return len * t;
	}
	
	/*-------------------------------------------------------------------------------------------------------*/
	
	Arc2D::Arc2D()
	{
		Reset();
	}
	
	void Arc2D::Reset()
	{
		start_ag = 0;
		end_ag = 0;
		center = vector2d::Vector2D();
		radius = 0;
		delta_ag = 0;
		
		len = 0;
		cur = 0;
		is_init = 0;
	}
	
	bool Arc2D::Init(vector2d::Vector2D start_, vector2d::Vector2D center_, float ag_)
	{
		if (vector2d::Vector2D::isZero(ag_) || fabsf(ag_) >= TWO_PI)
		{
			Reset();
			return false;
		}

		/*计算从圆心指向起点的向量*/
		vector2d::Vector2D r = start_ - center_;
		radius = r.length();

		/*半径不能为0*/
		if (vector2d::Vector2D::isZero(radius))
		{
			is_init = false;
			return false;
		}

		center = center_;/*圆心*/
		start_ag = r.angle();/*起点角度*/
		delta_ag = ag_;/*角度差*/
		end_ag = start_ag + ag_;/*终点角度*/
		len = radius * fabsf(ag_);/*弧长*/
		
		cur = 1.f / radius * (delta_ag > 0.f ? 1.f : -1.f);
		is_init = true;
		return true;
	}
	
	
	
	
	
	/*只能生成劣弧*/
	bool Arc2D::Init(vector2d::Vector2D start_, vector2d::Vector2D end_, float radius_, bool is_counter_clockwise)
	{
		if (vector2d::Vector2D::isZero(radius_))
		{
			Reset();
			return false;
		}
		radius = radius_;

		float d = vector2d::Vector2D::distance(start_, end_);
		if (d > 2.f * radius || vector2d::Vector2D::isZero(d))
		{
			is_init = false;
			return false;
		}

		vector2d::Vector2D mid = (start_ + end_) * 0.5f;
		vector2d::Vector2D perp = (end_ - start_).perpendicular();
		perp = perp.normalize();

		float h = sqrtf(radius * radius - (d * d) * 0.25f);

		/*顺时针 / 逆时针*/
		if (!is_counter_clockwise)
		{
			perp = perp * -1.f;
		}

		center = mid + perp * h;

		start_ag = (start_ - center).angle();/*-PI ~ PI*/
		end_ag = (end_ - center).angle();/*-PI ~ PI*/

		/*防止生成优弧*/
		float delta = end_ag - start_ag;
		if (delta < -PI)
		{
			end_ag += TWO_PI;
		}
		else if (delta > PI)
		{
			end_ag -= TWO_PI;
		}
		
		delta_ag = end_ag - start_ag;
		len = radius * fabsf(delta_ag);

		cur = 1.f / radius * (delta_ag > 0.f ? 1.f : -1.f);
		is_init = true;
		return true;
	}
	
	void Arc2D::Get_Point_On_T(float t, vector2d::Vector2D* p) const
	{
		if (!is_init) return;
		
		if (p == nullptr) return;
		
		float ag;

		if (t <= 0.f)
		{
			ag = start_ag;
		}
		else if (t >= 1.f)
		{
			ag = end_ag;
		}
		else
		{
			ag = start_ag + delta_ag * t;
		}
		
		*p = center + vector2d::Vector2D(ag) * radius;
	}
	
	void Arc2D::Get_Near_Point_T_Dis(vector2d::Vector2D p, vector2d::Vector2D* near_p, float* near_t, float* near_d) const
	{
		if (!is_init) return;
		
		if (p == center)/*点与圆心重合*/
		{
			if (near_t != nullptr)
			{
				*near_t = 0.5f;
			}
			
			if (near_p != nullptr)
			{
				Get_Point_On_T(0.5f, near_p);/*取中间点*/
			}
			
			if (near_d != nullptr)
			{
				*near_d = radius * (delta_ag > 0.f ? 1.f : -1.f);/*左正右负*/
			}
			
			return;
		}
		
		vector2d::Vector2D cp = p - center;
		
		float ag = cp.angle();
		
		/*防止优弧*/
		float delta = ag - start_ag;/*-PI ~ PI*/
		if (delta < -PI)
		{
			ag += TWO_PI;
		}
		else if (delta > PI)
		{
			ag -= TWO_PI;
		}
		
		float t = (ag - start_ag) / delta_ag;

		if (t < 0.f)
		{
			t = 0.f;
		}
		else if (t > 1.f)
		{
			t = 1.f;
		}
		
		if (near_t != nullptr)
		{	
			*near_t = t;
		}
		
		vector2d::Vector2D np;
		
		Get_Point_On_T(t, &np);
		
		if (near_p != nullptr)
		{
			*near_p = np;
		}
		
		if (near_d != nullptr)
		{
			vector2d::Vector2D npp = p - np;
			
			vector2d::Vector2D tan;
			
			Get_Tan_Nor_On_T(t, &tan, NULL);

			*near_d = npp.length() * (npp.cross(tan) > 0.f ? 1.f : -1.f);/*左正右负*/
		}
	}
	
	void Arc2D::Get_Tan_Nor_On_T(float t, vector2d::Vector2D* tan_, vector2d::Vector2D* nor_) const
	{
		if (!is_init) return;
		
		vector2d::Vector2D p;
		
		Get_Point_On_T(t, &p);
		
		vector2d::Vector2D nor = -(p - center).normalize() * (delta_ag > 0.f ? 1.f : -1.f);
		
		if (nor_ != nullptr)
		{
			*nor_ = nor;
		}
		
		if (tan_ != nullptr)
		{
			*tan_ = -nor.perpendicular();
		}
	}
	
	bool Arc2D::Get_Point_On_Len(float len_, vector2d::Vector2D* p) const
	{
		if (!is_init) return false;
		
		if (p == nullptr)
		{
			return false;
		}
		
		if (len_ < 0.f)
		{
			Get_Point_On_T(0.f, p);
			
			return false;
		}
		
		if (len_ > len)
		{
			Get_Point_On_T(1.f, p);
			
			return false;
		}
		
		Get_Point_On_T(len_ / len, p);
		
		return true;
	}
	
	float Arc2D::Get_Len_On_T(float t) const
	{
		if (!is_init) return 0;
		
		if (t < 0.f)
		{
			t = 0.f;
		}
		else if (t > 1.f)
		{
			t = 1.f;
		}
		
		return len * t;
	}
	
	
	
	
}