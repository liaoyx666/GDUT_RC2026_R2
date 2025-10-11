#include "RC_bezier_curve.h"

namespace curve
{
	BezierCurve::BezierCurve()
	{
		
	}
	
	// 一阶贝塞尔直线初始化
	BezierCurve::BezierCurve(vector2d::Vector2D start_point_, vector2d::Vector2D end_point_)
	{
		Bezier_Update(start_point_, end_point_);
	}
	
	
	// 二阶贝塞尔曲线初始化
	BezierCurve::BezierCurve(vector2d::Vector2D start_point_, vector2d::Vector2D control_point_, vector2d::Vector2D end_point_)
	{
		Bezier_Update(start_point_, control_point_, end_point_);
	}
	
	
	// 重置一阶贝塞尔曲线
	void BezierCurve::Bezier_Update(vector2d::Vector2D start_point_, vector2d::Vector2D end_point_)
	{
		order = FIRST_ORDER_BEZIER;
		
		start_point = start_point_;
		end_point = end_point_;
		
		len = 0;
		
		tangent_vector = (end_point - start_point).normalize();
		normal_vector = tangent_vector.perpendicular();
		
		len = vector2d::Vector2D::distance(start_point, end_point);
	}
	
	// 重置二阶贝塞尔曲线
	void BezierCurve::Bezier_Update(vector2d::Vector2D start_point_, vector2d::Vector2D control_point_, vector2d::Vector2D end_point_)
	{
		order = SECOND_ORDER_BEZIER;
		
		start_point = start_point_;
		control_point = control_point_;
		end_point = end_point_;
		
		len = 0;
		
		float temp_t = 0;
		float temp_len = 0;
		
		for (uint8_t i = 0; i < DISTANCE_SAMPLE_NUM; i++)
		{
			if (i < DISTANCE_SAMPLE_NUM - 1)
			{
				temp_len = vector2d::Vector2D::distance(Get_Point(temp_t), Get_Point(temp_t + distance_sample_step));
				
				temp_t += distance_sample_step;
			}
			else
			{
				temp_len = vector2d::Vector2D::distance(Get_Point(temp_t), end_point);
			}
			
			len += temp_len;
			
			// 提前储存一些点走过的长度
			distance_list[i] = len;
		}
	}
	
	
	
	// 获取点
	vector2d::Vector2D BezierCurve::Get_Point(const float t)
	{
		if (t <= 0.f) return start_point;
		else if (t >= 1.f) return end_point;
		
		if (order == FIRST_ORDER_BEZIER)
		{
			return vector2d::Vector2D::lerp(start_point, end_point, t);
		}
		else
		{
			return vector2d::Vector2D::lerp(
				vector2d::Vector2D::lerp(start_point, control_point, t), 
				vector2d::Vector2D::lerp(control_point, end_point, t), 
				t
			); 
		}
	}
	
	// 求最近点距离并输出对应t值
	float BezierCurve::Get_Nearest_Distance(const vector2d::Vector2D point, float* t)
	{
		if (order == FIRST_ORDER_BEZIER)
		{
			// 计算向量
			const vector2d::Vector2D d = end_point - start_point;
			const vector2d::Vector2D v = point - start_point;
			
			// 计算点积
			const float dot_vd = v.dot(d);
			const float dot_dd = d.dot(d);
			
			// 处理线段长度为0的特殊情况
			if (dot_dd < 1e-9f)
			{
				if (t != nullptr)
				{
					*t = 0.f;
				}
				return v.length();
			}
			
			// 计算t值
			const float t_val = dot_vd / dot_dd;
			
			if (t_val <= 0.0f)
			{
				if (t != nullptr)
				{
					*t = 0.0f;
				}
				
				return v.length();
			}
			else if (t_val >= 1.0f)
			{
				if (t != nullptr)
				{
					*t = 1.0f;
				}
				
				return vector2d::Vector2D::distance(end_point, point);
			}
			
			// 设置输出参数t
			if (t != nullptr)
			{
				*t = t_val;
			}
			
			const vector2d::Vector2D offset = v - d * t_val;
			
			// 返回距离
			return offset.length();
		}
		else
		{			
			// 黄金分割法查找最近点
			float left = 0.f;
			float right = 1.f;
			
			// 初始化黄金分割点
			float t1 = 1.f - GOLDEN_RATIO;
			float t2 = GOLDEN_RATIO;
			
			// 计算初始点的距离平方
			float d1 = vector2d::Vector2D::distanceSquared(Get_Point(t1), point);
			float d2 = vector2d::Vector2D::distanceSquared(Get_Point(t2), point);
			
			// 迭代收缩区间
			for (uint8_t i = 0; i < FIND_NEAREST_DISTANCE_STEP_COUNT; i++)
			{
				if (d1 < d2)
				{
					// 左侧区间更优，收缩右边界
					right = t2;
					t2 = t1;
					d2 = d1;
					t1 = right - GOLDEN_RATIO * (right - left);
					d1 = vector2d::Vector2D::distanceSquared(Get_Point(t1), point);
				}
				else
				{
					// 右侧区间更优，收缩左边界
					left = t1;
					t1 = t2;
					d1 = d2;
					t2 = left + GOLDEN_RATIO * (right - left);
					d2 = vector2d::Vector2D::distanceSquared(Get_Point(t2), point);
				}
			}
			
			// 找到最优值t1和d1（重复利用）
			if (d1 > d2)
			{
				d1 = d2;
				t1 = t2;
			}
			
			d2 = vector2d::Vector2D::distanceSquared(Get_Point(left), point);
			
			if (d2 < d1)
			{
				d1 = d2;
				t1 = left;
			}
			
			d2 = vector2d::Vector2D::distanceSquared(Get_Point(right), point);
			
			if (d2 < d1)
			{
				d1 = d2;
				t1 = right;
			}
			
			if (t != nullptr)
			{
				*t = t1;
			}
			
			// 返回实际距离（开方得到真实距离）
			return sqrtf(d1);
		}
	}
	
	
	// 获取当前位置走过的长度
	float BezierCurve::Get_Current_Len(float t)
	{
		if (t <= 0.f) return 0.f;
		else if (t >= 1.f) return len;
		
		if (order == FIRST_ORDER_BEZIER)
		{
			return (Get_Point(t) - start_point).length();
		}
		else
		{
			uint8_t distance_list_dx = 0;
			
			while (t >= distance_sample_step)
			{
				t -= distance_sample_step;
				distance_list_dx++;
			}
			
			t = t < 0 ? -t : t;
			
			float p = t / distance_sample_step;
			
			if (distance_list_dx == 0)
			{
				return distance_list[0] * p;
			}
			else if (distance_list_dx < DISTANCE_SAMPLE_NUM)
			{
				return distance_list[distance_list_dx - 1] * (1.f - p) + distance_list[distance_list_dx] * p;
			}
			else
			{
				return len;
			}
		}
	}
	
	// 获取单位切向量
	vector2d::Vector2D BezierCurve::Get_Tangent_Vector(const float t)
	{
		// 一阶直线的方向向量已经提前储存
		if (order != FIRST_ORDER_BEZIER)
		{
			tangent_vector = (Get_Point(t + 0.01f) - Get_Point(t - 0.01f)).normalize();
		}
		
		return tangent_vector;
	}
	
	
	// 获取单位法向量
	vector2d::Vector2D BezierCurve::Get_Normal_Vector(const vector2d::Vector2D& point, const float t)
	{
		if (order != FIRST_ORDER_BEZIER)
		{
			normal_vector = Get_Tangent_Vector(t).perpendicular();// 同时更新tangent_vector;
		}
		
		float cross_result;// 叉乘结果
		
		if (order == FIRST_ORDER_BEZIER)
		{
			cross_result = tangent_vector.cross(point - start_point);
		}
		else
		{
			cross_result = tangent_vector.cross(point - Get_Point(t));
		}
		
		if (cross_result > 0) 
		{
			return normal_vector;
		}
		else if (cross_result < 0)
		{
			return -normal_vector;
		}
		else
		{
			return vector2d::Vector2D(0, 0);
		}
	}
	
	
	
	
	
	
	
	
	
	
	
	float BezierCurve::Get_Curvature(float t)
	{
		// 处理t值边界情况
		if (t < 0.f) t = 0.f;
		else if (t > 1.f) t = 1.f;
		
		// 一阶贝塞尔曲线(直线)的曲率恒为0
		if (order == FIRST_ORDER_BEZIER)
		{
			return 0.0f;
		}
		// 二阶贝塞尔曲线曲率计算
		else
		{
			 // 获取控制点
			const vector2d::Vector2D p0 = start_point;
			const vector2d::Vector2D p1 = control_point;
			const vector2d::Vector2D p2 = end_point;
			
			// 修正：正确的导数系数
			// 一阶导数：B'(t) = 2(1-t)(P₁-P₀) + 2t(P₂-P₁) = 2(P₁-P₀) + 2t(P₂-2P₁+P₀)
			vector2d::Vector2D A = (p1 - p0) * 2.f;
			vector2d::Vector2D B = (p2 - p1 * 2.f + p0) * 2.f;  // 修正这里！
			
			// 计算t处的一阶导数(dx, dy)
			vector2d::Vector2D first_deriv = A + B * t;
			float dx = first_deriv.data()[0];
			float dy = first_deriv.data()[1];
			
			// 计算二阶导数(ddx, ddy) - 二阶导数是常数
			vector2d::Vector2D second_deriv = (p2 - p1 * 2.f + p0) * 2.f;  // B''(t) = 2(P₂-2P₁+P₀)
			float ddx = second_deriv.data()[0];
			float ddy = second_deriv.data()[1];
			
			// 曲率公式分子: |dx*ddy - dy*ddx|
			float numerator = fabsf(dx * ddy - dy * ddx);
			
			// 曲率公式分母: (dx² + dy²)^(3/2)
			float len_squared = dx * dx + dy * dy;
			
			// 避免除以零 (处理奇点情况)
			if (len_squared < 1e-12f)
			{
				return 1e12f;  // 返回一个大数表示曲率很大
			}
			
			// 修正：正确的分母计算
			float denominator = powf(len_squared, 1.5f);
			
			// 返回曲率值
			return numerator / denominator;
		}
	}
	
	
}