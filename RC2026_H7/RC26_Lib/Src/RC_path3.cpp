#include "RC_path3.h"

namespace path
{
	LonConstr3::LonConstr3()
	{
		v = 0;
		a = 0;
		j = 0;
	}
	
	LonConstr3 LonConstr3::min(const LonConstr3& c1, const LonConstr3& c2)
	{
		LonConstr3 c;
		c.v = ((c1.v < c2.v) ? c1.v : c2.v);
		c.a = ((c1.a < c2.a) ? c1.a : c2.a);
		c.j = ((c1.j < c2.j) ? c1.j : c2.j);
		return c;
	}
	
	HeadConstr3::HeadConstr3()
	{
		yaw = 0;
		w = 0;
		wa = 0;
		tan_head = false;
	}
	
	HeadConstr3 HeadConstr3::min(const HeadConstr3& c1, const HeadConstr3& c2)
	{
		HeadConstr3 c;
		c.w = ((c1.w < c2.w) ? c1.w : c2.w);
		c.wa = ((c1.wa < c2.wa) ? c1.wa : c2.wa);
		return c;
	}
	
	PathLonCon3::PathLonCon3()
	{
		len = 0;
		c = LonConstr3();
	}
	
	PathHeadCon3::PathHeadCon3()
	{
		len = 0;
		c = HeadConstr3();
	}
	
	PathEvent3::PathEvent3()
	{
		len = 0;
		event = 0;
	}
	
	

	Path3::Path3()
	{
		Reset();
	}
	
	void Path3::Reset()
	{
		line_num = 0;
		arc_num = 0;
		
		lon_num = 0;
		head_num = 0;
		event_num = 0;
		
		lon_dx = 0;
		head_dx = 0;
		
		is_wait = false;
		
		len = 0;
		memset(curve, 0, sizeof(curve));
		
		is_init = false;
	}
	
	bool Path3::Add_Line(vector2d::Vector2D start_, vector2d::Vector2D end_)
	{
		if (line_num >= PATH3_MAX_LINE_NUM)
		{
			return false;
		}
		
		if (line[line_num].Init(start_, end_))
		{
			curve[line_num + arc_num] = &line[line_num];
			len += line[line_num].Len();
			line[line_num].Set_End_Vel(PATH3_MAX_LIN_VEL);
			line_num++;
			return true;
		}
		
		return false;
	}
	
	bool Path3::Add_Arc(vector2d::Vector2D start_, vector2d::Vector2D end_, float radius_, bool is_counter_clockwise)
	{
		if (arc_num >= PATH3_MAX_ARC_NUM)
		{
			return false;
		}
		
		if (arc[arc_num].Init(start_, end_, radius_, is_counter_clockwise))
		{
			curve[line_num + arc_num] = &arc[arc_num];
			len += arc[arc_num].Len();
			arc[arc_num].Set_End_Vel(PATH3_MAX_LIN_VEL);
			arc_num++;
			return true;
		}
		
		return false;
	}
	
	
	bool Path3::Get_Point_On_Len(float len_, vector2d::Vector2D* p) const
	{
		uint8_t curve_num = line_num + arc_num;
		
		if (p == nullptr || curve_num == 0 || !is_init)
		{
			return false;
		}
		
		if (len_ < 0.f)
		{
			if (curve[0] != nullptr)
			{
				curve[0]->Get_Point_On_T(0, p);/*输出起点*/
			}
			
			return false;
		}
		
		if (len_ > len)
		{
			if (curve[curve_num - 1] != nullptr)
			{
				curve[curve_num - 1]->Get_Point_On_T(1, p);/*输出终点*/
			}
			
			return false;
		}
		
		float l = 0;/*当前曲线段的起点路程*/

		for (uint8_t i = 0; i < curve_num - 1; i++)
		{
			if (curve[i] == nullptr)
			{
				return false;
			}
			
			if (len_ <= l + curve[i]->Len())
			{
				curve[i]->Get_Point_On_Len(len_ - l, p);
				return true;
			}
			
			l += curve[i]->Len();
		}
		
		if (curve[curve_num - 1] == nullptr)
		{
			return false;
		}
		
		curve[curve_num - 1]->Get_Point_On_Len(len_ - l, p);/*遍历到最后一段曲线*/
		return true;
	}
	
	void Path3::Get_Point_On_T(float t, vector2d::Vector2D* p) const
	{
		if (!is_init || p == nullptr) return;
		
		uint8_t curve_num = line_num + arc_num;
		
		if (t <= 0.f)
		{
			if (curve[0] != nullptr)
			{
				curve[0]->Get_Point_On_T(0, p);/*输出起点*/
				return;
			}
			else
			{
				return;
			}
		}
		else if (t >= 1.f)
		{
			if (curve[curve_num - 1] != nullptr)
			{
				curve[curve_num - 1]->Get_Point_On_T(1.f, p);/*输出终点*/
				return;
			}
			else
			{
				return;
			}
		}
		
		float len_ = len * t;
		
		float l = 0;/*当前曲线段的起点路程*/

		for (uint8_t i = 0; i < curve_num - 1; i++)
		{
			if (curve[i] == nullptr)
			{
				return;
			}
			
			if (l <= len_ && len_ <= l + curve[i]->Len())
			{
				curve[i]->Get_Point_On_Len(len_ - l, p);
				return;
			}
			
			l += curve[i]->Len();
		}
		
		if (curve[curve_num - 1] == nullptr)
		{
			return;
		}
		
		curve[curve_num - 1]->Get_Point_On_Len(len_ - l, p);/*遍历到最后一段曲线*/
		return;
	}
	
	void Path3::Get_Near_Point_T_Len_Dis_Tan_Nor(
		vector2d::Vector2D p_, 
		vector2d::Vector2D* near_p, 
		float* near_t, 
		float* near_l, 
		float* near_d, 
		vector2d::Vector2D* tan_, 
		vector2d::Vector2D* nor_
	) const
	{
		if (!is_init) return;
		
		uint8_t dx;
		
		float nd;
		
		vector2d::Vector2D np;
		
		float nt;
		
		uint8_t curve_num = line_num + arc_num;

		dx = 0;/*初始化dx*/
		
		if (curve[dx] == nullptr) return;

		curve[dx]->Get_Near_Point_T_Dis(p_, &np, &nt, &nd);/*初始化nd*/
		
		/*遍历所有曲线查找最近点*/
		for (uint8_t i = 1; i < curve_num; i++)
		{
			if (curve[i] == nullptr) return;

			float d;
			
			vector2d::Vector2D p;
			
			float t;
			
			curve[i]->Get_Near_Point_T_Dis(p_, &p, &t, &d);
			
			if (fabsf(d) < fabsf(nd))
			{
				/*更新*/
				dx = i;
				nd = d;
				np = p;
				nt = t;
			}
		}
		
		float nl = 0;
		
		for (uint8_t i = 0; i < dx; i++)
		{
			nl += curve[i]->Len();
		}
		
		nl += curve[dx]->Get_Len_On_T(nt);
		
		if (tan_ != nullptr || nor_ != nullptr)
		{
			curve[dx]->Get_Tan_Nor_On_T(nt, tan_, nor_);/*输出切向量，法向量*/
		}
		
		if (near_l != nullptr)
		{
			*near_l = nl;/*输出最近点路程*/
		}
		
		if (near_t != nullptr)
		{
			/*输出最近点t值*/
			if (dx == 0 && nt == 0.f)
			{
				*near_t = 0.f;
			}
			else if (dx == curve_num - 1 && nt == 1.f)
			{
				*near_t = 1.f;
			}
			else
			{
				*near_t = nl / len;
			}
		}
		
		if (near_d != nullptr)
		{
			*near_d = nd;/*输出最近点距离*/
		}
		
		if (near_p != nullptr)
		{
			*near_p = np;/*输出最近点*/
		}

	}
	
	bool Path3::Add_PathLonCon(float l, LonConstr3 c)
	{
		if (lon_num >= PATHLONCON3_MAX_NUM) return false;
		
		if (lon_num == 0)/*第一个*/
		{
			lon[lon_num].len = l;
			lon[lon_num].c = c;
			lon_num++;
			return true;
		}
		else
		{
			if (l <= lon[lon_num - 1].len || vector2d::Vector2D::isZero(l - lon[lon_num - 1].len))/*len必须从小到大排布*/
			{
				return false;
			}
			else
			{
				lon[lon_num].len = l;
				lon[lon_num].c = c;
				lon_num++;
				return true;
			}
		}
	}
	
	
	bool Path3::Add_PathHeadCon(float l, HeadConstr3 c)
	{
		if (head_num >= PATHHEADCON3_MAX_NUM) return false;
		
		if (head_num == 0)/*第一个*/
		{
			head[head_num].len = l;
			head[head_num].c = c;
			head_num++;
			return true;
		}
		else
		{
			if (l <= head[head_num - 1].len || vector2d::Vector2D::isZero(l - head[head_num - 1].len))/*len必须从小到大排布*/
			{
				return false;
			}
			else
			{
				head[head_num].len = l;
				head[head_num].c = c;
				head_num++;
				return true;
			}
		}
	}   
	    
	
	bool Path3::Add_PathEvent(float l, Event3_t e)
	{
		if (event_num >= PATHEVENT3_MAX_NUM) return false;
		
		if (event_num == 0)/*第一个*/
		{
			event[event_num].len = l;
			event[event_num].event = e;
			event_num++;
			return true;
		}
		else
		{
			if (l <= event[event_num - 1].len || vector2d::Vector2D::isZero(l - event[event_num - 1].len))/*len必须从小到大排布*/
			{
				return false;
			}
			else
			{
				event[event_num].len = l;
				event[event_num].event = e;
				event_num++;
				return true;
			}
		}
	}
	
	void Path3::Get_Constr_On_Len(float l_, LonConstr3* lon_, HeadConstr3* head_)
	{
		if (!is_init) return;
		
		if (lon_ != nullptr && lon_num != 0 && lon_num <= PATHLONCON3_MAX_NUM)
		{
			float left = (lon_dx == 0 ? 0.f : lon[lon_dx - 1].len);
			float right = lon[lon_dx].len;
			
			if (l_ < left)
			{
				/*向左查找，都不满足条件就第一个*/
				while (lon_dx > 0)
				{
					lon_dx--;
					
					float left = (lon_dx == 0 ? 0.f : lon[lon_dx - 1].len);
					float right = lon[lon_dx].len;
					
					if (l_ > left && l_ <= right) break;
				}
			}
			else if (l_ > right)
			{
				/*向右查找，都不满足条件就最后一个*/
				while (lon_dx < lon_num - 1)
				{
					lon_dx++;
					
					float left = (lon_dx == 0 ? 0.f : lon[lon_dx - 1].len);
					float right = lon[lon_dx].len;
					
					if (l_ > left && l_ <= right) break;
				}
			}
			
			*lon_ = lon[lon_dx].c;
		}
		
		if (head_ != nullptr && head_num != 0 && head_num <= PATHHEADCON3_MAX_NUM)
		{
			float left = (head_dx == 0 ? 0.f : head[head_dx - 1].len);
			float right = head[head_dx].len;
			
			if (l_ < left)
			{
				/*向左查找，都不满足条件就第一个*/
				while (head_dx > 0)
				{
					head_dx--;
					
					float left = (head_dx == 0 ? 0.f : head[head_dx - 1].len);
					float right = head[head_dx].len;
					
					if (l_ > left && l_ <= right) break;
				}
			}
			else if (l_ > right)
			{
				/*向右查找，都不满足条件就最后一个*/
				while (head_dx < head_num - 1)
				{
					head_dx++;
					
					float left = (head_dx == 0 ? 0.f : head[head_dx - 1].len);
					float right = head[head_dx].len;
					
					if (l_ > left && l_ <= right) break;
				}
			}
			
			*head_ = head[head_dx].c;
		}
	}
	    
}