#include "RC_path3.h"

namespace path
{
	LonConstr3::LonConstr3()
	{
		v = 0;
		a = 0;
	}
	
	LonConstr3 LonConstr3::min(const LonConstr3& c1, const LonConstr3& c2)
	{
		LonConstr3 c;
		c.v = ((c1.v < c2.v) ? c1.v : c2.v);
		c.a = ((c1.a < c2.a) ? c1.a : c2.a);
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
		ndx = 0;
		len_dx = 0;
		event_dx = 0;
		
		wait_event = 0x00;

		memset(curve, 0, sizeof(curve));
		
		pre_align = false;
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
			uint8_t num = line_num + arc_num;
			curve[num] = &line[line_num];
			if (num != 0)
			{
				len[num] = len[num - 1] + line[line_num].Len();
			}
			else
			{
				len[num] = line[line_num].Len();
			}

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
			uint8_t num = line_num + arc_num;
			curve[line_num + arc_num] = &arc[arc_num];
			if (num != 0)
			{
				len[num] = len[num - 1] + arc[arc_num].Len();
			}
			else
			{
				len[num] = arc[arc_num].Len();
			}
		
			arc[arc_num].Set_End_Vel(PATH3_MAX_LIN_VEL);
			arc_num++;
			return true;
		}
		
		return false;
	}
	
	
	bool Path3::Get_Point_On_Len(float len_, vector2d::Vector2D* p) const
	{
		const uint8_t curve_num = line_num + arc_num;
		
		if (p == nullptr || curve_num == 0 || !is_init)
		{
			return false;
		}
		
		if (len_ <= 0.f)
		{
			if (curve[0] != nullptr)
			{
				curve[0]->Get_Point_On_T(0, p);/*输出起点*/
				len_dx = 0;
				return true;
			}
			else
			{
				return false;
			}
		}
		
		if (len_ >= len[curve_num - 1])
		{
			if (curve[curve_num - 1] != nullptr)
			{
				curve[curve_num - 1]->Get_Point_On_T(1, p);/*输出终点*/
				len_dx = curve_num - 1;
				return true;
			}
			else
			{
				return false;
			}
		}
		
		if (len_dx != curve_num - 1 && len_ > len[len_dx])
		{
			do {
				len_dx++; /*向前移动索引*/
			} while (len_dx < curve_num - 1 && len_ > len[len_dx]);
		}
		else if (len_dx != 0 && len_ < len[len_dx - 1])
		{
			do {
				len_dx--; /*向后移动索引*/
			} while (len_dx > 0 && len_ < len[len_dx - 1]);
		}
	
		const float offset = (len_dx == 0) ? len_ : (len_ - len[len_dx - 1]);
		curve[len_dx]->Get_Point_On_Len(offset, p);
		
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
		
		float len_ = len[curve_num - 1] * t;
		
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
	
	
	bool Path3::Check_Near_Point(vector2d::Vector2D p_, uint8_t check_dx_, uint8_t* ndx_, vector2d::Vector2D* np, float* nt, float* nd) const
	{
		if (curve[check_dx_] == nullptr) return false; /*未更新*/
		
		float d;
		vector2d::Vector2D p;
		float t;
		
		curve[check_dx_]->Get_Near_Point_T_Dis(p_, &p, &t, &d);
		
		if (fabsf(d) < fabsf(*nd))
		{
			/*更新*/
			*ndx_ = check_dx_;
			*nd = d;
			*np = p;
			*nt = t;
			return true; /*更新*/
		}
		
		return false; /*未更新*/
	}
	
	
	void Path3::Get_Near_Point_T_Len_Dis_Tan_Nor_Vel_Cur_Lon_Head(
		vector2d::Vector2D p_,
		vector2d::Vector2D* near_p,
		float* near_t, 
		float* near_l, 
		float* near_d, 
		vector2d::Vector2D* tan_, 
		vector2d::Vector2D* nor_,
		float* v,
		float* cur_,
		LonConstr3* lon,
		HeadConstr3* head
	) const
	{
		/*只查找当前，前，后三条路径*/
		/*----------------------------------------------------------------------*/
		if (!is_init) return;

		float nd; /*最近点距离*/
		
		vector2d::Vector2D np; /*最近点*/
																				//<< 初始化
		float nt; /*最近点的曲线t参数值*/
		
		const uint8_t curve_num = line_num + arc_num;

		if (ndx >= curve_num) ndx = curve_num - 1; /*防越界*/
		
		if (curve[ndx] == nullptr) return;
		/*----------------------------------------------------------------------*/
		/*使用上一次ndx初始化np nt nd*/
		curve[ndx]->Get_Near_Point_T_Dis(p_, &np, &nt, &nd);
		
		/*查找前一段路径*/
		if (ndx > 0)
		{
			Check_Near_Point(p_, ndx - 1, &ndx, &np, &nt, &nd);
		}
		
		/*查找后一段路径*/
		if (ndx < curve_num - 1)
		{
			Check_Near_Point(p_, ndx + 1, &ndx, &np, &nt, &nd);
		}
																				//<< 查找最近点及其所在曲线索引
		/*临界情况*/
		if (ndx > 0 && nt <= 0.f) /*在曲线起点处*/
		{
			uint8_t i = ndx;
			do {
				i--; /*向前查找*/
				Check_Near_Point(p_, i, &ndx, &np, &nt, &nd);
			} while (i > 0 && nt <= 0.f);
		}
		else if (ndx < curve_num - 1 && nt >= 1.f) /*在曲线终点处*/
		{
			uint8_t i = ndx;
			do {
				i++; /*向后查找*/
				Check_Near_Point(p_, i, &ndx, &np, &nt, &nd);
			} while (i < curve_num - 1 && nt >= 1.f);
		}
		/*----------------------------------------------------------------------*/
		float l = curve[ndx]->Get_Len_On_T(nt); /*最近点在这条曲线上的路程*/
																				//< 计算路程
		float nl = ((ndx == 0) ? 0 : len[ndx - 1]) + l; /*总路程*/
		/*----------------------------------------------------------------------*/
		if (cur_ != nullptr)
		{
			*cur_ = curve[ndx]->Cur();											//<< 当前曲率
		}
		/*----------------------------------------------------------------------*/
		if (lon != nullptr || v != nullptr)
		{
			LonConstr3 c;
			
			Get_Constr_On_Len(nl, &c, NULL); /*获取当前纵向约束*/
																				//<< 纵向约束
			if (lon != nullptr)
			{
				*lon = c;
			}
		/*----------------------------------------------------------------------*/
			if (v != nullptr)
			{
				/*输出当前最小速度*/
				*v = fminf(curve[ndx]->Vel_On_Len(l, c.a), c.v);				//<< 最小速度
			}
		}
		/*----------------------------------------------------------------------*/
		if (head != nullptr)
		{																		//<< 航向约束
			Get_Constr_On_Len(nl, NULL, head); /*获取当前航向约束*/
		}
		/*----------------------------------------------------------------------*/
		if (tan_ != nullptr || nor_ != nullptr)
		{
			/*输出切向量，法向量*/
			curve[ndx]->Get_Tan_Nor_On_T(nt, tan_, nor_);						//<< 切向量，法向量
		}
		/*----------------------------------------------------------------------*/
		if (near_l != nullptr)
		{
			*near_l = nl;/*输出最近点路程*/										//<< 路程
		}
		/*----------------------------------------------------------------------*/
		if (near_t != nullptr)
		{
			/*输出最近点t值*/
			if (ndx == 0 && nt == 0.f)
			{
				*near_t = 0.f;
			}
			else if (ndx == curve_num - 1 && nt == 1.f)
			{																	//<< t参数
				*near_t = 1.f;
			}
			else
			{
				*near_t = nl / len[curve_num - 1];
			}
		}
		/*----------------------------------------------------------------------*/
		if (near_d != nullptr)
		{
			*near_d = nd;/*输出最近点距离*/										//<< 最近点距离
		}
		/*----------------------------------------------------------------------*/
		if (near_p != nullptr)
		{
			*near_p = np;/*输出最近点*/											//<< 最近点坐标
		}
		/*----------------------------------------------------------------------*/
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
	
	
	void Path3::Get_Constr_On_Len(float l_, LonConstr3* lon_, HeadConstr3* head_) const
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
	
	void Path3::Trig_Event_On_Len(float l_) const
	{
		if (!is_init || event_num == 0) return;
		
		while(event[event_dx].len - PATH3_TRIG_EVENT_THRESHOLD < l_ && event_dx < event_num - 1)
		{
			if (event[event_dx].event)
			{
				for (uint8_t i = 0; i <= EVENT3_MAX_EVENT_NUM; i++)
				{
					if ((1 << i) & event[event_dx].event)
					{
						if (Event3::list[i] != nullptr)
						{
							Event3::list[i]->Trig_Once(); /*触发一次*/
						}
					}
				}
			}
			
			event_dx++;
		}
	}
	
	void Path3::Add_Wait_Event_At_End(Event3_t e)
	{
		for (uint8_t i = 0; i < EVENT3_MAX_EVENT_NUM; i++)
		{
			if (((1 << i) & e) && Event3::list[i]->Wait_Finish()) /*保存所有需要结束时等待的事件*/
			{
				wait_event = wait_event | (1 << i);
			}
		}
	}
	    
}