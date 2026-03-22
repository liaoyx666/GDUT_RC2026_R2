#include "RC_traj_plan3.h"

namespace path
{
	Point3::Point3()
	{
		point = vector2d::Vector2D();
		blend_dis = 0;
		param = 0;
		event = 0;
		lon = LonConstr3();
		head = HeadConstr3();
	}
	
	
	TrajPlan3::TrajPlan3(LonConstr3 l, HeadConstr3 h)
	{
		lon_m = LonConstr3(); /*约束最大值*/
		head_m = HeadConstr3(); /*约束最大值*/
		path = nullptr;
		
		state = TRAJPLAN_NULL;

		dir = vector2d::Vector2D();
	
		blend_dis = 0;
		
		tmp_s_p = vector2d::Vector2D();
		tmp_c = Point3();
	}
	 
	void TrajPlan3::Reset_Path()
	{
		if (path != nullptr)
		{
			path->Reset();
		}
		
		state = TRAJPLAN_NULL;
		
		tmp_s_p = vector2d::Vector2D();
		tmp_c = Point3();
		
		dir = vector2d::Vector2D();
		
		blend_dis = 0;
	}
	
	bool TrajPlan3::Load_Path(Path3* path_)
	{
		if (path_ == nullptr) return false;
		
		path = path_;
		Reset_Path();

		return true;
	}
	
	float TrajPlan3::Calc_CornerVel(float cor_ag)
	{
		if (fabsf(cor_ag) < 175.f / 180.f * PI)
		{
			return 0;
		}
		else
		{
			return lon_m.v; /*折角足够大，看成一条直线*/
		}
	}
	
	bool TrajPlan3::Add_Point(Point3 p)
	{
		if (path == nullptr || path->is_init) return false;

		uint8_t line_fs = path->Line_FreeSpace();
		uint8_t arc_fs = path->Arc_FreeSpace();
		
		switch(state)
		{
			/*==========================================================================================================================================*/
			case TRAJPLAN_NULL:
			{
				/*第一个点*/
				tmp_s_p = p.point; /*路径起始点*/
				
				/*第一个点无约束*/
				
				dir = vector2d::Vector2D(); /*第一个点还没有方向向量*/
				
				state = TRAJPLAN_HAVE_START;
				return true;
				break;
			}
			/*==========================================================================================================================================*/
			case TRAJPLAN_HAVE_START:
			{
				if (line_fs <= 1)
				{
					p.Set_Is_End(); /*强制结束*/
				}
				else if (line_fs == 2)
				{
					p.blend_dis = 0; /*不能再添加圆弧*/
				}
				
				if (arc_fs == 0)
				{
					p.blend_dis = 0; /*不能再添加圆弧*/
				}
				
				vector2d::Vector2D se = p.point - tmp_s_p; /*start -> end*/
				
				if (p.Is_End() || vector2d::Vector2D::isZero(p.blend_dis)) /*当前点为结束点 或 当前点不使用圆弧过渡*/
				{
					if (dir != vector2d::Vector2D()) /*dir不为0向量，说明已经生成dir*/
					{
						float cor_ag = vector2d::Vector2D::angleBetween(-dir, se); /*计算拐角角度*/
						
						if (path->curve[path->Curve_Num() - 1] != nullptr)
						{
							path->curve[path->Curve_Num() - 1]->Update_End_Vel(Calc_CornerVel(cor_ag)); /*计算上一曲线结束点的速度*/
						}
					}
					
					dir = se; /*更新方向向量*/
					
					path->Add_Line(tmp_s_p, p.point);/*新增直线*/ /*!!!LINE++*/////////////
					
					Add_Constr(p, path->len); /*添加当前点约束条件*/
					
					if (p.Is_End()) /*当前点是结束点*/
					{
						if (path->curve[path->Curve_Num() - 1] != nullptr)
						{
							path->curve[path->Curve_Num() - 1]->Update_End_Vel(0.f);/*设置曲线终点速度，路径终点为0*/
						}
						
						Add_MaxConstr(path->len);  /*添加最大约束条件，防止整条路径无约束*/
						
						if (p.Have_Event() && p.Is_Wait())
						{
							/*结束时要等待事件完成再切换下一段*/
							path->is_wait = true;
						}
						
						path->is_init = true; /*路径生成完成*/
					}
					
					state = TRAJPLAN_HAVE_START;
					return true; /*添加成功*/
				}
				else /*---------------------------------------------------------------------------------------------------------------------------------*/
				{
					/*当前点希望圆弧过渡*/
					dir = se; /*更新方向向量*/
					
					blend_dis = p.blend_dis; /*希望的圆角距离*/
					
					tmp_c = p; /*希望平滑处理的拐点*/
					
					state = TRAJPLAN_HAVE_START_CORNER;
					return true; /*添加成功*/
				}
				break;
			}
			/*==========================================================================================================================================*/
			case TRAJPLAN_HAVE_START_CORNER:
			{
				if (line_fs <= 2)
				{
					p.Set_Is_End(); /*强制结束*/
				}
				else if (line_fs == 3)
				{
					p.blend_dis = 0; /*不能再添加圆弧*/
				}
				
				if (arc_fs <= 1)
				{
					p.blend_dis = 0; /*不能再添加圆弧*/
				}
				
				vector2d::Vector2D cs = tmp_s_p - tmp_c.point; /*corner -> start*/
				vector2d::Vector2D ce = p.point - tmp_c.point; /*corner -> end*/
				
				float cs_l = cs.length(); /*边长*/
				float ce_l = ce.length(); /*边长*/
				
				blend_dis = fminf(tmp_c.blend_dis, fminf(ce_l, cs_l)); /*圆角距离必须小于两边的边长*/
				
				float cor_ag = vector2d::Vector2D::angleBetween(cs, ce); /*折角角度*/
				
				float r = calcFilletRadius(cor_ag, blend_dis); /*计算圆弧半径*/
				
				if (r <= TRAJPLAN3_MAX_RADIUS && r >= TRAJPLAN3_MIN_RADIUS)
				{
					/*圆弧半径在范围内*/
					vector2d::Vector2D s = vector2d::Vector2D::lerp(tmp_c.point, tmp_s_p, blend_dis / cs_l); /*计算圆弧起点*/
					
					if (!vector2d::Vector2D::isZero(blend_dis - cs_l)) /*圆弧起点没有和起始点重合*/
					{
						path->Add_Line(tmp_s_p, s); /*使用直线过渡*/ /*!!!LINE++*/////////////
					}
					
					vector2d::Vector2D e = vector2d::Vector2D::lerp(tmp_c.point, p.point, blend_dis / ce_l); /*计算圆弧终点*/
					
					path->Add_Arc(s, e, r, (cor_ag < 0 ? true : false)); /*添加圆弧*/ /*!!!ARC++*/////////////
					
					Add_Constr(tmp_c, path->len); /*取圆弧终点加入拐角点的约束*/

					dir = p.point - tmp_c.point; /*更新方向向量*/
					
					if (p.Is_End() || vector2d::Vector2D::isZero(p.blend_dis) || vector2d::Vector2D::isZero(blend_dis - ce_l))  
					{
						/*当前点为结束点 或 当前点不使用圆弧过渡 或 圆弧终点与当前点重合*/
						
						if (!vector2d::Vector2D::isZero(blend_dis - ce_l))
						{
							path->Add_Line(e, p.point); /*直线过渡*/ /*!!!LINE++*/////////////
						}
						
						tmp_s_p = p.point;
						
						Add_Constr(p, path->len); /*加入结束点约束*/
						
						if (p.Is_End())
						{
							if (path->curve[path->Curve_Num() - 1] != nullptr)
							{
								path->curve[path->Curve_Num() - 1]->Update_End_Vel(0.f);/*设置曲线终点速度，路径终点为0*/
							}
							
							Add_MaxConstr(path->len);  /*添加最大约束条件，防止整条路径无约束*/
						
							if (p.Have_Event() && p.Is_Wait())
							{
								path->is_wait = true;
							}
							
							path->is_init = true; /*路径生成完成*/
						}
						
						state = TRAJPLAN_HAVE_START;
						return true;
					}
					else /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
					{
						/*当前点也使用圆弧过渡*/
						tmp_s_p = e;
						
						tmp_c = p; /*希望平滑处理的拐点*/
						
						blend_dis = p.blend_dis;
						
						state = TRAJPLAN_HAVE_START_CORNER;
						return true;
					}
				}
				else /*---------------------------------------------------------------------------------------------------------------------------------*/
				{
					/*圆弧半径超出范围，采用折线过渡*/
					path->Add_Line(tmp_s_p, tmp_c.point); /*!!!LINE++*/////////////
					
					Add_Constr(tmp_c, path->len); /*加入拐点约束*/
			
					if (p.Is_End() || vector2d::Vector2D::isZero(p.blend_dis))
					{
						/*当前点为结束点 或 当前点不使用圆弧过渡*/
						path->Add_Line(tmp_c.point, p.point); /*!!!LINE++*/////////////
						
						if (path->curve[path->Curve_Num() - 1] != nullptr)
						{
							path->curve[path->Curve_Num() - 1]->Update_End_Vel(Calc_CornerVel(cor_ag)); /*计算上一曲线结束点的速度*/
						}
						
						Add_Constr(p, path->len); /*加入约束*/
						
						tmp_s_p = p.point;
			
						if (p.Is_End())
						{
							if (path->curve[path->Curve_Num() - 1] != nullptr)
							{
								path->curve[path->Curve_Num() - 1]->Update_End_Vel(0.f);/*设置曲线终点速度，路径终点为0*/
							}
							
							Add_MaxConstr(path->len);  /*添加最大约束条件，防止整条路径无约束*/
							
							if (p.Have_Event() && p.Is_Wait())
							{
								path->is_wait = true;
							}
							
							path->is_init = true; /*路径生成完成*/
						}
						
						state = TRAJPLAN_HAVE_START;
						return true;
					}
					else /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
					{
						dir = tmp_c.point - tmp_s_p; /*更新方向向量*/
						
						tmp_s_p = tmp_c.point;
						
						tmp_c = p; /*希望平滑处理的拐点*/
						
						blend_dis = p.blend_dis;
						
						state = TRAJPLAN_HAVE_START_CORNER;
						return true;
					}
				}
				break;
			}
			/*==========================================================================================================================================*/
			default:
			{
				state = TRAJPLAN_NULL;
				return false;
				break;
			}
		}
	}
	
	bool TrajPlan3::Add_Constr(Point3 p, float len)
	{
		if (path == nullptr) return false;
		if (path->Is_Init()) return false;
		
		if (p.Have_LonCon())
		{
			/*有约束*/
			path->Add_PathLonCon(len, LonConstr3::min(p.lon, lon_m));/*加入速度约束*/
		}
		
		if (p.Have_HeadCon())
		{
			/*有约束*/
			path->Add_PathHeadCon(len, HeadConstr3::min(p.head, head_m));/*加入航向约束*/
		}
		
		if (p.Have_Event())
		{
			path->Add_PathEvent(len, p.event); /*加入事件*/
		}
		
		return true;
	}
	
	bool TrajPlan3::Add_MaxConstr(float len)
	{
		if (path == nullptr) return false;
		if (path->Is_Init()) return false;
		
		path->Add_PathLonCon(len, lon_m);/*加入速度约束*/
		
		path->Add_PathHeadCon(len, head_m);/*加入航向约束*/
		
		return true;
	}
	
	void TrajPlan3::Calc_End_Vel()
	{
		if (path == nullptr || !path->Is_Init()) return;
		
		float l = path->Len(); /*路径总长度*/
		
		float dis = 0;
		
		float end_v = 0;
		
		float cur; /*曲率*/
		
		LonConstr3 c;
		
		path->Get_Constr_On_Len(l, &c, NULL);
		
		for (uint8_t i = path->Curve_Num() - 1; i >= 0; i--)
		{
			if (path->curve[i] != nullptr)
			{
				if (i != path->Curve_Num() - 1) /*最后一条曲线的结束点速度为0*/
				{
					end_v = calcVel(dis, end_v, c.a, c.j);
					
					if (cur > 1e-6f)
					{
						path->curve[i]->Update_End_Vel(sqrtf(c.a / cur));
					}
				}
				
				path->curve[i]->Update_End_Vel(end_v);
				
				path->Get_Constr_On_Len(l, &c, NULL);
				
				dis = path->curve[i]->Len(); /*这段曲线的长度*/
				
				cur = path->curve[path->Curve_Num() - 1]->Cur(); /*曲率*/
				
				l -= dis;
			}
		}
	}
	
	float calcFilletRadius(float theta, float blend_dis)
	{
		float half_angle = fabsf(theta) / 2.f;
		float radius = blend_dis * tanf(half_angle);
		return radius;
	}
	
	float calcVel(float dis, float end_v, float a, float j)
	{
		// 距离极小，只能以终点速度行驶
		if (dis < 1e-6f) return end_v;

		// 1. 加速度约束（梯形减速）
		float v_acc = sqrtf(end_v * end_v + 2.f * a * dis);

		// 2. 加加速度约束（S型减速）
		float v_jerk = end_v;
		float a3_6j2 = (a * a * a) / (6.f * j * j);
		if (dis > a3_6j2)
		{
			float v_rel = sqrtf(2.f * a * (dis - a3_6j2));
			v_jerk = end_v + v_rel;
		}

		// 速度上限取两者最小值
		float v_max = (v_acc < v_jerk) ? v_acc : v_jerk;
		// 下限保护：不能小于终点速度
		if (v_max < end_v) v_max = end_v;
		return v_max;
	}
}