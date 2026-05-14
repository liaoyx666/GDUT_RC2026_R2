#include "RC_best_path.h"

namespace ros
{
	BestPath::BestPath(cdc::CDC &cdc_, uint8_t rx_id_) : cdc::CDCHandler(cdc_, rx_id_)
	{
		
	}
	
	
	void BestPath::CDC_Receive_Process(uint8_t *buf, uint16_t len)
	{
		step_num = len;
		if (len <= 6 && is_init == false)
		{
			memcpy(step, buf, len);

			is_init = true;
		}
		
		// 应答
		uint8_t ack = 1;
		cdc->CDC_Send_Pkg(3, &ack, 1, 1000);
	}
	
	
	vector2d::Vector2D BestPath::Get_MF_Location(uint8_t n)
	{
		if (n >= 1 && n <= 12)
		{
			return MF_location[n - 1];
		}
		else
		{
			return vector2d::Vector2D();
		}
	}
	
	
	Dir BestPath::Dir_From_To(uint8_t from, uint8_t to)
	{
		if (from <= 12 && from >= 1 && to <= 12 && to >= 1)
		{
			int8_t d = to - from;
			switch (d)
			{
				case 1:
					return Dir::R;
					break;
				
				case -1:
					return Dir::L;
					break;
				
				case 3:
					return Dir::F;
					break;
				
				case -3:
					return Dir::B;
					break;
				
				default:
					return Dir::N;
					break;
			}
		}
		else
		{
			return Dir::N;
		}
	}
	
	
	float BestPath::Dir_To_Yaw(Dir d)
	{
		switch (d)
		{
			case Dir::F:
				return 0.f;
				break;
			
			case Dir::B:
				return PI;
				break;
			
			case Dir::L:
				return -PI / 2.f;
				break;
			
			case Dir::R:
				return PI / 2.f;
				break;
			
			default:
				return 0.f;
				break;
		}
	}
	
	
	void BestPath::MF_Best_Path_Plan(Map& MF_map, path::PathPlan& path_plan)
	{
		while(is_init == false || MF_map.Is_Init() == false)
		{
			osDelay(1);
		}
		
		path_plan.Add_Start_Point(vector2d::Vector2D(0, 0));// 启动点
		
		for (uint8_t i = 0; i < step_num + 1; i++)
		{
			if (i == 0)
			{
				if (MF_map.Get_MF(step[i]) == 2)
				{
					path_plan.Add_End_Point(vector2d::Vector2D(
						Get_MF_Location(step[i]).data()[0], Get_MF_Location(step[i]).data()[1] - MF_SIZE + CHASSIS_MOVE), 
						0.f,
						false
					);// 去拿取
					
					MF_map.Set_MF(step[i], 4);// 已拿取，变空格
				}
				else
				{
					path_plan.Add_Point(
						vector2d::Vector2D(Get_MF_Location(step[i]).data()[0], Get_MF_Location(step[i]).data()[1] - 1.5f), 
						0.5f
					);// 过渡
					
					path_plan.Add_Point(Get_MF_Location(step[i]), 0.f);// 去第一个格子中心
				}
			}
			else
			{
				if (i < step_num)
				{
					if (MF_map.Get_MF(step[i]) == 2)// 先拿路径上挡路的（顺路）
					{
						Dir d = Dir_From_To(step[i - 1], step[i]);
						
						float x = 0.f, y = 0.f;
						
						switch (d)
						{
							case Dir::F:
								x = 0.f;
								y = CHASSIS_MOVE;
								break;
							
							case Dir::B:
								x = 0.f;
								y = -CHASSIS_MOVE;
								break;
							
							case Dir::L:
								x = -CHASSIS_MOVE;
								y = 0.f;
								break;
							
							case Dir::R:
								x = CHASSIS_MOVE;
								y = 0.f;
								break;
							
							default:
								break;
						}
					
						path_plan.Add_End_Point(
							vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0] + x, Get_MF_Location(step[i - 1]).data()[1] + y),
							Dir_To_Yaw(d)
						);// 去拿取
						
						MF_map.Set_MF(step[i], 4);// 已拿取，变空格
					}
				}
				
				
				uint8_t count = 0;// 拿取不顺路的方块次数
				
				
				// 前
				if (MF_map.Kfs_On_Dir(step[i - 1], Dir::F) == 2)
				{
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0], Get_MF_Location(step[i - 1]).data()[1] + CHASSIS_MOVE), 
						Dir_To_Yaw(Dir::F)
					);// 去拿取
					
					count++;
					MF_map.Set_MF(step[i - 1] + 3, 4);// 已拿取，变空格
				}
				
				
				// 左
				if (MF_map.Kfs_On_Dir(step[i - 1], Dir::L) == 2)
				{
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0] - CHASSIS_MOVE, Get_MF_Location(step[i - 1]).data()[1]), 
						Dir_To_Yaw(Dir::L)
					);// 去拿取
					
					count++;
					MF_map.Set_MF(step[i - 1] - 1, 4);// 已拿取，变空格
				}
				
				
				// 后
				if (MF_map.Kfs_On_Dir(step[i - 1], Dir::B) == 2)
				{
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0], Get_MF_Location(step[i - 1]).data()[1] - CHASSIS_MOVE), 
						Dir_To_Yaw(Dir::B)
					);// 去拿取
					
					count++;
					MF_map.Set_MF(step[i - 1] - 3, 4);// 已拿取，变空格
				}
				
				// 右
				if (MF_map.Kfs_On_Dir(step[i - 1], Dir::R) == 2)
				{
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0] + CHASSIS_MOVE, Get_MF_Location(step[i - 1]).data()[1]), 
						Dir_To_Yaw(Dir::R)
					);// 去拿取
					
					count++;
					MF_map.Set_MF(step[i - 1] + 1, 4);// 已拿取，变空格
				}

				if (count > 0)
				{
					path_plan.Add_Point(
						Get_MF_Location(step[i - 1]), 
						0.f
					);// 拿完回到中心
				}

				if (i < step_num)
				{
					path_plan.Add_Point(
						Get_MF_Location(step[i]), 
						0.f
					);// 前往下一格子中心
				}
			}
		}
		
		path_plan.Add_End_Point(
			vector2d::Vector2D(Get_MF_Location(step[step_num - 1]).data()[0], Get_MF_Location(step[step_num - 1]).data()[1] + MF_SIZE), 
			0.f
		);// 出MF
	}
}