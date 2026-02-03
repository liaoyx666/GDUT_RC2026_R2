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
				return PI / 2.f;
				break;
			
			case Dir::R:
				return -PI / 2.f;
				break;
			
			default:
				return 0.f;
				break;
		}
	}
	
	/////////////////////////////////////////////////////////////////////////////
	
	void BestPath::MF_Best_Path_Plan(Map& MF_map, path::PathPlan2& path_plan)
	{
		while(is_init == false || MF_map.Is_Init() == false)
		{
			osDelay(1);
		}
		
		// 过渡点,硬编码
		path_plan.Add_Point(
			vector2d::Vector2D(Get_MF_Location(2).data()[0] - 2.f, Get_MF_Location(2).data()[1]), 		
			0,
			0,
			PATH_MAX_PARAM,									
			PATH_MAX_PARAM,											
			PATH_MAX_PARAM,												
			PATH_MAX_PARAM,										
			PATH_MAX_PARAM,														
			PATH_MAX_PARAM,																									
			0
		);
		
		if (MF_map.Get_MF(2) == 2)
		{
			// 2方块变空格
			MF_map.Set_MF(2, 4);
			
			// 2有方块
			// 夹取
			path_plan.Add_End_Point(
				vector2d::Vector2D(Get_MF_Location(2).data()[0] - MF_SIZE_HALF - CHASSIS_HALF - MIN_DIS, Get_MF_Location(2).data()[1]), 
				0,// 到达前目标yaw
				0,// 离开前目标yaw
				1,
				PATH_MAX_PARAM,
				PATH_MAX_PARAM,
				PATH_MAX_PARAM,
				PATH_MAX_PARAM,
				PATH_MAX_PARAM,
				false,// 是否停止																				
				0// 事件id
			);
			
			// 后退
			// 准备上2
			path_plan.Add_Point(
				vector2d::Vector2D(Get_MF_Location(2).data()[0] - 2.f * MF_SIZE_HALF - 0.1f, Get_MF_Location(2).data()[1]), 					
				0,
				0,
				1,
				PATH_MAX_PARAM,										
				PATH_MAX_PARAM,												
				PATH_MAX_PARAM,										
				PATH_MAX_PARAM,														
				PATH_MAX_PARAM,																									
				1
			);
		}
		else
		{
			// 2无方块
			// 准备上2
			path_plan.Add_Point(
				vector2d::Vector2D(Get_MF_Location(2).data()[0] - 1.8f, Get_MF_Location(2).data()[1]), 					
				0,
				0,				
				1,
				PATH_MAX_PARAM,										
				PATH_MAX_PARAM,												
				PATH_MAX_PARAM,										
				PATH_MAX_PARAM,														
				PATH_MAX_PARAM,																									
				1
			);
		}
		
		// 到2中心后5cm
		// 等待复位
		path_plan.Add_End_Point(
			vector2d::Vector2D(Get_MF_Location(2).data()[0] - CHASSIS_OFFSET, Get_MF_Location(2).data()[1]),// 坐标  
			PATH_NO_TARGET_YAW,// 到达前目标yaw
			PATH_NO_TARGET_YAW,// 离开前目标yaw
			0.5,
			PATH_MAX_PARAM,
			PATH_MAX_PARAM,
			PATH_MAX_PARAM,
			PATH_MAX_PARAM,
			PATH_MAX_PARAM,
			true,// 是否停止																				
			3// 事件id
		);
		
		last_yaw = 0;
		
		///////////////////////////////////////////
		
		// i = 0为2，从i = 1开始
		for (uint8_t i = 1; i < step_num + 1; i++)
		{
			if (i < step_num)
			{
				if (MF_map.Get_MF(step[i]) == 2)// 先拿路径上挡路的（顺路）
				{
					// 有东西
					Dir d = Dir_From_To(step[i - 1], step[i]);
					
					float x = 0.f, y = 0.f;
					
					switch (d)
					{
						case Dir::F:
							y = 0.f;
							x = CHASSIS_MOVE;
							break;
						
						case Dir::B:
							y = 0.f;
							x = -CHASSIS_MOVE;
							break;
						
						case Dir::L:
							y = CHASSIS_MOVE;
							x = 0.f;
							break;
						
						case Dir::R:
							y = -CHASSIS_MOVE;
							x = 0.f;
							break;
						
						default:
							break;
					}
					
					float target_yaw = Dir_To_Yaw(d);
					
					// 先回到中心
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0], Get_MF_Location(step[i - 1]).data()[1]),// 坐标  
						last_yaw,// 到达前目标yaw					
						target_yaw,// 离开前目标yaw					
						1,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						false,// 是否停止																				
						0// 事件id
					);
					
					uint8_t event_id = 0;
					
					// 判断高低
					if (MF_high[step[i] - 1] - MF_high[step[i - 1] - 1] > 0)
					{
						// 夹高处
						event_id = 4;
					}
					else
					{
						// 夹低处
						event_id = 5;
					}
					
					// 去夹取
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0] + x, Get_MF_Location(step[i - 1]).data()[1] + y),// 坐标  
						target_yaw,// 到达前目标yaw
						target_yaw,// 离开前目标yaw
						1,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						false,// 是否停止																				
						0// 事件id
					);
					
					last_yaw = target_yaw;
					
					MF_map.Set_MF(step[i], 4);// 已拿取，变空格
				}
			

				// 前
				if (MF_map.Kfs_On_Dir(step[i - 1], Dir::F) == 2)
				{
					// 先回到中心
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0], Get_MF_Location(step[i - 1]).data()[1]),
						last_yaw,// 到达前目标yaw					
						Dir_To_Yaw(Dir::F),// 离开前目标yaw					
						1,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						false,// 是否停止																				
						0// 事件id
					);
					
					uint8_t event_id = 0;
					
					// 判断高低
					if (MF_high[step[i - 1] - 1 + 3] - MF_high[step[i - 1] - 1] > 0)
					{
						// 夹高处
						event_id = 4;
					}
					else
					{
						// 夹低处
						event_id = 5;
					}
					
					// 去夹取
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0] + CHASSIS_MOVE, Get_MF_Location(step[i - 1]).data()[1]),
						Dir_To_Yaw(Dir::F),// 到达前目标yaw
						Dir_To_Yaw(Dir::F),// 离开前目标yaw
						1,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						false,// 是否停止																				
						0// 事件id
					);
					
					last_yaw = Dir_To_Yaw(Dir::F);
					
					MF_map.Set_MF(step[i - 1] + 3, 4);// 已拿取，变空格
				}
				
				// 左
				if (MF_map.Kfs_On_Dir(step[i - 1], Dir::L) == 2)
				{
					// 先回到中心
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0], Get_MF_Location(step[i - 1]).data()[1]),
						last_yaw,// 到达前目标yaw					
						Dir_To_Yaw(Dir::L),// 离开前目标yaw					
						1,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						false,// 是否停止																				
						0// 事件id
					);
					
					uint8_t event_id = 0;
					
					// 判断高低
					if (MF_high[step[i - 1] - 1 - 1] - MF_high[step[i - 1] - 1] > 0)
					{
						// 夹高处
						event_id = 4;
					}
					else
					{
						// 夹低处
						event_id = 5;
					}
					
					// 去夹取
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0], Get_MF_Location(step[i - 1]).data()[1] + CHASSIS_MOVE),
						Dir_To_Yaw(Dir::L),// 到达前目标yaw
						Dir_To_Yaw(Dir::L),// 离开前目标yaw
						1,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						false,// 是否停止																				
						0// 事件id
					);
					
					last_yaw = Dir_To_Yaw(Dir::L);
				
					MF_map.Set_MF(step[i - 1] - 1, 4);// 已拿取，变空格
				}
				
				// 后
				if (MF_map.Kfs_On_Dir(step[i - 1], Dir::B) == 2)
				{
					// 先回到中心
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0], Get_MF_Location(step[i - 1]).data()[1]),
						last_yaw,// 到达前目标yaw					
						Dir_To_Yaw(Dir::B),// 离开前目标yaw					
						1,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						false,// 是否停止																				
						0// 事件id
					);
					
					uint8_t event_id = 0;
					
					// 判断高低
					if (MF_high[step[i - 1] - 1 - 3] - MF_high[step[i - 1] - 1] > 0)
					{
						// 夹高处
						event_id = 4;
					}
					else
					{
						// 夹低处
						event_id = 5;
					}
					
					// 去夹取
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0], Get_MF_Location(step[i - 1]).data()[1] - CHASSIS_MOVE),
						Dir_To_Yaw(Dir::B),// 到达前目标yaw
						Dir_To_Yaw(Dir::B),// 离开前目标yaw
						1,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						false,// 是否停止																				
						0// 事件id
					);
					
					last_yaw = Dir_To_Yaw(Dir::B);
		
					MF_map.Set_MF(step[i - 1] - 3, 4);// 已拿取，变空格
				}
				
				// 右
				if (MF_map.Kfs_On_Dir(step[i - 1], Dir::R) == 2)
				{
					// 先回到中心
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0], Get_MF_Location(step[i - 1]).data()[1]),
						last_yaw,// 到达前目标yaw					
						Dir_To_Yaw(Dir::R),// 离开前目标yaw					
						1,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						false,// 是否停止																				
						0// 事件id
					);
					
					uint8_t event_id = 0;
					
					// 判断高低
					if (MF_high[step[i - 1] - 1 + 1] - MF_high[step[i - 1] - 1] > 0)
					{
						// 夹高处
						event_id = 4;
					}
					else
					{
						// 夹低处
						event_id = 5;
					}
					
					// 去夹取
					path_plan.Add_End_Point(
						vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0], Get_MF_Location(step[i - 1]).data()[1] - CHASSIS_MOVE),
						Dir_To_Yaw(Dir::R),// 到达前目标yaw
						Dir_To_Yaw(Dir::R),// 离开前目标yaw
						1,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						PATH_MAX_PARAM,
						false,// 是否停止																				
						0// 事件id
					);
					
					last_yaw = Dir_To_Yaw(Dir::R);
					
					MF_map.Set_MF(step[i - 1] + 1, 4);// 已拿取，变空格
				}
			}

			if (i < step_num)
			{
				uint8_t event_id = 0;
				
				// 判断高低
				if (MF_high[step[i] - 1] - MF_high[step[i - 1] - 1] > 0)
				{
					// 上台阶
					event_id = 1;
				}
				else
				{
					// 下台阶
					event_id = 2;
				}
				
				// 下一个台阶方向
				Dir d = Dir_From_To(step[i - 1], step[i]); 
				
				// 回到中心准备上下台阶
				path_plan.Add_End_Point(
					vector2d::Vector2D(Get_MF_Location(step[i - 1]).data()[0], Get_MF_Location(step[i - 1]).data()[1]),
					last_yaw,// 到达前目标yaw
					Dir_To_Yaw(d),// 离开前目标yaw
					1,
					PATH_MAX_PARAM,
					PATH_MAX_PARAM,
					PATH_MAX_PARAM,
					PATH_MAX_PARAM,
					PATH_MAX_PARAM,
					false,// 是否停止																				
					event_id// 事件id
				);
					
				float x = 0.f, y = 0.f;
					
				switch (d)
				{
					case Dir::F:
						y = 0.f;
						x = -CHASSIS_OFFSET;
						break;
					
					case Dir::B:
						y = 0.f;
						x = CHASSIS_OFFSET;
						break;
					
					case Dir::L:
						y = -CHASSIS_OFFSET;
						x = 0.f;
						break;
					
					case Dir::R:
						y = CHASSIS_OFFSET;
						x = 0.f;
						break;
					
					default:
						break;
				}
				
				// 到下一台阶中心偏移5cm
				// 等待恢复
				path_plan.Add_End_Point(
					vector2d::Vector2D(Get_MF_Location(step[i]).data()[0] + x, Get_MF_Location(step[i]).data()[1] + y),
					PATH_NO_TARGET_YAW,// 到达前目标yaw
					PATH_NO_TARGET_YAW,// 离开前目标yaw
					0.5,
					PATH_MAX_PARAM,
					PATH_MAX_PARAM,
					PATH_MAX_PARAM,
					PATH_MAX_PARAM,
					PATH_MAX_PARAM,
					true,// 是否停止																				
					3// 事件id
				);

			}
		}
		
		// 回到最后台阶中心准备下台阶出MF
		path_plan.Add_End_Point(
			vector2d::Vector2D(Get_MF_Location(step[step_num - 1]).data()[0], Get_MF_Location(step[step_num - 1]).data()[1]),
			last_yaw,// 到达前目标yaw
			-PI,// 离开前目标yaw
			1,
			PATH_MAX_PARAM,
			PATH_MAX_PARAM,
			PATH_MAX_PARAM,
			PATH_MAX_PARAM,
			PATH_MAX_PARAM,
			false,// 是否停止																				
			2// 事件id
		);
		
		// 出MF
		path_plan.Add_End_Point(
			vector2d::Vector2D(Get_MF_Location(step[step_num - 1]).data()[0] + (MF_SIZE_HALF * 2.f), Get_MF_Location(step[step_num - 1]).data()[1]),
			PATH_NO_TARGET_YAW,// 到达前目标yaw
			PATH_NO_TARGET_YAW,// 离开前目标yaw
			1,
			PATH_MAX_PARAM,
			PATH_MAX_PARAM,
			PATH_MAX_PARAM,
			PATH_MAX_PARAM,
			PATH_MAX_PARAM,
			true,// 是否停止																				
			3// 事件id
		);
	}
}