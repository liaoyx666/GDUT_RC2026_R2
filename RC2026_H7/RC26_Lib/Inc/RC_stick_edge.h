#pragma once

#ifdef __cplusplus
#include "RC_event3.h"
#include "RC_omni_chassis.h"
#include "RC_path_plan3.h"
#include "RC_timer.h"
#include "RC_gantry.h"
#include "RC_gripper.h"
#include "RC_IR_communication.h"
#include "RC_data_pool.h"
#include "RC_ws2812.h"

enum StickDockState : uint8_t 
{
	STICK_DOCK_RESET = 0,
	
	
	STICK_DOCK_DIS_PATH,
	STICK_DOCK_TAKE_CTRL,
	STICK_DOCK_GET_TIME,
	STICK_DOCK_STICK,
	STICK_DOCK_STOP,
	STICK_DOCK_WAIT_GO_CMD,
	STICK_DOCK_WAIT,
	STICK_DOCK_FINISH,
	
	DOCK_SET_POS,
	DOCK_OPEN,
	DOCK_WAIT_GO_CMD,
	DOCK_WAIT,
	DOCK_FINISH,
	
};


//volatile uint8_t count_ = 0;


class StickEdge
{
public:
	StickEdge(
		chassis::Omni4Chassis& c_, 
		path::PathPlan3& p_, 
		gantry::Gantry& gan_,
		gantry::Gripper& gripper_,
		data::RobotPose& pose_,
		ws2812::Ws2812B& led_
	);
	~StickEdge() = default;

	void Stick_Edge()
	{
//		if (open_cmd.Get_Cmd())
//		{
//		
//			count_++;
//		}
		
		
		
		switch (state)
		{
			case STICK_DOCK_RESET:
			{
				if (stick_l_event.Is_Trig())
				{
					state = STICK_DOCK_TAKE_CTRL;
				}
				else if (stick_l_event_2.Is_Trig())
				{
					state = DOCK_SET_POS;
				}
				
				break;
			}
			
			case STICK_DOCK_TAKE_CTRL:
			{
				float tar_yaw;
				
				if (data::Side::Is_Blue_Left_Side())
				{
					tar_yaw = HALF_PI;
				}
				else
				{
					tar_yaw = -HALF_PI;
				}
				
				float delta = tar_yaw - pose.Yaw();
				
				if (delta > PI)
					delta -= TWO_PI;
				else if (delta < -PI)
					delta += TWO_PI;
				
				if (
					user.Take_Control() &&
					fabsf(delta) < (2.f / 180.f * PI)
				)
				{
					state = STICK_DOCK_DIS_PATH;
				}
				
				break;
			}
			
			case STICK_DOCK_DIS_PATH:
			{
				p.Disable(); // 路径规划失能
				c.Set_Max_Current(3000);
				
				user.Set_X(0.0);
				if (data::Side::Is_Blue_Left_Side())
				{
					user.Set_Y(0.09);
				}
				else
				{
					user.Set_Y(-0.09);
				}
				user.Set_Z(0.06);
				user.Set_P_Max_T(3);
				user.Set_P(0.2);
				
				
				state = STICK_DOCK_GET_TIME;
				break;
			}
			
			
			
			
			case STICK_DOCK_GET_TIME:
			{
				last_time = timer::Timer::Get_TimeStamp();
				
				state = STICK_DOCK_STICK;
				break;
			}
			
			
			case STICK_DOCK_STICK:
			{
				if (data::Side::Is_Blue_Left_Side())
				{
					c.Set_Robot_Lin_Vel(vector2d::Vector2D(0, 0.2));
				}
				else
				{
					c.Set_Robot_Lin_Vel(vector2d::Vector2D(0, -0.2));
				}
				c.Set_Ang_Vel(0);

				state = STICK_DOCK_STOP;
				break;
			}
			
			
			case STICK_DOCK_STOP:
			{
				if (data::Side::Is_Blue_Left_Side())
				{
					c.Set_Robot_Lin_Vel(vector2d::Vector2D(0, 0.2));
				}
				else
				{
					c.Set_Robot_Lin_Vel(vector2d::Vector2D(0, -0.2));
				}
				
				c.Set_Ang_Vel(0);
				
				if (open_cmd.Get_Cmd())
				{
					user.Set_P(0.5);
					
					user.Set_Z(0);
					
					gripper.Open();
					
					state = STICK_DOCK_WAIT_GO_CMD;
					
				}
				break;
			}
			
			
			case STICK_DOCK_WAIT_GO_CMD:
			{
				if (go_cmd.Get_Cmd())
				{
					state = STICK_DOCK_WAIT;
					last_time = timer::Timer::Get_TimeStamp();
				}
				
				break;
			}
			
			
			case STICK_DOCK_WAIT:
			{
				if (timer::Timer::Get_DeltaTime(last_time) > 2000000)
				{
					state = STICK_DOCK_FINISH;
				}
				break;
			}
			
			case STICK_DOCK_FINISH:
			{
				user.Set_P_Max_T(27);
				c.Set_Max_Current(16384);
				user.Set_Reset_Pos();
				user.Give_Control();
				stick_l_event.Finish();
				p.Enable(); // 路径规划
				
				state = STICK_DOCK_RESET;
				break;
			}
			
			/*-----------------------------------------------*/
			
			case DOCK_SET_POS:
			{
				if (user.Take_Control())
				{
					user.Set_X(0.0);
					user.Set_Y(-0.05);
					user.Set_Z(0.06);
					user.Set_P_Max_T(3);
					user.Set_P(0.2);
					
					if (open_cmd.Get_Cmd())
					{
						state = DOCK_OPEN;
					}
				}
				break;
			}
			
			
			case DOCK_OPEN:
			{
				user.Set_P(0.5);
					
				user.Set_Z(0);
				
				gripper.Open();
				
				state = DOCK_WAIT_GO_CMD;
				break;
			}
			
			case DOCK_WAIT_GO_CMD:
			{
				if (go_cmd.Get_Cmd())
				{
					last_time = timer::Timer::Get_TimeStamp();
					state = DOCK_WAIT;
				}
				break;
			}
			
			case DOCK_WAIT:
			{
				if (timer::Timer::Get_DeltaTime(last_time) > 2000000)
				{
					state = DOCK_FINISH;
				}
				
				break;
			}
			
			case DOCK_FINISH:
			{
				user.Set_P_Max_T(27);
				user.Set_Reset_Pos();
				user.Give_Control();
				stick_l_event_2.Finish();
				
				state = STICK_DOCK_RESET;
				break;
			}
					
			
			
			default:
			{
				state = STICK_DOCK_RESET;
				break;
			}
		
		}
	}
	
	
private:
	path::Event3 stick_l_event;
	path::Event3 stick_l_event_2;
	//float y_pos = 0;
	StickDockState state;
	chassis::Omni4Chassis& c;
	path::PathPlan3& p;
	uint32_t last_time;
	gantry::Gripper& gripper;
	data::RobotPose& pose;
	gantry::GantryUser user;

	uint8_t count_ = 0;

	IR::IRCmd open_cmd;
	IR::IRCmd go_cmd;
};
#endif
