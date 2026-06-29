#include "RC_lift_chassis.h"

namespace chassis
{
	LiftChassis::LiftChassis(
		motor::Motor& L_lift_, motor::Motor& R_lift_,
		motor::Motor& L_wheel_, motor::Motor& R_wheel_,
		chassis::Chassis& chassis_, fusion::QEO& qeo_
	) : L_lift(L_lift_), R_lift(R_lift_), chassis(chassis_), qeo(qeo_), L_wheel(L_wheel_), R_wheel(R_wheel_),
	lift_event{
		path::Event3(5 , 0.1f, false, false),	  // EVENT_UP_2_READY_L
		path::Event3(6 , 0.4f, false, false),     // EVENT_UP_4_READY_L
		path::Event3(7 , 0.1f, false, false),     // EVENT_UP_2_READY_R
		path::Event3(8 , 0.4f, false, false),     // EVENT_UP_4_READY_R
		path::Event3(9 , 0.1f, false, false),     // EVENT_DOWN_2_READY_L
		path::Event3(10, 0.1f, false, false),     // EVENT_DOWN_4_READY_L
		path::Event3(11, 0.1f, false, false),     // EVENT_DOWN_2_READY_R
		path::Event3(12, 0.1f, false, false)      // EVENT_DOWN_4_READY_R
	}
	{
		state = LIFT_RESET;
		up_pos = -1;
		down_pos = -1;
		/*---------------------*/
		lift_trig = false;
	}
	
	void LiftChassis::Set_wheel_Vel(float vel)
	{
		if (state != LIFT_RESET_CHECK)
		{
			vel *= LIFT_WHEEL_VEL_TO_RPM;
			L_wheel.Set_Out_Rpm(vel);
			R_wheel.Set_Out_Rpm(-vel);
		}
		else
		{
			L_wheel.Set_Out_Rpm(0);
			R_wheel.Set_Out_Rpm(0);
		}
	}
	
	constexpr float LIFT_POS_THRESHOLD    = 9.f;
	
	
	
	constexpr float LIFT_RESET_R          = 4500;
	constexpr float LIFT_RESET_V_MAX      = 680;
	
	constexpr float LIFT_LOAD_R           = 2800;
	constexpr float LIFT_LOAD_V_MAX       = 680;
	
	
	
	constexpr float LIFT_CHASSIS_UP_VEL   = 0.36f;
	constexpr float LIFT_CHASSIS_DOWN_VEL = 0.38f;
	constexpr float LIFT_CHASSIS_FAST_VEL = 2.8f;
	
	constexpr float LIFT_CHASSIS_SLOW_ACC = 3.f;
	constexpr float LIFT_CHASSIS_FAST_ACC = 4.f;

	
	
	constexpr float LIFT_CHASSIS_UP_DIS = 0.35f;
	constexpr float LIFT_CHASSIS_DOWN_DIS = 0.3f;

	void LiftChassis::Lift(LiftAction a_, LiftHeigth h_, LiftDir d_, bool trig)
	{
		if (state == LIFT_UP_READY || state == LIFT_DOWN_READY)
		{
			if (a_ == LIFT_LOCK)
			{
				state = LIFT_RESET;
			}
		}
		
		Set_wheel_Vel(chassis.Get_Vel().y());

		for (uint8_t i = 0; i < 6; i++)
		{
			senser_value[i] = (bool)HAL_GPIO_ReadPin(SENSER_GPIO_PORT[i], SENSER_GPIO_PIN[i]);
		}
		
		switch (state)
		{
			/*=========================复位============================*/
			case LIFT_RESET:
			{
				// 解除底盘限制
				Chassis_Start();
				Chassis_Start_Spin();
				chassis.Set_Lin_Accel(LIFT_CHASSIS_FAST_ACC);
				chassis.Set_Max_Linear_Vel(LIFT_CHASSIS_FAST_VEL);
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);				
				
				Set_Front_Lift_Pos(RESET_POS);
				Set_Back_Lift_Pos(RESET_POS);
				
				state = LIFT_RESET_CHECK;
				break;
			}
			case LIFT_RESET_CHECK:
			{
				if (a_ != LIFT_LOCK && trig)
				{
					d = d_;

					if (a_ == LIFT_UP)
					{
						state = LIFT_UP_READY;
					}
					else
					{
						state = LIFT_DOWN_READY;
					}

					
					if (h_ == LIFT_20)
					{
						up_pos = UP_20_POS;
						down_pos = DOWN_20_POS;
					}
					else
					{
						up_pos = UP_40_POS;
						down_pos = DOWN_40_POS;
					}
				}
				break;
			}
			/*=========================复位============================*/
			
			
			/*===========================上台阶==========================*/
			case LIFT_UP_READY:
			{
				chassis.Set_Max_Linear_Vel(LIFT_CHASSIS_UP_VEL); // 限制底盘速度
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				
				Set_Front_Lift_Pos(up_pos);
				Set_Back_Lift_Pos(ZERO_POS + DELTA_H);
				
				state = LIFT_UP_READY_CHECK;
				break;
			}
			case LIFT_UP_READY_CHECK:
			{
				if (
					Get_Senser_Value(1) && 
					fabsf(Get_Front_Lift_Pos() - up_pos) < LIFT_POS_THRESHOLD &&
					fabsf(Get_Back_Lift_Pos() - (ZERO_POS + DELTA_H)) < LIFT_POS_THRESHOLD
				)
				{
					state = LIFT_UP_RISE;
				}
				break;
			}
			
			case LIFT_UP_RISE:
			{
				// 底盘停止
				Chassis_Stop();
				Chassis_Stop_Spin();
				
				Set_Front_Lift_Td(LIFT_LOAD_R, LIFT_LOAD_V_MAX);
				Set_Back_Lift_Td(LIFT_LOAD_R, LIFT_LOAD_V_MAX);
				
				Set_Front_Lift_Pos(ZERO_POS - DELTA_H_BIG);
				Set_Back_Lift_Pos(down_pos - DELTA_H);
				
				state = LIFT_UP_RISE_CHECK;
				break;
			}
			case LIFT_UP_RISE_CHECK:
			{
				if (
					fabsf(Get_Front_Lift_Pos() - (ZERO_POS - DELTA_H_BIG)) < LIFT_POS_THRESHOLD && 
					fabsf(Get_Back_Lift_Pos() - (down_pos - DELTA_H)) < LIFT_POS_THRESHOLD
				)
				{
					state = LIFT_UP_FORWARD;
				}
				break;
			}
			
			case LIFT_UP_FORWARD:
			{
				// 允许底盘平动
				Chassis_Start();
				
				Reset_Pos(); // 复位里程计
				
				chassis.Set_Lin_Accel(LIFT_CHASSIS_SLOW_ACC); // 限制加速度
				
				Set_Front_Lift_Td(LIFT_LOAD_R, LIFT_LOAD_V_MAX);
				Set_Back_Lift_Td(LIFT_LOAD_R, LIFT_LOAD_V_MAX);
				
				Set_Front_Lift_Pos(ZERO_POS - DELTA_H);
				Set_Back_Lift_Pos(down_pos - DELTA_H);
				
				state = LIFT_UP_FORWARD_CHECK;
				break;
			}
			case LIFT_UP_FORWARD_CHECK:
			{
				float dis = Get_Dis(); // 里程计当前值
				float max_vel = 0; // 最大速度
				
				if (dis < LIFT_CHASSIS_UP_DIS)
				{
					max_vel = sqrtf((LIFT_CHASSIS_UP_DIS - dis) * 2.f * LIFT_CHASSIS_SLOW_ACC + (LIFT_CHASSIS_UP_VEL * LIFT_CHASSIS_UP_VEL));
					
					max_vel = fminf(max_vel, LIFT_CHASSIS_FAST_VEL);
					max_vel = fmaxf(max_vel, LIFT_CHASSIS_UP_VEL);
				}
				else
				{
					max_vel = LIFT_CHASSIS_UP_VEL;
				}
				
				chassis.Set_Max_Linear_Vel(max_vel); // 实时计算最大速度
				
				if (
					Get_Senser_Value(4)
				)
				{
					state = LIFT_UP_WITHDRAW;
				}
				break;
			}
			
			case LIFT_UP_WITHDRAW:
			{
				chassis.Set_Lin_Accel(LIFT_CHASSIS_FAST_ACC); // 解除加速度限制
				
				Chassis_Stop(); // 停车
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				
				Set_Front_Lift_Pos(RESET_POS);
				Set_Back_Lift_Pos(RESET_POS);
				
				state = LIFT_UP_WITHDRAW_CHECK;
				break;
			}
			case LIFT_UP_WITHDRAW_CHECK:
			{
				if (
					Get_Back_Lift_Pos() > ZERO_POS
				)
				{
					state = LIFT_RESET;
				}
				break;
			}
			/*=========================上台阶============================*/
			
			
			/*=========================下台阶============================*/
			case LIFT_DOWN_READY:
			{
				chassis.Set_Max_Linear_Vel(LIFT_CHASSIS_DOWN_VEL); // 限制最大速度
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				
				Set_Front_Lift_Pos(ZERO_POS + DELTA_H);
				Set_Back_Lift_Pos(RESET_POS);
				
				state = LIFT_DOWN_READY_CHECK;
				break;
			}
			case LIFT_DOWN_READY_CHECK:
			{
				if (
					!Get_Senser_Value(3)
				)
				{
					state = LIFT_DOWN_STRETCH;
				}
				break;
			}
			
			case LIFT_DOWN_STRETCH:
			{
				// 底盘停止
				Chassis_Stop();
				Chassis_Stop_Spin();
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				
				Set_Front_Lift_Pos(down_pos - DELTA_H);
				Set_Back_Lift_Pos(ZERO_POS - DELTA_H_BIG);
				
				state = LIFT_DOWN_STRETCH_CHECK;
				break;
			}
			case LIFT_DOWN_STRETCH_CHECK:
			{
				if (
					fabsf(Get_Front_Lift_Pos() - (down_pos - DELTA_H)) < LIFT_POS_THRESHOLD && 
					fabsf(Get_Back_Lift_Pos() - (ZERO_POS - DELTA_H_BIG)) < LIFT_POS_THRESHOLD
				)
				{
					state = LIFT_DOWN_FORWARD;
				}
				break;
			}
			
			case LIFT_DOWN_FORWARD:
			{
				Chassis_Start(); // 允许平动
				
				Reset_Pos(); // 复位里程计
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				
				//Set_Front_Lift_Pos(down_pos - DELTA_H);
				//Set_Back_Lift_Pos(ZERO_POS - DELTA_H);
				
				state = LIFT_DOWN_FORWARD_CHECK;
				break;
			}
			case LIFT_DOWN_FORWARD_CHECK:
			{
				float dis = Get_Dis(); // 里程计当前值
				float max_vel = 0; // 最大速度
				
				if (dis < LIFT_CHASSIS_DOWN_DIS)
				{
					max_vel = sqrtf((LIFT_CHASSIS_DOWN_DIS - dis) * 2.f * LIFT_CHASSIS_SLOW_ACC + (LIFT_CHASSIS_DOWN_VEL * LIFT_CHASSIS_DOWN_VEL));
					
					max_vel = fminf(max_vel, LIFT_CHASSIS_FAST_VEL);
					max_vel = fmaxf(max_vel, LIFT_CHASSIS_DOWN_VEL);
				}
				else
				{
					max_vel = LIFT_CHASSIS_DOWN_VEL;
				}
				
				chassis.Set_Max_Linear_Vel(max_vel); // 实时计算最大速度
				
				if (
					!Get_Senser_Value(5)
				)
				{
					state = LIFT_DOWN_FALL;
				}
				break;
			}
			
			case LIFT_DOWN_FALL:
			{
				chassis.Set_Lin_Accel(LIFT_CHASSIS_FAST_ACC); // 解除加速度限制
				
				Chassis_Stop(); // 停车
				
				Set_Front_Lift_Td(LIFT_LOAD_R, LIFT_LOAD_V_MAX);
				Set_Back_Lift_Td(LIFT_LOAD_R, LIFT_LOAD_V_MAX);
				
				Set_Front_Lift_Pos(ZERO_POS + DELTA_H);
				Set_Back_Lift_Pos(up_pos);
				
				state = LIFT_DOWN_FALL_CHECK;
				break;
			}
			case LIFT_DOWN_FALL_CHECK:
			{
				if (
					fabsf(Get_Front_Lift_Pos() - (ZERO_POS + DELTA_H)) < LIFT_POS_THRESHOLD && 
					fabsf(Get_Back_Lift_Pos() - up_pos) < LIFT_POS_THRESHOLD)
				{
					state = LIFT_DOWN_WITHDRAW;
				}
				break;
			}
			
			case LIFT_DOWN_WITHDRAW:
			{
				Chassis_Start();
				chassis.Set_Max_Linear_Vel(LIFT_CHASSIS_FAST_VEL);
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				
				Set_Front_Lift_Pos(RESET_POS);
				Set_Back_Lift_Pos(up_pos);
				
				state = LIFT_DOWN_WITHDRAW_CHECK;
				break;
			}
			case LIFT_DOWN_WITHDRAW_CHECK:
			{
				if (
					!Get_Senser_Value(6)
				)
				{
					state = LIFT_RESET;
				}
				break;
			}
			/*===========================下台阶==========================*/
			default:
			{
				state = LIFT_RESET;
				break;
			}
		}
	}
}