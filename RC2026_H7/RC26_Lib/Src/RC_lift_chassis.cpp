#include "RC_lift_chassis.h"

namespace chassis
{
	LiftChassis::LiftChassis(
		motor::Motor& L_lift_, motor::Motor& R_lift_,
		motor::Motor& L_wheel_, motor::Motor& R_wheel_,
		chassis::Chassis* chassis_, fusion::QEO& qeo_
	) : L_lift(L_lift_), R_lift(R_lift_), chassis(chassis_), qeo(qeo_), L_wheel(L_wheel_), R_wheel(R_wheel_),
	lift_event{
		path::Event3(5 , 0.1f, false, false),	  // EVENT_UP_2_READY_L
		path::Event3(6 , 0.5f, false, false),     // EVENT_UP_4_READY_L
		path::Event3(7 , 0.1f, false, false),     // EVENT_UP_2_READY_R
		path::Event3(8 , 0.5f, false, false),     // EVENT_UP_4_READY_R
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
		if (state != LIFT_RESET)
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
	
	constexpr float LIFT_POS_THRESHOLD    = 7.f;
	
	constexpr float LIFT_RESET_R          = 4500;
	constexpr float LIFT_RESET_V_MAX      = 680;
	
	constexpr float LIFT_LOAD_R           = 2800;
	constexpr float LIFT_LOAD_V_MAX       = 680;
	
	
	
	constexpr float LIFT_CHASSIS_UP_VEL = 0.3f;
	
	constexpr float LIFT_CHASSIS_SLOW_VEL = 0.36f;
	constexpr float LIFT_CHASSIS_FAST_VEL = 2.8f;
	
	
	constexpr float LIFT_CHASSIS_SLOW_ACC = 2.5f;
	constexpr float LIFT_CHASSIS_FAST_ACC = 4.f;

	
	constexpr float LIFT_CHASSIS_DIS = 0.4f;

	void LiftChassis::Lift(LiftAction a_, LiftHeigth h_, LiftDir d_, bool trig)
	{
		if (state == LIFT_UP_READY || state == LIFT_DOWN_READY)
		{
			if (a_ == LIFT_LOCK)
			{
				state = LIFT_RESET;
			}
		}
		
		if (chassis)
		{
			Set_wheel_Vel(chassis->Get_Vel().y());
		}
		else
		{
			Set_wheel_Vel(0);
		}
		
		
		for (uint8_t i = 0; i < 6; i++)
		{
			senser_value[i] = (bool)HAL_GPIO_ReadPin(SENSER_GPIO_PORT[i], SENSER_GPIO_PIN[i]);
		}
		
		switch (state)
		{
			/*=========================复位============================*/
			case LIFT_RESET:
			{
				Chassis_Start();
				Chassis_Start_Spin();
				if (chassis) chassis->Set_Lin_Accel(LIFT_CHASSIS_FAST_ACC);
				if (chassis) chassis->Set_Max_Linear_Vel(LIFT_CHASSIS_FAST_VEL);
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);				
				
				Set_Front_Lift_Pos(RESET_POS);
				Set_Back_Lift_Pos(RESET_POS);
				
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
				Chassis_Start();
				Chassis_Start_Spin();
				
				if (chassis) chassis->Set_Max_Linear_Vel(LIFT_CHASSIS_UP_VEL);
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				
				Set_Front_Lift_Pos(up_pos);
				Set_Back_Lift_Pos(ZERO_POS);
				
				state = LIFT_UP_READY_CHECK;
				break;
			}
			case LIFT_UP_READY_CHECK:
			{
				if (
					Get_Senser_Value(1) && 
					fabsf(Get_Front_Lift_Pos() - up_pos) < LIFT_POS_THRESHOLD &&
					fabsf(Get_Back_Lift_Pos() - ZERO_POS) < LIFT_POS_THRESHOLD
				)
				{
					state = LIFT_UP_RISE;
				}
				break;
			}
			
			case LIFT_UP_RISE:
			{
				Chassis_Stop();
				Chassis_Stop_Spin();
				
				
				Set_Front_Lift_Td(LIFT_LOAD_R, LIFT_LOAD_V_MAX);
				Set_Back_Lift_Td(LIFT_LOAD_R, LIFT_LOAD_V_MAX);
				
				Set_Front_Lift_Pos(ZERO_POS - 7.f);
				Set_Back_Lift_Pos(down_pos - 7.f);
				
				state = LIFT_UP_RISE_CHECK;
				break;
			}
			case LIFT_UP_RISE_CHECK:
			{
				if (
					fabsf(Get_Front_Lift_Pos() - (ZERO_POS - 7.f)) < LIFT_POS_THRESHOLD && 
					fabsf(Get_Back_Lift_Pos() - (down_pos - 7.f)) < LIFT_POS_THRESHOLD
				)
				{
					state = LIFT_UP_FORWARD;
				}
				break;
			}
			
			case LIFT_UP_FORWARD:
			{
				Chassis_Start();
				Chassis_Stop_Spin();
				Reset_Pos();
				if (chassis) chassis->Set_Lin_Accel(LIFT_CHASSIS_SLOW_ACC);
				
				Set_Front_Lift_Td(LIFT_LOAD_R, LIFT_LOAD_V_MAX);
				Set_Back_Lift_Td(LIFT_LOAD_R, LIFT_LOAD_V_MAX);
				
				Set_Front_Lift_Pos(ZERO_POS - 7.f);
				Set_Back_Lift_Pos(down_pos - 7.f);
				
				state = LIFT_UP_FORWARD_CHECK;
				break;
			}
			case LIFT_UP_FORWARD_CHECK:
			{
				float max_vel = 0;
				float dis = Get_Dis();
				
				if (dis < LIFT_CHASSIS_DIS)
				{
					max_vel = sqrtf((LIFT_CHASSIS_DIS - dis) * 2.f * LIFT_CHASSIS_SLOW_ACC + (LIFT_CHASSIS_UP_VEL * LIFT_CHASSIS_UP_VEL));
					
					max_vel = fminf(max_vel, LIFT_CHASSIS_FAST_VEL);
					max_vel = fmaxf(max_vel, LIFT_CHASSIS_UP_VEL);
				}
				else
				{
					max_vel = LIFT_CHASSIS_UP_VEL;
				}
				
				if (chassis) chassis->Set_Max_Linear_Vel(max_vel);
				
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
				if (chassis) chassis->Set_Lin_Accel(LIFT_CHASSIS_FAST_ACC);
				Chassis_Stop();
				
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
				Chassis_Start();
				if (chassis) chassis->Set_Max_Linear_Vel(LIFT_CHASSIS_SLOW_VEL); 
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				
				Set_Front_Lift_Pos(ZERO_POS);
				Set_Back_Lift_Pos(RESET_POS);
				
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
				Chassis_Stop();
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				
				Set_Front_Lift_Pos(down_pos - 7.f);
				Set_Back_Lift_Pos(ZERO_POS - 7.f);
				
				if (
					fabsf(Get_Front_Lift_Pos() - (down_pos - 7.f)) < LIFT_POS_THRESHOLD && 
					fabsf(Get_Back_Lift_Pos() - (ZERO_POS - 7.f)) < LIFT_POS_THRESHOLD
				)
				{
					state = LIFT_DOWN_FORWARD;
				}
				break;
			}
			
			case LIFT_DOWN_FORWARD:
			{
				Chassis_Start();
				if (chassis) chassis->Set_Max_Linear_Vel(LIFT_CHASSIS_SLOW_VEL);
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				
				Set_Front_Lift_Pos(down_pos - 7.f);
				Set_Back_Lift_Pos(ZERO_POS - 7.f);
				
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
				Chassis_Stop();
				
				Set_Front_Lift_Td(LIFT_LOAD_R, LIFT_LOAD_V_MAX);
				Set_Back_Lift_Td(LIFT_LOAD_R, LIFT_LOAD_V_MAX);
				
				Set_Front_Lift_Pos(ZERO_POS);
				Set_Back_Lift_Pos(up_pos);
				
				if (
					fabsf(Get_Front_Lift_Pos() - ZERO_POS) < LIFT_POS_THRESHOLD && 
					fabsf(Get_Back_Lift_Pos() - up_pos) < LIFT_POS_THRESHOLD)
				{
					state = LIFT_DOWN_WITHDRAW;
				}
				break;
			}
			
			case LIFT_DOWN_WITHDRAW:
			{
				Chassis_Start();
				if (chassis) chassis->Set_Max_Linear_Vel(LIFT_CHASSIS_FAST_VEL);
				
				Set_Front_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				Set_Back_Lift_Td(LIFT_RESET_R, LIFT_RESET_V_MAX);
				
				Set_Front_Lift_Pos(RESET_POS);
				Set_Back_Lift_Pos(up_pos);
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