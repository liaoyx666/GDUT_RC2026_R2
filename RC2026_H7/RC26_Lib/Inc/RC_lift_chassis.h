#pragma once
#include "RC_motor.h"
#include "gpio.h"
#include "RC_chassis.h"
#include "RC_traj_track3.h"
#include "RC_event3.h"
#include "RC_QEO.h"

#ifdef __cplusplus
namespace chassis
{
	constexpr float ZERO_POS    = 0.f;
    constexpr float UP_20_POS   = 295.f + ZERO_POS;
    constexpr float UP_40_POS   = 590.f + ZERO_POS;
    constexpr float DOWN_20_POS = -295.f + ZERO_POS;
    constexpr float DOWN_40_POS = -590.f + ZERO_POS;
    constexpr float RESET_POS   = 100.f;
	
	constexpr float DELTA_H 	= 16.f;
	constexpr float DELTA_H_BIG	= 25.f;

    constexpr float L_LIFT_POL  = -1.f;
    constexpr float R_LIFT_POL  = -1.f;
	
	
	/*
	a2
	d14
	c12
	g1
	d15
	f9
	*/
	
	
	const static GPIO_TypeDef* SENSER_GPIO_PORT[6] =
	{
		GPIOA,
		GPIOD,
		GPIOC,
		GPIOG,
		GPIOD,
		GPIOF
	};
	
	constexpr static uint16_t SENSER_GPIO_PIN[6] =
	{
		GPIO_PIN_2,
		GPIO_PIN_14,
		GPIO_PIN_12,
		GPIO_PIN_1,
		GPIO_PIN_15,
		GPIO_PIN_9
	};
	
	enum LiftState : uint8_t
	{
		LIFT_RESET = 0, 	/* 默认状态 */
		LIFT_RESET_CHECK,
		
		LIFT_UP_READY, 		/* 准备上台阶 */
		LIFT_UP_READY_CHECK,
		LIFT_UP_RISE, 		/* 抬升 */
		LIFT_UP_RISE_CHECK,
		LIFT_UP_FORWARD, 	/* 前进 */
		LIFT_UP_FORWARD_CHECK,
		LIFT_UP_WITHDRAW, 	/* 收回机构 */
		LIFT_UP_WITHDRAW_CHECK,
		
		LIFT_DOWN_READY, 	/* 准备下台阶 */
		LIFT_DOWN_READY_CHECK,
		LIFT_DOWN_STRETCH, 	/* 伸出机构 */
		LIFT_DOWN_STRETCH_CHECK,
		LIFT_DOWN_FORWARD, 	/* 前进 */
		LIFT_DOWN_FORWARD_CHECK,
		LIFT_DOWN_FALL, 	/* 下降 */
		LIFT_DOWN_FALL_CHECK,
		LIFT_DOWN_WITHDRAW,	/* 收回机构 */
		LIFT_DOWN_WITHDRAW_CHECK,
	};
	
	enum LiftDir : uint8_t
	{
		LIFT_L = 0,
		LIFT_R = 1,
	};
	
	enum LiftHeigth : uint8_t
	{
		LIFT_20 = 0,
		LIFT_40 = 1,
	};
	
	enum LiftAction: uint8_t
	{
		LIFT_LOCK = 0,
		LIFT_UP = 1,
		LIFT_DOWN = 2,
	};
	
	constexpr float LIFT_WHEEL_RADIUS = 0.02f;
	constexpr float LIFT_WHEEL_VEL_TO_RPM = 1.f / LIFT_WHEEL_RADIUS * (60.0f / (2.0f * PI));
	
	class LiftChassis
    {
    public:
		LiftChassis(
			motor::Motor& L_lift_, motor::Motor& R_lift_,
			motor::Motor& L_wheel_, motor::Motor& R_wheel_,
			chassis::Chassis& chassis_, fusion::QEO& qeo_
		);
		~LiftChassis() = default;
		
		void Lift(LiftAction a_, LiftHeigth h_, LiftDir d_, bool trig);
		bool Is_End() const {return (state == LIFT_RESET_CHECK);}
	
		/*------------------------------------*/
		
		inline void Auto_Lift()
		{
			// EVENT_UP_2_READY_L
			// EVENT_UP_4_READY_L
			// EVENT_UP_2_READY_R
			// EVENT_UP_4_READY_R
			// EVENT_DOWN_2_READY_L
			// EVENT_DOWN_4_READY_L
			// EVENT_DOWN_2_READY_R
			// EVENT_DOWN_4_READY_R
			
			if (Is_End())
			{
				for (uint8_t i = 0; i < 8; i++)
				{
					if (lift_event[i].Is_Trig())
					{
						if (i < 4)
							la = LIFT_UP;
						else
							la = LIFT_DOWN;
						
						if ((i % 2) == 0)
							lh = LIFT_20;
						else
							lh = LIFT_40;
							
						 if ((i % 4) < 2)
							ld = LIFT_L;  // 左
						else
							ld = LIFT_R;  // 右
						
						lift_trig = true;
						break;
					}
				}
			}
			
			Lift(la, lh, ld, lift_trig);
			
			if (lift_trig) lift_trig = false;
		}
		
    private:
		void Set_wheel_Vel(float vel);
	
		void Set_Front_Lift_Pos(float pos)
		{
			if (d == LIFT_L)
				L_lift.Set_Pos(pos * L_LIFT_POL);
			else
				R_lift.Set_Pos(pos * R_LIFT_POL);
		}
	
		void Set_Back_Lift_Pos(float pos)
		{
			if (d == LIFT_L)
				R_lift.Set_Pos(pos * R_LIFT_POL);
			else
				L_lift.Set_Pos(pos * L_LIFT_POL);
		}
		
		float Get_Front_Lift_Pos()
		{
			if (d == LIFT_L)
				return L_lift.Get_Pos() * L_LIFT_POL;
			else
				return R_lift.Get_Pos() * R_LIFT_POL;
		}
		
		float Get_Back_Lift_Pos()
		{
			if (d == LIFT_L)
				return R_lift.Get_Pos() * R_LIFT_POL;
			else
				return L_lift.Get_Pos() * L_LIFT_POL;
		}
		
		
		void Set_Front_Lift_Td(float r, float v2_max)
		{
			if (d == LIFT_L)
				L_lift.pid_pos.Set_Td(r, v2_max);
			else
				R_lift.pid_pos.Set_Td(r, v2_max);
		}
		
		void Set_Back_Lift_Td(float r, float v2_max)
		{
			if (d == LIFT_L)
				R_lift.pid_pos.Set_Td(r, v2_max);
			else
				L_lift.pid_pos.Set_Td(r, v2_max);
		}
		
		void Chassis_Stop()
		{
			chassis.Force_Lin_Vel_Zero(1);
		}
		
		void Chassis_Start()
		{
			chassis.Unforce_Lin_Vel_Zero(1);
		}
		
		void Chassis_Stop_Spin()
		{
			chassis.Force_Ang_Vel_Zero(1);
		}
			
		void Chassis_Start_Spin()
		{
			chassis.Unforce_Ang_Vel_Zero(1);
		}
		
		bool Get_Senser_Value(uint8_t n)
		{
			if (n > 6 || n == 0) return false; 
			
			if (d == LIFT_L)
				return !senser_value[n - 1];
			else
				return !senser_value[7 - n - 1];
		}
		
		
		
		void Reset_Pos()
		{
			qeo.Set_X(0);
			qeo.Set_Y(0);
		}
		
		
		float Get_Dis()
		{
			float dis = qeo.Y();
			
			if (d == LIFT_L)
			{
				return dis;
			}
			else
			{
				return -dis;
			}
		}

	
		float up_pos;
		float down_pos;
		
		LiftDir d;
		LiftState state;
		
		
		bool senser_value[6];
	
		motor::Motor& L_lift;
		motor::Motor& R_lift;
		
		motor::Motor& L_wheel;
		motor::Motor& R_wheel;
		
		chassis::Chassis& chassis;
		fusion::QEO& qeo;
		
		/*---------------------------*/
		
		path::Event3 lift_event[8];
		
		bool lift_trig;
		LiftAction la;
		LiftHeigth lh;
		LiftDir ld;
    };
}
#endif
