#pragma once
#include "RC_motor.h"
#include "gpio.h"
#include "RC_chassis.h"
#include "RC_traj_track3.h"

#ifdef __cplusplus
namespace chassis
{
	#define ZERO_POS 	( 0.f)
	#define UP_20_POS 	( 295.f + ZERO_POS)
	#define UP_40_POS 	( 590.f + ZERO_POS)
	#define DOWN_20_POS (-295.f + ZERO_POS)
	#define DOWN_40_POS (-590.f + ZERO_POS)
	#define RESET_POS 	( 100.f)

	#define L_LIFT_POL 	(-1.f)
	#define R_LIFT_POL 	(-1.f)
	
	const static GPIO_TypeDef* SENSER_GPIO_PORT[6] =
	{
		GPIOE,
		GPIOD,
		GPIOG,
		GPIOG,
		GPIOD,
		GPIOE
	};
	
	constexpr static uint16_t SENSER_GPIO_PIN[6] =
	{
		GPIO_PIN_3,
		GPIO_PIN_13,
		GPIO_PIN_1,
		GPIO_PIN_0,
		GPIO_PIN_12,
		GPIO_PIN_2
	};
		
	enum LiftState : uint8_t
	{
		LIFT_RESET = 0, 	/* 默认状态 */
		
		LIFT_UP_READY, 		/* 准备上台阶 */
		LIFT_UP_RISE, 		/* 抬升 */
		LIFT_UP_FORWARD, 	/* 前进 */
		LIFT_UP_WITHDRAW, 	/* 收回机构 */
		
		LIFT_DOWN_READY, 	/* 准备下台阶 */
		LIFT_DOWN_STRETCH, 	/* 伸出机构 */
		LIFT_DOWN_FORWARD, 	/* 前进 */
		LIFT_DOWN_FALL, 	/* 下降 */
		LIFT_DOWN_WITHDRAW,	/* 收回机构 */
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
			chassis::Chassis* chassis_, path::TrajTrack3* track_
		);
		~LiftChassis() = default;
		
		void Lift(LiftAction a_, LiftHeigth h_, LiftDir d_, bool trig);
		bool Is_End() const {return (state == LIFT_RESET);}
	
	
    private:
		void Set_wheel_Vel(float vel);
	
	
		inline void Set_Front_Lift_Pos(float pos)
		{
			if (d == LIFT_L)
				L_lift.Set_Pos(pos * L_LIFT_POL);
			else
				R_lift.Set_Pos(pos * R_LIFT_POL);
		}
	
		inline void Set_Back_Lift_Pos(float pos)
		{
			if (d == LIFT_L)
				R_lift.Set_Pos(pos * R_LIFT_POL);
			else
				L_lift.Set_Pos(pos * L_LIFT_POL);
		}
		
		inline float Get_Front_Lift_Pos()
		{
			if (d == LIFT_L)
				return L_lift.Get_Pos() * L_LIFT_POL;
			else
				return R_lift.Get_Pos() * R_LIFT_POL;
		}
		
		inline float Get_Back_Lift_Pos()
		{
			if (d == LIFT_L)
				return R_lift.Get_Pos() * R_LIFT_POL;
			else
				return L_lift.Get_Pos() * L_LIFT_POL;
		}
		
		
		inline void Set_Front_Lift_Td(float r, float v2_max)
		{
			if (d == LIFT_L)
				L_lift.pid_pos.Set_Td(r, v2_max);
			else
				R_lift.pid_pos.Set_Td(r, v2_max);
		}
		
		inline void Set_Back_Lift_Td(float r, float v2_max)
		{
			if (d == LIFT_L)
				R_lift.pid_pos.Set_Td(r, v2_max);
			else
				L_lift.pid_pos.Set_Td(r, v2_max);
		}
		
		inline void Chassis_Stop()
		{
			if (chassis)
				chassis->Force_Lin_Vel_Zero();
			
			if (track)
				track->Force_Tan_Vel_Zero();
		}
		
		inline void Chassis_Start()
		{
			if (chassis)
				chassis->Unforce_Lin_Vel_Zero();

			if (track)
				track->Unforce_Tan_Vel_Zero();
		}
		
		inline bool Get_Senser_Value(uint8_t n)
		{
			if (n > 6 && n == 0) return false; 
			
			if (d == LIFT_L)
				return senser_value[n - 1];
			else
				return senser_value[7 - n - 1];
		}
	
		LiftDir d;
		
		float up_pos;
		float down_pos;
		
		LiftState state;
		bool senser_value[6];
	
		motor::Motor& L_lift;
		motor::Motor& R_lift;
		
		motor::Motor& L_wheel;
		motor::Motor& R_wheel;
		
		chassis::Chassis* chassis;
		path::TrajTrack3* track;
    };
}
#endif
