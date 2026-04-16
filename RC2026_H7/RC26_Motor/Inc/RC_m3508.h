#pragma once
#include "RC_dji_motor.h"

#ifdef __cplusplus
namespace motor
{
	class M3508 : public DjiMotor
	{
	public:
		M3508(uint8_t id_, can::Can &can_, tim::Tim *tim_, float gear_ratio_ = 3591.f / 187.f, bool is_reset_pos_angle = false);
		~M3508() = default;

	protected:
		void Dji_Id_Init(uint8_t id_) override;
	};
	
	
	/**
    * @brief M3508双电机并联控制
    * @param _m : 主电机参数， _s : 从电机参数， pol_s : 从电机极性
    */
	class M3508D : public M3508
    {
    public:
		M3508D(
			uint8_t id_m, can::Can &can_m, tim::Tim *tim_m, 
			uint8_t id_s, can::Can &can_s, tim::Tim *tim_s, 
			float gear_ratio_ = 3591.f / 187.f, MotorPol pol_s = POL_REV, bool is_reset_pos_angle = false
		);
		~M3508D() = default;
		
    protected:
		void Tim_It_Process() override;
	
    private:
		M3508 slave;
		MotorPol pol; /*从电机极性*/
    };
}
#endif
