#pragma once
#include "RC_dji_motor.h"

#ifdef __cplusplus
namespace motor
{
	class M2006 : public DjiMotor
	{
	public:
		M2006(uint8_t id_, can::Can &can_, tim::Tim *tim_, float gear_ratio_ = 36.f, bool is_reset_pos_angle = false);
		virtual ~M2006() {}

	protected:
		void Dji_Id_Init(uint8_t id_) override;
	};
	
	
	/**
    * @brief M2006双电机并联控制
    * @param _m : 主电机参数， _s : 从电机参数， pol_s : 从电机极性
    */
	class M2006D : public M2006
    {
    public:
		M2006D(
			uint8_t id_m, can::Can &can_m, tim::Tim *tim_m, 
			uint8_t id_s, can::Can &can_s, tim::Tim *tim_s, 
			float gear_ratio_ = 36.f, MotorPol pol_s = POL_REV, bool is_reset_pos_angle = false
		);
		virtual ~M2006D() {}
		
    protected:
		void Tim_It_Process() override;
	
    private:
		M2006 slave;
		MotorPol pol; /*从电机极性*/
    };
}
#endif
