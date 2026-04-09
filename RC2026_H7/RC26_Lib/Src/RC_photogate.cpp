#include "RC_photogate.h"

namespace photogate
{
	PhoGateRepos::PhoGateRepos(motor::DjiMotor& motor_, bool& is_reposition_, float angle_, uint16_t gpio_pin_, float min_rpm_)
	: gpio::GpioExti(gpio_pin_), motor(motor_), is_reposition(is_reposition_)
	{
		min_rpm = min_rpm_;
		
		if (angle_ < 0 || angle_ > TWO_PI) angle_ = 0;

		angle = angle_;
	}
	
	void PhoGateRepos::EXTI_Prosess()
	{
		if (!is_reposition && motor.Get_Out_Rpm() > min_rpm) /*不总是校准*/
		{
  			motor.Reset_Out_Angle(angle);
			is_reposition = true;
		}
	}
}