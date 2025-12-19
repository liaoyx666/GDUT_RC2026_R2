#include "RC_photogate.h"

namespace photogate
{
	PhoGateRepos::PhoGateRepos(motor::Motor* motor_ptr_, bool* is_reposition_, float angle_, uint16_t gpio_pin_, float min_rpm_) : gpio::GpioExti(gpio_pin_)
	{
		motor_ptr = motor_ptr_;
		
		min_rpm = min_rpm_;
		
		is_reposition = is_reposition_;
		
		if (angle_ < 0 || angle_ > TWO_PI) angle_ = 0;

		angle = angle_;
	}
	
	void PhoGateRepos::EXTI_Prosess()
	{
		if (motor_ptr != nullptr && motor_ptr->Get_Out_Rpm() > min_rpm)
		{
  			motor_ptr->Reset_Out_Angle(angle);
			*is_reposition = true;
		}
	}
}