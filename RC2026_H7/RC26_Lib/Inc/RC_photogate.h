#pragma once
#include "RC_gpio_exti.h"
#include "RC_motor.h"

#ifdef __cplusplus
namespace photogate
{
	// 上拉输入
	class PhoGateRepos : gpio::GpioExti
    {
    public:
		PhoGateRepos(motor::Motor* motor_ptr_, bool* is_reposition_, float angle_, uint16_t gpio_pin_, float min_rpm_ = 0);
		virtual ~PhoGateRepos() {}
		
    protected:
		
    private:
		motor::Motor* motor_ptr;
		bool* is_reposition;
		float angle;
		float min_rpm = 0;
	
		void EXTI_Prosess() override;
    };
}
#endif
