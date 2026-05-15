#pragma once 

#ifdef __cplusplus
#include "gpio.h"
namespace gantry
{
	class Suction
	{
	public:
		Suction(GPIO_TypeDef* port, uint16_t pin);
	
		constexpr void On()
		{
			HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
			state_ = true;
		}

		constexpr void Off()
		{
			HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
			state_ = false;
		}

	private:
		GPIO_TypeDef* port_;
		int16_t pin_;
		bool state_ = false;
	};
}
#endif