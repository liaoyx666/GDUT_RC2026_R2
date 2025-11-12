#include "RC_gpio_exti.h"


extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    gpio::GpioExti::All_EXTI_Prosess(GPIO_Pin);
}

namespace gpio
{
	uint16_t GpioExti::gpio_exti_num = 0;
	GpioExti* GpioExti::gpio_exti_list[MAX_GPIO_EXTI_NUM] = {nullptr};
	
	
	GpioExti::GpioExti(uint16_t gpio_pin_)
	{
		if (gpio_exti_num < MAX_GPIO_EXTI_NUM)
		{
			gpio_pin = gpio_pin_;
			gpio_exti_list[gpio_exti_num] = this;
			gpio_exti_num++;
		}
	}
	
	void GpioExti::All_EXTI_Prosess(uint16_t gpio_pin_)
	{
		for (uint8_t i = 0; i < gpio_exti_num; i++)
		{
			if (gpio_exti_list[i] != nullptr && gpio_exti_list[i]->gpio_pin == gpio_pin_)
			{
				gpio_exti_list[i]->EXTI_Prosess();
				break;
			}
		}
	}
}