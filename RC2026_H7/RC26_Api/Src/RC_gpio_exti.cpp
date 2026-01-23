#include "RC_gpio_exti.h"


extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    gpio::GpioExti::All_EXTI_Prosess(GPIO_Pin);
}

namespace gpio
{
	GpioExti* GpioExti::gpio_exti_list[MAX_GPIO_EXTI_NUM] = {nullptr};
	
	
	GpioExti::GpioExti(uint16_t gpio_pin_)
	{
		// 计算 x 二进制中尾随零的个数（从最低位开始）（开平方根）
		uint8_t pin = __builtin_ctz(gpio_pin_);
		
		if (pin < MAX_GPIO_EXTI_NUM)
		{
			gpio_pin = gpio_pin_;
			gpio_exti_list[pin] = this;
		}
	}
	
	void GpioExti::All_EXTI_Prosess(uint16_t gpio_pin_)
	{
		// 计算 x 二进制中尾随零的个数（从最低位开始）（开平方根）
		uint8_t pin = __builtin_ctz(gpio_pin_);
		
		if (pin < MAX_GPIO_EXTI_NUM)
		{
			if (gpio_exti_list[pin] != nullptr)
			{
				gpio_exti_list[pin]->EXTI_Prosess();
			}
		}
	}
}