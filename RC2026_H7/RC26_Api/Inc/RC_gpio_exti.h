#pragma once
#include "gpio.h"

#define MAX_GPIO_EXTI_NUM 20

#ifdef __cplusplus
namespace gpio
{
	class GpioExti
    {
    public:
		GpioExti(uint16_t gpio_pin_);
		virtual ~GpioExti() {}
		
		static void All_EXTI_Prosess(uint16_t gpio_pin_);
		
    protected:
		virtual void EXTI_Prosess() = 0;
		
    private:
		uint16_t gpio_pin;
	
		static uint16_t gpio_exti_num;
		static GpioExti* gpio_exti_list[MAX_GPIO_EXTI_NUM];
    };
}
#endif
