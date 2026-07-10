#pragma once
#include "gpio.h"

#ifdef __cplusplus

class StartSwitch
{
public:
	StartSwitch(GPIO_TypeDef* port_, uint16_t pin_);
	~StartSwitch() = default;

	bool Get_Start_Switch() { return HAL_GPIO_ReadPin(port, pin); }
protected:
	
private:
	GPIO_TypeDef* port;
	uint16_t pin;

};

#endif
