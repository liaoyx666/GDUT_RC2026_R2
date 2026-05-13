#pragma once 
#ifdef __cplusplus
#include "gpio.h"

class Suction
{
public:
		Suction(GPIO_TypeDef* port, uint16_t pin);
    void On();  
    void Off(); 

private:
		GPIO_TypeDef* port_;
		int16_t pin_;
    bool state_ = false;
};


#endif