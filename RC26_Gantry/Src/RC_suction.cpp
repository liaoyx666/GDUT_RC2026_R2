#include "RC_suction.h"
Suction::Suction(GPIO_TypeDef* port, uint16_t pin)
{
		port_ = port;
    pin_ = pin;
}

void Suction::On()
{
    HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET);
    state_ = true;
}

void Suction::Off()
{
    HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
    state_ = false;
}
		