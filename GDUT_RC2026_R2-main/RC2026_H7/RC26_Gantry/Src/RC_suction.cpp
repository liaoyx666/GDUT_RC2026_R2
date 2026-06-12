#include "RC_suction.h"


namespace gantry
{
	Suction::Suction(GPIO_TypeDef* port, uint16_t pin)
	{
		port_ = port;
		pin_ = pin;
	}

	
			
}