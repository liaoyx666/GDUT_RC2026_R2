#include "RC_go.h"

namespace motor
{
	Go::Go(can::Can &can_, tim::Tim &tim_) : can::CanHandler(can_), tim::TimHandler(tim_)
	{
		
	}
}