#pragma once
#include "RC_motor.h"
#include "RC_pid.h"
#include "RC_can.h"
#include "RC_tim.h"

#ifdef __cplusplus
namespace motor
{
	class Go : public Motor, public can::CanHandler, public tim::TimHandler
    {
    public:
		Go(can::Can &can_, tim::Tim &tim_);
		virtual ~Go() {}
			
    protected:
		
    private:
		
    };
}
#endif
