#pragma once
#ifdef __cplusplus

#include "RC_event3.h"
#include "RC_chassis.h"
#include "RC_lift_chassis.h"

namespace combine
{
	
	class Combine
    {
    public:
		Combine(chassis::Chassis& chassis_, chassis::LiftChassis& lift_);
		~Combine() = default;
	
		void Auto_Combine();
    private:
		path::Event3 combine_event;
		chassis::Chassis& chassis;
		chassis::LiftChassis& lift;
	
		bool is_combine;
    };

}
#endif
