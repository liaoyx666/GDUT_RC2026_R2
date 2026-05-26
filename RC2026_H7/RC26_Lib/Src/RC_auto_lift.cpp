#include "RC_auto_lift.h"


namespace chassis
{
	AutoLift::AutoLift(LiftChassis& lift_)
	 : lift_event{
		path::Event3(5 , 0.1f, false, false),		// EVENT_UP_2_READY_L
		path::Event3(6 , 0.3f, false, false),     // EVENT_UP_4_READY_L
		path::Event3(7 , 0.1f, false, false),     // EVENT_UP_2_READY_R
		path::Event3(8 , 0.3f, false, false),     // EVENT_UP_4_READY_R
		path::Event3(9 , 0.1f, false, false),     // EVENT_DOWN_2_READY_L
		path::Event3(10, 0.1f, false, false),     // EVENT_DOWN_4_READY_L
		path::Event3(11, 0.1f, false, false),     // EVENT_DOWN_2_READY_R
		path::Event3(12, 0.1f, false, false)      // EVENT_DOWN_4_READY_R
	}, lift(lift_)
	{
		
		lift_trig = false;
		
	}
	
	
}
