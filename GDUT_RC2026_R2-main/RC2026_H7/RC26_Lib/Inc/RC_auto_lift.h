#pragma once
#include "RC_event3.h"
#include "RC_traj_track3.h"
#include "RC_data_pool.h"
#include "RC_lift_chassis.h"

#ifdef __cplusplus

namespace chassis
{
	class AutoLift
    {
    public:
		AutoLift(LiftChassis& lift_);
		~AutoLift() = default;

		inline void Auto_Lift()
		{
			// EVENT_UP_2_READY_L
			// EVENT_UP_4_READY_L
			// EVENT_UP_2_READY_R
			// EVENT_UP_4_READY_R
			// EVENT_DOWN_2_READY_L
			// EVENT_DOWN_4_READY_L
			// EVENT_DOWN_2_READY_R
			// EVENT_DOWN_4_READY_R
			
			if (lift.Is_End())
			{
				for (uint8_t i = 0; i < 8; i++)
				{
					if (lift_event[i].Is_Trig())
					{
						if (i < 4)
							la = LIFT_UP;
						else
							la = LIFT_DOWN;
						
						if ((i % 2) == 0)
							lh = LIFT_20;
						else
							lh = LIFT_40;
							
						 if ((i % 4) < 2)
							ld = LIFT_L;  // 左
						else
							ld = LIFT_R;  // 右
						
						lift_trig = true;
						break;
					}
				}
			}
			
			lift.Lift(la, lh, ld, lift_trig);
			
			if (lift_trig) lift_trig = false;
		}
		
    private:
		path::Event3 lift_event[8];
		
		LiftChassis& lift;
		bool lift_trig;
		LiftAction la;
		LiftHeigth lh;
		LiftDir ld;
    };
}
#endif
