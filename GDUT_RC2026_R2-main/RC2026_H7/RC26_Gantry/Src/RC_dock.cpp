#include "RC_dock.h"

namespace gantry
{
	Dock::Dock(Gripper& grip_) : dock_event(21, 0.1f, true, true), grip(grip_)
	{
		state = 0;
		last_time = 0;
	}
	
	void Dock::Auto_Dock()
	{
		switch (state)
		{
			case 0:
			{
				if (dock_event.Is_Trig())
				{
					state = 1;
					last_time = timer::Timer::Get_TimeStamp();
				}
				
				break;
			}
			
			case 1:
			{
				grip.Open();
				
				if (timer::Timer::Get_DeltaTime(last_time) > 3000000)
				{
					state = 0;
					dock_event.Finish();
				}
				
				break;
			}
			
			default:
			{
				state = 0;
				
				break;
			}
		}
	}
}