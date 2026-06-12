#pragma once
#include "RC_event3.h"
#include "RC_gripper.h"
#include "RC_timer.h"

#ifdef __cplusplus
namespace gantry
{
	class Dock
    {
    public:
		Dock(Gripper& grip_);
		~Dock() = default;

		void Auto_Dock();
	
    private:
		path::Event3 dock_event;
		uint8_t state;
		uint32_t last_time;
	
		Gripper& grip;
    };
}
#endif
