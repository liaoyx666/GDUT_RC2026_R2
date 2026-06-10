#include "RC_wait_R1.h"

namespace ros
{
	WaitR1::WaitR1(cdc::CDC &cdc_, uint8_t rx_id_, chassis::Chassis& chassis_, path::HeadCtrl& head_ctrl_)
	: chassis(chassis_), 
	  wait_event_L(23, 0.09f, false, false), 
<<<<<<< HEAD
	  wait_event_R(23, 0.09f, false, false), 
=======
	  wait_event_R(24, 0.09f, false, false), 
>>>>>>> main
	  cdc::CDCHandler(cdc_, rx_id_), 
	  id(rx_id_), 
	  head_ctrl(head_ctrl_)
	{
		is_empty = false;
		state = WAIT_R1_WAIT_TRIG;
		is_L = false;
<<<<<<< HEAD
=======
		//is_enable = false;
>>>>>>> main
	}
	

}
