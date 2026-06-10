#pragma once
#ifdef __cplusplus
#include "RC_chassis.h"
#include "RC_event3.h"
#include "RC_cdc.h"
#include "RC_head_ctrl.h"

namespace ros
{
	enum WaitR1State : uint8_t
	{
		WAIT_R1_WAIT_TRIG = 0,
<<<<<<< HEAD
=======
		WAIT_R1_STOP,
>>>>>>> main
		WAIT_R1_WAIT_EMPTY,
	};
	
	
	class WaitR1 : public cdc::CDCHandler
    {
    public:
		WaitR1(cdc::CDC &cdc_, uint8_t rx_id_, chassis::Chassis& chassis_, path::HeadCtrl& head_ctrl_);
		~WaitR1() = default;
			
		void Wait_R1()
		{
<<<<<<< HEAD
=======
			if (is_empty && state != WAIT_R1_WAIT_EMPTY)
			{
				Close();
			}
			
>>>>>>> main
			switch (state)
			{
				case WAIT_R1_WAIT_TRIG:
				{
					chassis.Unforce_Lin_Vel_Zero(3);
					
					if (wait_event_L.Is_Trig())
					{
<<<<<<< HEAD
						state = WAIT_R1_WAIT_EMPTY;
						is_empty = false;
=======
						state = WAIT_R1_STOP;
>>>>>>> main
						is_L = true;
					}
					else if (wait_event_R.Is_Trig())
					{
<<<<<<< HEAD
						state = WAIT_R1_WAIT_EMPTY;
						is_empty = false;
=======
						state = WAIT_R1_STOP;
>>>>>>> main
						is_L = false;
					}
					break;
				}
				
<<<<<<< HEAD
				case WAIT_R1_WAIT_EMPTY:
				{
					chassis.Force_Lin_Vel_Zero(3);
					
=======
				case WAIT_R1_STOP:
				{
					chassis.Force_Lin_Vel_Zero(3);
					
					// 等待上次关闭
					if (!is_empty)
					{
						state = WAIT_R1_WAIT_EMPTY;
					}
					
					break;
				}
				
				case WAIT_R1_WAIT_EMPTY:
				{
>>>>>>> main
					// yaw对齐后再请求数据
					if (fabsf(head_ctrl.Get_Delta_Yaw()) < (4.f / 180.f * PI))
					{
						if (is_L)
						{
							Request_L();
						}
						else
						{
							Request_R();
						}
					}
					
<<<<<<< HEAD
					if (is_empty)
					{
						is_empty = false;
=======
					// 开启说明可以通行
					if (is_empty)
					{
>>>>>>> main
						state = WAIT_R1_WAIT_TRIG;
					}
					break;
				}
				
				default:
				{
					break;
				}
			}
		}
    private:
		void CDC_Receive_Process(uint8_t *buf, uint16_t len) override
		{
<<<<<<< HEAD
			if (len == 1 && buf[0] == 1)
			{
				is_empty = true;
				
				// 关闭
				uint8_t send = 0;
				cdc->CDC_Send_Pkg(id, &send, 1, 0); /*不等待*/
			}
		}
	
		bool is_empty;
	
=======
			if (len == 1)
			{
				if (buf[0] == 1)
				{
					is_empty = true;
				}
				else if (buf[0] == 0)
				{
					is_empty = false;
				}
			}
		}

		bool is_empty;
>>>>>>> main
		WaitR1State state;
	
		void Request_L()
		{
			uint8_t send = 1;
			cdc->CDC_Send_Pkg(id, &send, 1, 0); /*不等待*/
		}
		
		void Request_R()
		{
			uint8_t send = 2;
			cdc->CDC_Send_Pkg(id, &send, 1, 0); /*不等待*/
		}
		
<<<<<<< HEAD
		bool is_L;
		uint8_t id;
		chassis::Chassis& chassis;
		path::HeadCtrl& head_ctrl;
=======
		void Close()
		{
			uint8_t send = 0;
			cdc->CDC_Send_Pkg(id, &send, 1, 0); /*不等待*/
		}
		

		bool is_L;
		uint8_t id;
		
		chassis::Chassis& chassis;
		path::HeadCtrl& head_ctrl;
		
>>>>>>> main
		path::Event3 wait_event_L;
		path::Event3 wait_event_R;
    };
}
#endif
