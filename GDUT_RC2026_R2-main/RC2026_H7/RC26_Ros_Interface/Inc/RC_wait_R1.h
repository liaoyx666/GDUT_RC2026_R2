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
		WAIT_R1_STOP,
		WAIT_R1_WAIT_EMPTY,
	};
	
	
	class WaitR1 : public cdc::CDCHandler
    {
    public:
		WaitR1(cdc::CDC &cdc_, uint8_t rx_id_, chassis::Chassis& chassis_, path::HeadCtrl& head_ctrl_);
		~WaitR1() = default;
			
		void Wait_R1()
		{
			if (is_empty && state != WAIT_R1_WAIT_EMPTY)
			{
				Close();
			}
			
			switch (state)
			{
				case WAIT_R1_WAIT_TRIG:
				{
					chassis.Unforce_Lin_Vel_Zero(3);
					
					if (wait_event_L.Is_Trig())
					{
						state = WAIT_R1_STOP;
						is_L = true;
					}
					else if (wait_event_R.Is_Trig())
					{
						state = WAIT_R1_STOP;
						is_L = false;
					}
					break;
				}
				
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
					
					// 开启说明可以通行
					if (is_empty)
					{
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
		
		void Close()
		{
			uint8_t send = 0;
			cdc->CDC_Send_Pkg(id, &send, 1, 0); /*不等待*/
		}
		

		bool is_L;
		uint8_t id;
		
		chassis::Chassis& chassis;
		path::HeadCtrl& head_ctrl;
		
		path::Event3 wait_event_L;
		path::Event3 wait_event_R;
    };
}
#endif
