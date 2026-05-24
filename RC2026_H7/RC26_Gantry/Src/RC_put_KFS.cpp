#include "RC_put_KFS.h"
#include "RC_data_pool.h"

namespace gantry
{
	constexpr float PUTKFS_POS_THRESTHOLD_BIG = 0.03;
	constexpr float PUTKFS_POS_THRESTHOLD_SMALL = 0.005;
	constexpr float PUTKFS_ANG_THRESTHOLD_BIG = 0.03;
	constexpr float PUTKFS_ANG_THRESTHOLD_SMALL = 0.01;
	
	constexpr float PUTKFS_2L_Z = 0.86;
	constexpr float PUTKFS_3L_Z = 0.86;
	
	constexpr float PUTKFS_GET_KFS_LOW_Z = 0.1;
	constexpr float PUTKFS_GET_KFS_HIGH_Z = 0.45;
	
	PutKFS::PutKFS(Gantry& gan_, Suction& suck_)
	 : 	user(gan_), 
		suck(suck_), 
		put_event{
			path::Event3(17, 1.f, false, false),		//EVENT_PUT_KFS_2L_READY
			path::Event3(18, 1.f, false, false),   		//EVENT_PUT_KFS_3L_READY
			path::Event3(19, 0.03f, true, true) 	 	//EVENT_PUT_KFS_PUT
		}
	{
		phase = PUTKFS_RESET;
		last_time = 0;
		ready_trig = false;
	}
	
	void PutKFS::Auto_Put_KFS()
	{
		switch (phase)
		{
			case PUTKFS_RESET:
			{
				if (put_event[0].Is_Trig())
				{
					ready_trig = true;
					put_z = PUTKFS_2L_Z;
				}
				else if (put_event[1].Is_Trig())
				{
					ready_trig = true;
					put_z = PUTKFS_3L_Z;
				}
				
				if (ready_trig)
				{
					if (!user.Take_Control()) return;
					
					ready_trig = false;
					
					if (data::KFS_Num() == 0) return;
					else if (data::KFS_Num() == 1) get_z = PUTKFS_GET_KFS_LOW_Z;
					else get_z = PUTKFS_GET_KFS_HIGH_Z;
					
					suck.Off();
					
					get_state = PUTKFS_GET_STRETCH;
					put_state = PUTKFS_PUT_CLOSE_TO;
					
					phase = PUTKFS_GET_PHASE;
				}
				break;
			}
			
			case PUTKFS_GET_PHASE:
			{
				if (
					Get_KFS_Phase() &&
					put_event[2].Is_Trig()
				)
				{
					phase = PUTKFS_PUT_PHASE;
				}
				break;
			}
			
			case PUTKFS_PUT_PHASE:
			{
				if (Put_KFS_Phase())
				{
					put_event[2].Finish();

					user.Set_Defualt_Td();
					user.Set_Reset_Pos();
				
					data::KFS_Sub_One();
					user.Give_Control();
					
					phase = PUTKFS_RESET;
				}
				break;
			}
			
			default:
			{
				phase = PUTKFS_RESET;
				break;
			}
		}
	}
	

	
	constexpr float PUTKFS_GET_KFS_STRETCH_X = 0.05;
	constexpr float PUTKFS_GET_KFS_WITHDRAW_X = 0.0;
	constexpr float PUTKFS_GET_KFS_OUT_X = 0.07;
	
	bool PutKFS::Get_KFS_Phase()
	{
		switch (get_state)
		{
			case PUTKFS_GET_STRETCH:
			{
				user.Set_X(PUTKFS_GET_KFS_STRETCH_X);
				user.Set_Y(0);
				user.Set_P(0);
				
				get_state = PUTKFS_GET_STRETCH_CHECK;
				break;
			}
			
			case PUTKFS_GET_STRETCH_CHECK:
			{
				if(
					user.Get_X() > (PUTKFS_GET_KFS_STRETCH_X - PUTKFS_POS_THRESTHOLD_SMALL)
				)
				{
					get_state = PUTKFS_GET_TO_KFS;
				}
				break;
			}
			
			case PUTKFS_GET_TO_KFS:
			{
				user.Set_Z(get_z);
				
				get_state = PUTKFS_GET_TO_KFS_CHECK;
				break;
			}
			
			
			case PUTKFS_GET_TO_KFS_CHECK:
			{
				if (
					fabsf(user.Get_Y() - 0) < PUTKFS_POS_THRESTHOLD_SMALL && 
					fabsf(user.Get_Z() - get_z) < PUTKFS_POS_THRESTHOLD_SMALL && 
					fabsf(user.Get_P() - 0) < PUTKFS_ANG_THRESTHOLD_SMALL
				)
				{
					last_time = timer::Timer::Get_TimeStamp();
					get_state = PUTKFS_GET_WITHDRAW_SUCK;
				}
				break;
			}
			
			case PUTKFS_GET_WITHDRAW_SUCK:
			{
				suck.On();
				
				user.Set_X(PUTKFS_GET_KFS_WITHDRAW_X);
				
				get_state = PUTKFS_GET_WITHDRAW_SUCK_CHECK;
				break;
			}
			
			case PUTKFS_GET_WITHDRAW_SUCK_CHECK:
			{
				if (
					fabsf(user.Get_X() - PUTKFS_GET_KFS_WITHDRAW_X) < PUTKFS_POS_THRESTHOLD_SMALL &&
					timer::Timer::Get_DeltaTime(last_time) > 1000000
				)
				{
					get_state = PUTKFS_GET_KFS_OUT;
				}
				break;
			}
			
			case PUTKFS_GET_KFS_OUT:
			{
				user.Set_X_Td(500, 837.76);
				user.Set_P_Td(4, 5);
				
				user.Set_X(PUTKFS_GET_KFS_OUT_X);
				user.Set_P(HALF_PI);
				
				return true;
				break;
			}
			
			default:
			{
				get_state = PUTKFS_GET_STRETCH;
				break;
			}
		}
		
		return false;
	}
	
	
	
	constexpr float PUTKFS_PUT_KFS_CLOSE_X = 0.2;
	constexpr float PUTKFS_PUT_KFS_IN_X = 0.43;
	constexpr float PUTKFS_PUT_KFS_DOWN_DELTA_Z = 0.03;
	constexpr float PUTKFS_PUT_KFS_OUT_X = 0.32;
	
	constexpr float PUTKFS_PUT_KFS_CLOSE_P = 2.1;
	constexpr float PUTKFS_PUT_KFS_CLOSE_DELTA_Z = 0.3;
	
	bool PutKFS::Put_KFS_Phase()
	{
		switch (put_state)
		{
			case PUTKFS_PUT_CLOSE_TO:
			{
				user.Set_X(PUTKFS_PUT_KFS_CLOSE_X);
				user.Set_Z(put_z - PUTKFS_PUT_KFS_CLOSE_DELTA_Z);
				user.Set_P(PUTKFS_PUT_KFS_CLOSE_P);
				
				put_state = PUTKFS_PUT_CLOSE_TO_CHECK;
				break;
			}
			
			case PUTKFS_PUT_CLOSE_TO_CHECK:
			{
				if (
					fabsf(user.Get_Z() - (put_z - PUTKFS_PUT_KFS_CLOSE_DELTA_Z))     < PUTKFS_POS_THRESTHOLD_SMALL &&
					fabsf(user.Get_P() - PUTKFS_PUT_KFS_CLOSE_P)                     < PUTKFS_ANG_THRESTHOLD_SMALL
				)
				{
					put_state = PUTKFS_PUT_IN;
				}
				
				break;
			}
			
			case PUTKFS_PUT_IN:
			{
				user.Set_Z_Td(500.f, 314.16);
				user.Set_P_Td(3, 5);
				
				user.Set_X(PUTKFS_PUT_KFS_IN_X);
				user.Set_Z(put_z);
				user.Set_P(PI);
				
				put_state = PUTKFS_PUT_IN_CHECK;
				break;
			}
			
			
			case PUTKFS_PUT_IN_CHECK:
			{
				if (
					fabsf(user.Get_X() - PUTKFS_PUT_KFS_IN_X) 	  < PUTKFS_POS_THRESTHOLD_SMALL &&
					fabsf(user.Get_Z() - put_z)                   < PUTKFS_POS_THRESTHOLD_SMALL &&
					fabsf(user.Get_P() - PI)                      < PUTKFS_ANG_THRESTHOLD_SMALL
				)
				{
					put_state = PUTKFS_PUT_DOWN;
				}
				break;
			}
			
			case PUTKFS_PUT_DOWN:
			{
				user.Set_Z(put_z - PUTKFS_PUT_KFS_DOWN_DELTA_Z);
				
				put_state = PUTKFS_PUT_DOWN_CHECK;
				break;
			}
			
			case PUTKFS_PUT_DOWN_CHECK:
			{
				if (
					fabsf(user.Get_Z() - (put_z - PUTKFS_PUT_KFS_DOWN_DELTA_Z)) < PUTKFS_POS_THRESTHOLD_SMALL
				)
				{
					last_time = timer::Timer::Get_TimeStamp();
					put_state = PUTKFS_PUT_RELESE;
				}
				break;
			}
			
			case PUTKFS_PUT_RELESE:
			{
				suck.Off();
				
				put_state = PUTKFS_PUT_RELESE_CHECK;
				break;
			}
			
			case PUTKFS_PUT_RELESE_CHECK:
			{
				if (
					timer::Timer::Get_DeltaTime(last_time) > 300000
				)
				{
					return true;
					//put_state = PUTKFS_PUT_OUT;
				}
				break;
			}
			
//			case PUTKFS_PUT_OUT:
//			{
//				user.Set_X(PUTKFS_PUT_KFS_OUT_X);
//				
//				put_state = PUTKFS_PUT_OUT_CHECK;
//				break;
//			}
//			
//			case PUTKFS_PUT_OUT_CHECK:
//			{
//				if (
//					fabsf(user.Get_X() - PUTKFS_PUT_KFS_OUT_X) < PUTKFS_POS_THRESTHOLD_SMALL
//				)
//				{
//					return true;
//				}
//				break;
//			}
			
			default:
			{
				put_state = PUTKFS_PUT_CLOSE_TO;
				break;
			}
		}
		
		return false;
	}
}
