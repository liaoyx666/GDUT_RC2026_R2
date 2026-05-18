#include "RC_put_KFS.h"
#include "RC_data_pool.h"

namespace gantry
{
	constexpr float PUTKFS_POS_THRESTHOLD_BIG = 0.02;
	constexpr float PUTKFS_POS_THRESTHOLD_SMALL = 0.005;
	constexpr float PUTKFS_ANG_THRESTHOLD = 0.01;
	
	constexpr float PUTKFS_2L_Z = 0.86;
	constexpr float PUTKFS_3L_Z = 0.86;
	
	constexpr float PUTKFS_GET_KFS_LOW_Z = 0.1;
	constexpr float PUTKFS_GET_KFS_HIGH_Z = 0.45;
	
	PutKFS::PutKFS(Gantry& gan_, Suction& suck_)
	 : 	user(gan_), 
		suck(suck_), 
		put_event{
			path::Event3(17, 0.07f, true, true),	//EVENT_PUT_KFS_2L
			path::Event3(18, 0.07f, true, true),    //EVENT_PUT_KFS_3L
		}
	{
		phase = PUTKFS_RESET;
		last_time = 0;
		put_trig = false;
	}
	
	void PutKFS::Auto_Put_KFS()
	{
		if (put_event[0].Is_Trig())
		{
			put_trig = true;
			h = PUTKFS_2L;
		}
		else if (put_event[1].Is_Trig())
		{
			put_trig = true;
			h = PUTKFS_3L;
		}
		
		Put_KFS(h, put_trig);
		if (put_trig) put_trig = false;
	}
	
	
	void PutKFS::Put_KFS(PutKFSHeight h_, bool trig_)
	{
		switch (phase)
		{
			case PUTKFS_RESET:
			{
				if (trig_)
				{
					if (h_ == PUTKFS_2L) put_z = PUTKFS_2L_Z;
					else put_z = PUTKFS_3L_Z;
					
					if (data::KFS_Num() == 0) return;
					else if (data::KFS_Num() == 1) get_z = PUTKFS_GET_KFS_LOW_Z;
					else get_z = PUTKFS_GET_KFS_HIGH_Z;
					
					if (!user.Take_Control()) return;
					
					suck.Off();
					
					get_state = PUTKFS_GET_STRETCH;
					put_state = PUTKFS_PUT_CLOSE_TO;
					phase = PUTKFS_GET_PHASE;
				}
				break;
			}
			
			case PUTKFS_GET_PHASE:
			{
				if (Get_KFS_Phase())
				{
					phase = PUTKFS_PUT_PHASE;
				}
				break;
			}
			
			case PUTKFS_PUT_PHASE:
			{
				if (Put_KFS_Phase())
				{
					user.Set_Defualt_Td();
					user.Set_Reset_Pos();
					user.Give_Control();
					data::KFS_Sub_One();
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
	constexpr float PUTKFS_GET_KFS_OUT_X = 0.05;
	

	
	bool PutKFS::Get_KFS_Phase()
	{
		switch (get_state)
		{
			case PUTKFS_GET_STRETCH:
			{
				user.Set_X(PUTKFS_GET_KFS_STRETCH_X);
				user.Set_Y(0);
				user.Set_P(0);
				
				if(
					user.Get_X() > PUTKFS_GET_KFS_STRETCH_X - PUTKFS_POS_THRESTHOLD_BIG
				)
				{
					get_state = PUTKFS_GET_TO_KFS;
				}
				break;
			}
			
			case PUTKFS_GET_TO_KFS:
			{
				user.Set_Z(get_z);
				
				if (
					fabsf(user.Get_Y() - 0) < PUTKFS_POS_THRESTHOLD_SMALL && 
					fabsf(user.Get_Z() - get_z) < PUTKFS_POS_THRESTHOLD_SMALL && 
					fabsf(user.Get_P() - 0) < PUTKFS_POS_THRESTHOLD_SMALL
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
	constexpr float PUTKFS_PUT_KFS_IN_X = 0.45;
	constexpr float PUTKFS_PUT_KFS_DOWN_DELTA_Z = 0.05;
	constexpr float PUTKFS_PUT_KFS_OUT_X = 0.32;
	
	constexpr float PUTKFS_PUT_KFS_CLOSE_P = 2.3;
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
				
				if (
					fabsf(user.Get_Z() - (put_z - PUTKFS_PUT_KFS_CLOSE_DELTA_Z))     < PUTKFS_POS_THRESTHOLD_SMALL &&
					fabsf(user.Get_P() - PUTKFS_PUT_KFS_CLOSE_P)                     < PUTKFS_ANG_THRESTHOLD
				)
				{
					put_state = PUTKFS_PUT_IN;
				}
				break;
			}
			
			case PUTKFS_PUT_IN:
			{
				user.Set_Z_Td(500.f, 314.16);
				
				user.Set_X(PUTKFS_PUT_KFS_IN_X);
				user.Set_Z(put_z);
				user.Set_P(PI);
				
				if (
					fabsf(user.Get_X() - PUTKFS_PUT_KFS_IN_X) 	  < PUTKFS_POS_THRESTHOLD_SMALL &&
					fabsf(user.Get_Z() - put_z)                   < PUTKFS_POS_THRESTHOLD_SMALL &&
					fabsf(user.Get_P() - PI)                      < PUTKFS_ANG_THRESTHOLD
				)
				{
					put_state = PUTKFS_PUT_DOWN;
				}
				break;
			}
			
			case PUTKFS_PUT_DOWN:
			{
				user.Set_Z(put_z - PUTKFS_PUT_KFS_DOWN_DELTA_Z);
				
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
				
				if (
					timer::Timer::Get_DeltaTime(last_time) > 500000
				)
				{
					put_state = PUTKFS_PUT_OUT;
				}
				break;
			}
			
			case PUTKFS_PUT_OUT:
			{
				user.Set_X(PUTKFS_PUT_KFS_OUT_X);
				
				if (
					fabsf(user.Get_X() - PUTKFS_PUT_KFS_OUT_X) < PUTKFS_POS_THRESTHOLD_SMALL
				)
				{
					return true;
				}
				break;
			}
			
			default:
			{
				put_state = PUTKFS_PUT_CLOSE_TO;
				break;
			}
		}
		
		return false;
	}
}
