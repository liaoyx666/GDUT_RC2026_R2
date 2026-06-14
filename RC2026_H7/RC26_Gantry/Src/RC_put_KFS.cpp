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
	
	PutKFS::PutKFS(Gantry& gan_, Suction& suck_, mini_laser::MiniLaser& laser_, path::Navigation& navi_)
	 : 	user(gan_), 
		suck(suck_), 
		put_event{
			path::Event3(17, 1.f, false, false),		//EVENT_PUT_KFS_2L_READY
			path::Event3(18, 1.f, false, false),   		//EVENT_PUT_KFS_3L_READY
			path::Event3(19, 0.03f, true, true) 	 	//EVENT_PUT_KFS_PUT
		},
		laser(laser_),
		navi(navi_)
	{
		phase = PUTKFS_RESET;
		last_time = 0;
		last_check_time = 0;
		ready_trig = false;
		is_fail = false;
		fail_num = 0;
		is_check = false;
		height = PUTKFS_2L;
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
					is_check = true;
					height = PUTKFS_2L;
				}
				else if (put_event[1].Is_Trig())
				{
					ready_trig = true;
					put_z = PUTKFS_3L_Z;
					is_check = false;
					height = PUTKFS_3L;
				}
				
				if (ready_trig)
				{
					if (!user.Take_Control()) return;
					
					ready_trig = false;
					
					if (data::KFS_Num() == 0)
					{
						return;
					}
					else if (data::KFS_Num() == 1)
					{
						get_z = PUTKFS_GET_KFS_LOW_Z;
						get_state = PUTKFS_GET_STRETCH;
						
						phase = PUTKFS_GET_PHASE;
					}
					else if (data::KFS_Num() == 2)
					{
						get_z = PUTKFS_GET_KFS_HIGH_Z;
						get_state = PUTKFS_GET_STRETCH;
						
						phase = PUTKFS_GET_PHASE;
					}
					else
					{
						get_state = PUTKFS_GET_KFS_OUT;
						
						phase = PUTKFS_GET_PHASE;
					}
					
					// 上次放失败
					if (put_state == PUTKFS_PUT_CHECK_SUDOKU_FAIL)
					{
						get_state = PUTKFS_GET_KFS_OUT;
					}
					
					put_state = PUTKFS_PUT_CHECK_SUDOKU;
					
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
	constexpr float PUTKFS_GET_KFS_OUT_X = 0.27;
	constexpr float PUTKFS_PUT_KFS_OUT_DELTA_Z = 0.06;
	
	
	constexpr float PUTKFS_PUT_KFS_CLOSE_X = 0.2;
	constexpr float PUTKFS_PUT_KFS_IN_X = 0.43;
	constexpr float PUTKFS_PUT_KFS_DOWN_DELTA_Z = 0.03;
	constexpr float PUTKFS_PUT_KFS_OUT_X = 0.32;
	
	constexpr float PUTKFS_PUT_KFS_CLOSE_P = 2.1;
	constexpr float PUTKFS_PUT_KFS_CLOSE_DELTA_Z = 0.3;
	
	
	
	constexpr float PUTKFS_PUT_KFS_IN_X_3L = 0.42;
	constexpr float PUTKFS_PUT_KFS_IN_P_3L = 2.6;
	constexpr float PUTKFS_PUT_KFS_IN_Z_3L = 0.86;
	
	bool PutKFS::Get_KFS_Phase()
	{
		switch (get_state)
		{
			case PUTKFS_GET_STRETCH:
			{
				suck.Off();
				
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
				
				if (is_check)
				{
					user.Set_X(PUTKFS_GET_KFS_OUT_X);
					user.Set_Z(put_z - PUTKFS_PUT_KFS_OUT_DELTA_Z);
					user.Set_P(HALF_PI);
					
					get_state = PUTKFS_GET_KFS_OUT_CHECK;
				}
				else
				{
					user.Set_X(PUTKFS_PUT_KFS_CLOSE_X);
					user.Set_Z(put_z - PUTKFS_PUT_KFS_CLOSE_DELTA_Z);
					user.Set_P(PUTKFS_PUT_KFS_CLOSE_P);

					return true;
				}
				break;
			}
			
			case PUTKFS_GET_KFS_OUT_CHECK:
			{
				if (
					fabsf(user.Get_X() - PUTKFS_GET_KFS_OUT_X) < PUTKFS_POS_THRESTHOLD_SMALL && 
					fabsf(user.Get_Z() - (put_z - PUTKFS_PUT_KFS_OUT_DELTA_Z)) < PUTKFS_POS_THRESTHOLD_SMALL && 
					fabsf(user.Get_P() - HALF_PI) < PUTKFS_ANG_THRESTHOLD_SMALL
				)
				{
					return true;
				}
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
	
	
	

	
	bool PutKFS::Put_KFS_Phase()
	{
		switch (put_state)
		{
			case PUTKFS_PUT_CHECK_SUDOKU:
			{
				last_time = last_time = timer::Timer::Get_TimeStamp();
				last_check_time = last_time = timer::Timer::Get_TimeStamp();
				put_state = PUTKFS_PUT_CHECK_SUDOKU_CHECK;
				break;
			}
			
			
			
			case PUTKFS_PUT_CHECK_SUDOKU_CHECK:
			{
				// 小于阈值更新
				if (laser.distance < 350)
				{
					last_check_time = timer::Timer::Get_TimeStamp();
				}
				
				if (!is_check)
				{
					put_state = PUTKFS_PUT_DROP;
				}
				else if (timer::Timer::Get_DeltaTime(last_check_time) > 400000) // 0.4s没检测到就成功
				{
					put_state = PUTKFS_PUT_DROP;
				}
				else if (timer::Timer::Get_DeltaTime(last_time) > 1000000) // 2s超时
				{
					put_state = PUTKFS_PUT_CHECK_SUDOKU_FAIL;
				}
				break;
			}

			/*--------------------------------------------*/
			
			case PUTKFS_PUT_CHECK_SUDOKU_FAIL:
			{
				is_fail = true;
				fail_num++;
				return true;
				break;
			}
			
			/*--------------------------------------------*/
			
			
			
			
			case PUTKFS_PUT_DROP:
			{
				user.Set_X(PUTKFS_PUT_KFS_CLOSE_X);
				user.Set_Z(put_z - PUTKFS_PUT_KFS_CLOSE_DELTA_Z);
				user.Set_P(PUTKFS_PUT_KFS_CLOSE_P);
				
				put_state = PUTKFS_PUT_DROP_CHECK;
				break;
			}
			
			case PUTKFS_PUT_DROP_CHECK:
			{
				if (
					fabsf(user.Get_Z() - (put_z - PUTKFS_PUT_KFS_CLOSE_DELTA_Z))     < PUTKFS_POS_THRESTHOLD_SMALL &&
					fabsf(user.Get_P() - PUTKFS_PUT_KFS_CLOSE_P)                     < PUTKFS_ANG_THRESTHOLD_SMALL
				)
				{
					if (height == PUTKFS_2L)
					{
						put_state = PUTKFS_PUT_IN;
					}
					else
					{
						put_state = PUTKFS_PUT_IN_3L;
					}
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
					timer::Timer::Get_DeltaTime(last_time) > 900000
				)
				{
					user.Set_Defualt_Td();
					user.Set_Reset_Pos();
					data::KFS_Sub_One();// 放成功
					user.Give_Control();
					return true;
				}
				break;
			}
			
			/*--------------------------------------------*/
			
			
			case PUTKFS_PUT_IN_3L:
			{
				user.Set_X(PUTKFS_PUT_KFS_IN_X_3L);
				user.Set_Z(PUTKFS_PUT_KFS_IN_Z_3L);
				user.Set_P(PUTKFS_PUT_KFS_IN_P_3L);
				
				put_state = PUTKFS_PUT_IN_CHECK_3L;
				break;
			}
			
			case PUTKFS_PUT_IN_CHECK_3L:
			{
				if (
					fabsf(user.Get_X() - PUTKFS_PUT_KFS_IN_X_3L) < PUTKFS_POS_THRESTHOLD_SMALL &&
					fabsf(user.Get_Z() - PUTKFS_PUT_KFS_IN_Z_3L) < PUTKFS_POS_THRESTHOLD_SMALL &&
					fabsf(user.Get_P() - PUTKFS_PUT_KFS_IN_P_3L) < PUTKFS_ANG_THRESTHOLD_SMALL
				)
				{
					put_state = PUTKFS_PUT_RELESE_3L;
				}
				
				
				break;
			}
		
			
			
			
			case PUTKFS_PUT_RELESE_3L:
			{
				suck.Off();
				last_time = timer::Timer::Get_TimeStamp();
				
				put_state = PUTKFS_PUT_RELESE_CHECK_3L;

				break;
			}
			
			case PUTKFS_PUT_RELESE_CHECK_3L:
			{	
				if (
					timer::Timer::Get_DeltaTime(last_time) > 900000
				)
				{
					user.Set_Defualt_Td();
					user.Set_Reset_Pos();
					data::KFS_Sub_One();// 放成功
					user.Give_Control();
					return true;
				}
				break;
			}
				
			/*--------------------------------------------*/
			default:
			{
				put_state = PUTKFS_PUT_CHECK_SUDOKU;
				break;
			}
		}
		
		return false;
	}
}
